from collections.abc import Callable
from copy import copy
from dataclasses import dataclass
from functools import reduce
from math import pi, cos, asin, atan, tan, floor, ceil

import adsk.core, adsk.fusion

from .lib.function import minimize, find_root

from . import gear_curve
from .lib import fusion_helper as fh
from .lib.fusion_helper import Vector, vec, radius_from_3points
from .gear_worm_wheel_segment import Line, Arc, Segments
from .lib.spline import evenly_spaced_points_on_spline, interpolate


def gear_worm_wheel(
    parent_occurrence: adsk.fusion.Occurrence,
    params: gear_curve.GearParams,
    worm_diameter: float,
    worm_spirals: int,
    thickness: float,
    helix_angle: float,
):
    parent = parent_occurrence.component
    occurrence = parent.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    occurrence = occurrence.createForAssemblyContext(parent_occurrence)
    occurrence.isGroundToParent = False
    comp = occurrence.component

    rk = params.m / cos(helix_angle) * params.z / 2 + params.m * params.mk
    camera_backup = fh.camera_setup(
        Vector(x=rk + max(thickness, pi * params.m) * 4),
        Vector(),
        Vector(z=1),
        perspective=pi / 8,
        occurrence=occurrence,
    )

    sketch = comp.sketches.add(comp.xYConstructionPlane)
    sketch = sketch.createForAssemblyContext(occurrence)
    sketch.isComputeDeferred = True

    # generate the groove profiles for height z
    n = max(4, ceil(thickness / params.m / cos(helix_angle) * 1.5))
    for i in range(n):
        z = (n - 1 - i) / (n - 1) * thickness * 1.005 / 2
        shape = worm_wheel_shape_at_height(z, params, helix_angle, worm_diameter / 2, worm_spirals)
        if len(shape) == 0:
            continue
        shape = evenly_spaced_points_on_spline(shape, 50)
        spline = fh.sketch_fitted_splines(sketch, [Vector(v.x, v.y, z) for v in shape])
        sketch.sketchCurves.sketchArcs.addByCenterStartEnd(
            fh.point3d(0, 0, z),
            spline.endSketchPoint,
            spline.startSketchPoint,
        )
        fh.app_refresh()
    fh.sketch_fix_all(sketch)
    sketch.isComputeDeferred = False

    # generate patch
    # We generate the patch from curve because generating patch from profile sometimes fails.
    curves = sorted(sketch.sketchCurves, key=lambda c: c.boundingBox.minPoint.z)
    patches = [
        fh.comp_patch(comp, curves[i * 2 : (i + 1) * 2], fh.FeatureOperations.new_body).bodies[0]
        for i in range(1, floor(len(curves) / 2))
    ]
    sketch.isVisible = False

    # copy it to the other side with rotation
    for i, patch in enumerate(patches.copy()):
        if patch.boundingBox.minPoint.z > 0:
            patch2 = fh.comp_mirror(comp, patch, comp.xZConstructionPlane).bodies[0]
            fh.comp_move_free(
                comp,
                patch2,
                fh.matrix_translate(z=-patch.boundingBox.minPoint.z * 2),
            )
            patches.insert(0, patch2)

    # create a disk by extruding the tip circle
    sketch2 = comp.sketches.add(comp.xYConstructionPlane)
    sketch2.sketchCurves.sketchCircles.addByCenterRadius(fh.point3d(), rk)
    disk = fh.comp_extrude(
        comp,
        sketch2.profiles[0],
        fh.FeatureOperations.new_body,
        thickness,
        True,
        True,
    )

    # cut a groove
    teeth = fh.comp_loft(
        comp,
        fh.FeatureOperations.cut,
        [p.faces[0] for p in patches],
        disk.bodies[0],
    )
    fh.comp_remove(comp, patches)

    # copy it around the axis
    fh.comp_circular_pattern(comp, teeth, comp.zConstructionAxis, round(params.z))

    # draw axis
    sketch3 = comp.sketches.add(comp.xYConstructionPlane)
    center_axis = sketch3.sketchCurves.sketchLines.addByTwoPoints(
        fh.point3d(z=-thickness / 2), fh.point3d(z=thickness / 2)
    )
    center_axis.isFixed = True

    # draw reference circle
    rp = params.m / cos(helix_angle) * params.z / 2
    circle = sketch3.sketchCurves.sketchCircles.addByCenterRadius(fh.point3d(), rp)
    circle.isConstruction = True
    sketch3.sketchDimensions.addDiameterDimension(circle, fh.point3d(rp, rp), isDriving=True)
    for p in sketch3.sketchPoints:
        p.isFixed = True

    fh.camera_setup(camera_backup)


def worm_wheel_shape_at_height(
    height: float, params: gear_curve.GearParams, beta: float, worm_r: float, n_spiral: int  #
) -> list[Vector]:
    """Worm wheel groove shape at the specified `height` with helical angle `beta`,
    worm radius `worm_r` and number of spiral `n_spiral`."""
    cos_beta = cos(beta)
    params = copy(params)
    params.m /= cos(beta)
    params.alpha = atan(tan(params.alpha) / cos_beta)
    params.mf *= cos_beta
    params.mk *= cos_beta

    extension = 0.3
    rp = params.m * params.z / 2  # pitch radius
    rk = params.m * (params.z / 2 + params.mk + extension)  # addendum circle radius
    eps = params.m / 1000

    n_worm_curve = 200  # larger value give better approximation
    curvature_limit = 1 / 20  # smaller value give better approximation

    def generate_rack_geometry() -> Segments:
        """Position the rack shape at the symmetric position
        with fillet at the tip."""
        (r1, _, center, radius) = gear_curve.rack_geometry(
            params.m,
            params.alpha,
            params.fillet,
            params.mf,
            1.1 * params.mk,
            params.rc,
            params.backlash,
            params.z,
            params.shift,
        )
        r1 += vec(0, (pi * params.m) / 4)
        center += vec(0, (pi * params.m) / 4)
        r3 = center + Vector.polar(radius, params.alpha + pi / 2)
        r4 = center + Vector.polar(radius, pi)
        if abs(center.y) < eps:
            # draw one arc if fillet circle is on the center
            return Segments(
                Line(r1, r3),
                Arc(
                    center,
                    radius,
                    params.alpha + pi / 2,
                    2 * pi - (params.alpha + pi / 2),
                ),
                Line(r3.flip_y(), r1.flip_y()),
            )

        return Segments(
            # two arcs and bottom line if fillet circle is not on the center
            Line(r1, r3),
            Arc(center, radius, params.alpha + pi / 2, pi),
            Line(r4, r4.flip_y()),
            Arc(center.flip_y(), radius, pi, 2 * pi - (params.alpha + pi / 2)),
            Line(r3.flip_y(), r1.flip_y()),
        )

    # calc the worm curve at the height
    def calc_worm_curve(n_points: int):
        rack_geometry = generate_rack_geometry()
        worm_axis_x = (params.z / 2 + params.shift) * params.m + worm_r
        trans_coef = pi * params.m * n_spiral / (2 * pi)
        if beta < 0:
            trans_coef *= -1

        def translate(v: Vector):
            ratio = height / (v.x - worm_axis_x)
            ratio = max(-1, min(1, ratio))  # limit to avoid asin error
            t = -asin(ratio)
            return vec(
                worm_axis_x + (v.x - worm_axis_x) * cos(t),
                v.y + trans_coef * t,
            )

        return rack_geometry.to_curve(n_points, translate)

    @dataclass
    class TranslatedCurve:
        t: float
        curve: Segments
        spline: Callable[[float], Vector]
        length: float
        closest: float
        closest_p: Vector

    def generate_translated_curves() -> list[TranslatedCurve]:
        worm_curve = calc_worm_curve(n_worm_curve)

        # rotate the worm around the wheel
        def translate_rotate(theta: float):
            c = worm_curve.duplicate()
            c.translate(vec(0, theta * rp))  # worm translation
            c.rotate(-theta)  # wheel rotation

            translated = c.translated()
            ts = [cp[1] for cp in c.points]
            xs = [cp.x for cp in translated]
            ys = [cp.y for cp in translated]
            spline_x = interpolate(ts, xs)
            spline_y = interpolate(ts, ys)
            spline = lambda t: vec(spline_x(t), spline_y(t))

            length = c.points[-1][1]
            closest = minimize(0, length, lambda t: spline(t).norm(), eps)
            closest_p = spline(closest)
            return TranslatedCurve(theta, c, spline, length, closest, closest_p)

        # collect translated curves
        curves: list[TranslatedCurve] = []
        dt = pi / params.z / 2
        curves.append(translate_rotate(0))
        curves.append(translate_rotate(dt))

        while curves[0].closest_p.norm() < curves[1].closest_p.norm():
            curves.insert(0, translate_rotate(curves[0].t - dt))
        while curves[0].closest_p.norm() < rk:
            curves.insert(0, translate_rotate(curves[0].t - dt))
        while curves[-1].closest_p.norm() < curves[-2].closest_p.norm():
            curves.append(translate_rotate(curves[-1].t + dt))
        while curves[-1].closest_p.norm() < rk:
            curves.append(translate_rotate(curves[-1].t + dt))

        # add point where interval is too large compared to curvature radius
        i = 1
        while i < len(curves) - 1:
            p1 = curves[i - 1].spline(curves[i - 1].length / 2)
            p2 = curves[i].spline(curves[i].length / 2)
            p3 = curves[i + 1].spline(curves[i + 1].length / 2)
            v1 = p2 - p1
            v2 = p3 - p2
            r = radius_from_3points(p1, p2, p3)
            limit = max(params.m / 1000, r * curvature_limit)
            if v1.norm() > limit:
                c1 = translate_rotate((curves[i].t + curves[i - 1].t) / 2)
                curves.insert(i, c1)
                continue  # no increment
            elif v2.norm() > limit:
                c2 = translate_rotate((curves[i].t + curves[i + 1].t) / 2)
                curves.insert(i + 1, c2)
                continue  # no increment
            i += 1

        return curves

    curves = generate_translated_curves()

    # radius of bottom circle
    rf = reduce(lambda rf, c: min(rf, c.closest_p.norm()), curves, rk)
    if rf >= rk:
        return []

    # find intersection of wheel groove and circles
    def find_intersections():
        @dataclass
        class Intersection:
            r: float
            a1: float
            a2: float

            def values(self):
                return (self.r, self.a1, self.a2)

        # find intersection with circle of radius r
        def intersect(r: float):
            def reduce_max(a: float, c: TranslatedCurve):
                if c.closest_p.norm() > r:
                    return a
                t = find_root(0, c.closest, lambda t: c.spline(t).norm() - r, eps)
                return max(a, c.spline(t).angle())

            def reduce_min(a: float, c: TranslatedCurve):
                if c.closest_p.norm() > r:
                    return a
                t = find_root(c.closest, c.length, lambda t: c.spline(t).norm() - r, eps)
                return min(a, c.spline(t).angle())

            a1 = reduce(reduce_max, curves, -float("inf"))
            a2 = reduce(reduce_min, curves, float("inf"))
            return Intersection(r, a1, a2)

        # calculate initial_division points of r
        initial_division = 16
        intersections: list[Intersection] = []

        for i in range(1, initial_division + 2):
            r = rf + ((rk - rf) * i) / initial_division
            intersections.append(intersect(r))

        # add points at the bottom part
        intersections.insert(0, intersect(rf + (rk - rf) / (initial_division * 2)))
        intersections.insert(0, intersect(rf + (rk - rf) / (initial_division * 8)))
        intersections.insert(0, intersect(rf + (rk - rf) / (initial_division * 128)))

        # add points where separation is too large compared to curvature radius
        i = 1
        while i < len(intersections) - 1:
            r1, a1, b1 = intersections[i - 1].values()
            r2, a2, b2 = intersections[i].values()
            r3, a3, b3 = intersections[i + 1].values()

            # check upper side
            p1 = Vector.polar(r1, a1)
            p2 = Vector.polar(r2, a2)
            p3 = Vector.polar(r3, a3)
            v1 = p2 - p1
            v2 = p3 - p2
            rr1 = radius_from_3points(p1, p2, p3)
            m = 60
            mm = 10
            if v1.norm() > params.m / m and v1.norm() > rr1 / mm:
                intersections.insert(i, intersect((r1 + r2) / 2))
                continue  # no increment
            if v2.norm() > params.m / m and v2.norm() > rr1 / mm:
                intersections.insert(i + 1, intersect((r2 + r3) / 2))
                continue  # no increment

            # check lower side
            q1 = Vector.polar(r1, b1)
            q2 = Vector.polar(r2, b2)
            q3 = Vector.polar(r3, b3)
            w1 = q2 - q1
            w2 = q3 - q2
            rr2 = radius_from_3points(q1, q2, q3)
            if w1.norm() > params.m / m and w1.norm() > rr2 / mm:
                intersections.insert(i, intersect((r1 + r2) / 2))
                continue  # no increment
            if w2.norm() > params.m / m and w2.norm() > rr2 / mm:
                intersections.insert(i + 1, intersect((r2 + r3) / 2))
                continue  # no increment

            i += 1

        return intersections

    intersections = find_intersections()
    if len(intersections) == 0:
        return []
    result = [Vector.polar(p.r, p.a1) for p in reversed(intersections)]
    result += [Vector.polar(rf, (intersections[0].a1 + intersections[0].a2) / 2)]
    result += [Vector.polar(p.r, p.a2) for p in intersections]
    return result
