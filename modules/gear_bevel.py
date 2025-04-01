from math import asin, acos, atan, tan, sin, cos, pi, atan2, sqrt, ceil, log
from collections.abc import Iterable
from typing import Literal, cast

import adsk.core, adsk.fusion

from .lib import fusion_helper as fh
from .lib.fusion_helper import Vector
from .lib.function import minimize


def gear_bevel(
    axes_angle: float,
    module: float,
    z1: int,
    z2: int,
    width: float,
    beta: float,
    addendum: float,
    dedendum: float,
    pressure_angle: float,
    backlash: float,
):
    m, sigma, alpha, mk, mf = (
        module,
        axes_angle,
        pressure_angle,
        addendum,
        dedendum,
    )

    # calculate transverse values for spiral gears
    m *= 1 / cos(beta)
    mf *= cos(beta)
    mk *= cos(beta)
    alpha = atan(tan(alpha) / cos(beta))

    # basic dimensions
    gamma_p1 = atan2(sin(sigma), z2 / z1 + cos(sigma))
    r = z1 / (2 * sin(gamma_p1))
    r0 = r * m
    rm = 1 / r

    gamma_f1 = gamma_p1 - mf * atan(rm)
    gamma_k1 = gamma_p1 + mk * atan(rm)
    gamma_t1 = gamma_p1 + mf * atan(rm)
    gamma_b1 = asin(cos(alpha) * sin(gamma_p1))

    gamma_p2 = sigma - gamma_p1
    gamma_f2 = gamma_p2 - mf * atan(rm)
    gamma_k2 = gamma_p2 + mk * atan(rm)
    gamma_t2 = gamma_p2 + mf * atan(rm)
    gamma_b2 = asin(cos(alpha) * sin(gamma_p2))

    axis1 = Vector(cos(gamma_p1), 0, sin(gamma_p1))
    axis2 = Vector(cos(gamma_p2), 0, -sin(gamma_p2))

    def rotate(v: Vector, axis: Vector, angle: float):
        """rotate v around axis by angle (in radians)"""
        axis = axis.normalize()
        cos_a = cos(angle)
        sin_a = sin(angle)
        dot = v.dot(axis)
        cross = v.cross(axis)
        return v * cos_a + cross * sin_a + axis * dot * (1 - cos_a)

    def trans(epsilon: float, v: Vector, axis: Vector, gamma_b: float):
        """transform v along base circle by angle specified by epsilon
        and then pushed back along the great circle by same distance"""
        axis = axis.normalize(cos(gamma_b))
        v = v.normalize()
        oq = v - axis
        om = rotate(oq, axis, epsilon)
        oom = axis + om
        axis2 = oom - axis.normalize(1 / cos(gamma_b))
        op = rotate(oom, axis2, epsilon * sin(gamma_b))
        return op

    def gamma2theta(gamma: float, gamma_b: float):
        varphi = acos(cos(gamma) / cos(gamma_b))
        return varphi / sin(gamma_b) - atan2(tan(varphi), sin(gamma_b))

    def gamma2epsilon(gamma: float, gamma_b: float):
        varphi = acos(cos(gamma) / cos(gamma_b))
        return varphi / sin(gamma_b)

    def spherical_involute(
        axis: Vector, v: Vector, gamma_b: float, gamma_s: float, gamma_e: float, n=20
    ):
        points: list[Vector] = []
        ts = gamma2epsilon(gamma_s, gamma_b)
        te = gamma2epsilon(gamma_e, gamma_b)
        for i in range(0, n + 1):
            t = ts + ((te - ts) / n) * i
            points.append(trans(t, v, axis, gamma_b))
        return points

    def spherical_trochoid(
        t1: Vector,
        q2: Vector,
        axis1: Vector,
        axis2: Vector,
        z1: float,
        z2: float,
        tooth2: list[Vector],
        gamma_b2: float,
        gamma_f2: float,
        n=10,
    ):
        def trochoid_core(t: float):
            return rotate(rotate(t1, axis1, -t), axis2, (-t / z2) * z1)

        # after touching the bottom circle, reach the tip circle
        c1: list[Vector] = []
        i = 0
        while True:
            t = (0.02 * (i * pi)) / sqrt(z1)
            c1.append(trochoid_core(t))
            if (c1[-1] - axis2).norm() > (tooth2[-1] - axis2).norm():
                # find the touching point to the bottom circle
                t1a = minimize(0, t, lambda t: (trochoid_core(t) - axis2).norm())
                # the point higher than the tip circle
                t1b = t
                break
            i += 1

        if gamma_b2 > gamma_f2:
            # find the point the trochoid curve crosses the base circle
            t1c = minimize(
                t1a, t1b, lambda t: abs((trochoid_core(t) - axis2).norm() - (q2 - axis2).norm())
            )
        else:
            # bottom circle
            t1c = t1a

        def distance_from_involute(t: float):
            """Measure the distance from the involute curve at the same gamma value"""
            v = trochoid_core(t)
            gamma = acos(v.dot(axis2))
            u = trans(gamma2epsilon(gamma, gamma_b2), q2, axis2, gamma_b2)
            return (u - v).norm()

        # find the point on the involute curve that is closest to the trochoid curve
        t1d = minimize(t1c, t1b, distance_from_involute)

        # use the part from bottom circle (t1a) to cross point with involute (t1d)
        c1.clear()
        for i in range(0, n + 1):
            t = t1a + ((t1d - t1a) * i) / n
            c1.append(trochoid_core(t))

        gamma = acos(c1[-1].dot(axis2))  # for cross point with involute curve
        return (c1, gamma)

    def apply_backlash(curve: list[Vector], axis: Vector, z: float):
        def angle(p: Vector):
            return acos(
                (p - axis.normalize(p.dot(axis)))
                .normalize()
                .dot((Vector(1, 0, 0) - axis.normalize(Vector(1, 0, 0).dot(axis))).normalize())
            )

        # scale by r0 and apply backlash
        curve = [rotate(r0 * p, axis, backlash / (m * z / 2)) for p in curve]

        # if tip or bottom are overlapped, remove the part
        if angle(curve[0]) > pi / z / 2:
            while angle(curve[0]) > pi / z / 2:
                curve.pop(0)
        if angle(curve[-1]) > pi / z / 2:
            while angle(curve[-1]) > pi / z / 2:
                curve.pop()
        return curve

    def generate_arc(
        axis: Vector,
        p1: Vector,
        p2: Vector,
        great_arc=False,
        n=6,
    ):
        """Generate an arc between two points on a sphere."""
        c: list[Vector] = []
        if great_arc:
            v1 = p1
            v2 = p2
            t = acos(v1.dot(v2) / (v1.norm() * v2.norm()))
            for j in range(0, n + 1):
                t2 = (t * j) / n
                c.append(rotate(v1, axis, t2))
        else:
            axis = axis.normalize(axis.normalize().dot(p1))
            v1 = p1 - axis
            v2 = p2 - axis
            t = acos(v1.dot(v2) / (v1.norm() * v2.norm()))
            for j in range(0, n + 1):
                t2 = (t * j) / n
                c.append(axis + rotate(v1, axis, t2))
        return c

    def gear_curves():
        """Generate the involute and trochoid curves for the two gears."""

        # base point on the base circle
        q1 = rotate(
            Vector(cos(gamma_p1 - gamma_b1), 0, sin(gamma_p1 - gamma_b1)),
            axis1,
            -gamma2theta(gamma_p1, gamma_b1),
        )
        involute1 = spherical_involute(axis1, q1, gamma_b1, max(gamma_b1, gamma_f1), gamma_k1)
        # tip point on the extended tip circle
        t1 = trans(gamma2epsilon(gamma_t1, gamma_b1), q1, axis1, gamma_b1)

        # base point on the base circle
        q2 = rotate(
            Vector(cos(gamma_p2 - gamma_b2), 0, -sin(gamma_p2 - gamma_b2)),
            axis2,
            -gamma2theta(gamma_p2, gamma_b2),
        )
        involute2 = spherical_involute(axis2, q2, gamma_b2, max(gamma_b2, gamma_f2), gamma_k2)
        # tip point on the extended tip circle
        t2 = trans(gamma2epsilon(gamma_t2, gamma_b2), q2, axis2, gamma_b2)

        trochoid1, gamma1 = spherical_trochoid(
            t2, q1, axis2, axis1, z2, z1, involute1, gamma_b1, gamma_f1
        )
        involute1 = spherical_involute(axis1, q1, gamma_b1, gamma1, gamma_k1 + rm * 0.2)

        trochoid2, gamma2 = spherical_trochoid(
            t1, q2, axis1, axis2, z1, z2, involute2, gamma_b2, gamma_f2
        )
        involute2 = spherical_involute(axis2, q2, gamma_b2, gamma2, gamma_k2 + rm * 0.2)

        return trochoid1, involute1, trochoid2, involute2

    def tooth_profile(trochoid: list[Vector], involute: list[Vector], axis: Vector, z: float):
        trochoid1 = apply_backlash(trochoid, axis, z)
        involute1 = apply_backlash(involute, axis, z)
        involute2 = [rotate(p.flip_y(), axis, -pi / z) for p in involute1]
        trochoid2 = [rotate(p.flip_y(), axis, -pi / z) for p in trochoid1]
        bottom = generate_arc(axis, trochoid2[0], trochoid1[0])
        top = generate_arc(axis, involute2[-1], involute1[-1])
        return [
            reversed(involute1),
            reversed(trochoid1),
            reversed(bottom),
            trochoid2,
            involute2,
            top,
        ]

    def connect_by_splines(sketch: adsk.fusion.Sketch, curves: list[Iterable[Vector]]):
        """Generate splines and connect them with each other to make a closed loop."""

        result: list[adsk.fusion.SketchFittedSpline] = []
        for i, curve in enumerate(curves):
            points = [fh.point3d(p) for p in curve]
            if i == 0:
                pass
            else:
                points[0] = result[i - 1].endSketchPoint
                if i == len(curves) - 1:
                    points[-1] = result[0].startSketchPoint
            result.append(sketch.sketchCurves.sketchFittedSplines.add(fh.collection(points)))
        return result

    def line(
        sketch: adsk.fusion.Sketch, p1: Vector | adsk.core.Point3D, p2: Vector | adsk.core.Point3D
    ):
        """Add a line by two points, accepting both Vector and Point3D types."""
        if isinstance(p1, Vector):
            p1 = fh.point3d(p1)
        if isinstance(p2, Vector):
            p2 = fh.point3d(p2)
        return sketch.sketchCurves.sketchLines.addByTwoPoints(
            cast(adsk.core.Point3D, p1), cast(adsk.core.Point3D, p2)
        )

    def generate_gear(
        occurrence: adsk.fusion.Occurrence,
        axis: Vector,
        z: int,
        trochoid: list[Vector],
        involute: list[Vector],
        flip: Literal[1, -1],
    ):
        comp = occurrence.component

        # generate a patch with the tooth groove shape
        sketch1 = comp.sketches.add(comp.xYConstructionPlane)
        sketch1.isComputeDeferred = True
        connect_by_splines(sketch1, tooth_profile(trochoid, involute, axis, z))
        patch = fh.comp_patch(comp, sketch1.sketchCurves[0], fh.FeatureOperations.new_body)
        for c in sketch1.sketchCurves:
            c.isFixed = True
        for p in sketch1.sketchPoints:
            p.isFixed = True
        sketch1.isVisible = False
        sketch1.isComputeDeferred = False

        # generate the base body of the gear
        sketch2 = comp.sketches.add(comp.xYConstructionPlane)
        sketch2.isComputeDeferred = True
        va = Vector(r0, 0, -mk * m * flip)
        vb = Vector(r0, 0, (mf + 1) * m * flip)
        vas = va * ((r0 - width) / r0)
        vbs = vb * ((r0 - width) / r0)
        la = line(sketch2, va, vb)
        lb = line(sketch2, vas, vbs)
        line(sketch2, la.startSketchPoint, lb.startSketchPoint)
        vax0 = axis.normalize(r0 * sqrt(1 - (rm * z / 2) ** 2))
        vax1 = axis.normalize(axis.normalize().dot(vb))
        vax2 = axis.normalize(axis.normalize().dot(vbs))
        lc = line(sketch2, la.endSketchPoint, vax1)
        ld = line(sketch2, lb.endSketchPoint, vax2)
        le = line(sketch2, lc.endSketchPoint, ld.endSketchPoint)
        for c in sketch2.sketchCurves:
            c.isFixed = True
        for p in sketch2.sketchPoints:
            p.isFixed = True
        sketch2.isComputeDeferred = False

        body = fh.comp_revolve(comp, sketch2.profiles[0], le, fh.FeatureOperations.new_body)

        # refresh the viewport
        adsk.doEvents()
        adsk.core.Application.get().activeViewport.refresh()

        def spiral_angle(scale: float):
            return flip * 2 * r * tan(beta) / z * log(scale)

        # determine the lofting range of the tooth groove shape
        scale_start = 1.02
        scale_end = (r0 - width) / r0 * 0.98
        scale_n = 1 if beta == 0 else max(5, ceil(abs(spiral_angle(scale_end)) / pi * 20))

        # create the tooth patches for lofting by scaling and rotating the original patch
        patches: list[adsk.fusion.BRepBody] = []
        for i in range(scale_n + 1):
            scale = scale_start * (scale_end / scale_start) ** (i / scale_n)
            copy = fh.comp_copy(comp, patch.bodies[0])
            fh.comp_scale(comp, copy.bodies[0], comp.originConstructionPoint, scale)
            if (scale - 1) * tan(beta) != 0:
                fh.comp_move_rotate(
                    comp,
                    copy.bodies[0],
                    le.createForAssemblyContext(occurrence),
                    spiral_angle(scale),
                )
            patches.append(copy.bodies[0])

        # loft the patches to create the tooth shape and circular pattern it
        tooth = fh.comp_loft(
            comp, fh.FeatureOperations.cut, [p.faces[0] for p in patches], [body.bodies[0]]
        )
        fh.comp_remove(comp, patches + [patch.bodies[0]])
        fh.comp_circular_pattern(comp, tooth, le, z)

        # refresh the viewport
        adsk.doEvents()
        adsk.core.Application.get().activeViewport.refresh()

        # return the axis of the gear for joint creation
        return vax0, vax1

    trochoid1, involute1, trochoid2, involute2 = gear_curves()

    app = adsk.core.Application.get()
    design = adsk.fusion.Design.cast(app.activeProduct)

    # Create the wrapper component
    comp_occurrence = design.activeComponent.occurrences.addNewComponent(
        fh.matrix_rotate(pi / 2 - gamma_p2, fh.vector3d(0, 1, 0))
    )
    comp_occurrence.isGroundToParent = False
    comp = comp_occurrence.component
    # pylint: disable=inconsistent-quotes
    comp.name = (
        f"Bevel{format(round(m*10,2),"g")}M{z1}T{z2}T{format(round(sigma/pi*180,2), 'g')}deg"
    )

    # generate the first gear
    gear1_occurrence = comp.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    gear1_occurrence.isGroundToParent = False
    gear1_occurrence = gear1_occurrence.createForAssemblyContext(comp_occurrence)
    gear1 = gear1_occurrence.component
    gear1.name = f"bevel{format(round(m*10,2),"g")}M{z1}T{format(round(sigma/pi*180,2), "g")}deg"
    axis1p = generate_gear(gear1_occurrence, axis1, z1, trochoid1, involute1, 1)

    # generate the second gear
    gear2_occurrence = comp.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    gear2_occurrence = gear2_occurrence.createForAssemblyContext(comp_occurrence)
    gear2 = gear2_occurrence.component
    gear2.name = f"bevel{format(round(m*10,2),"g")}M{z2}T{format(round(sigma/pi*180,2), "g")}deg"
    axis2p = generate_gear(gear2_occurrence, axis2, z2, trochoid2, involute2, -1)

    # draw the sphere on xz plane to check the size and position of gears
    sketch0 = comp.sketches.add(comp.xZConstructionPlane)
    circ0 = sketch0.sketchCurves.sketchCircles.addByCenterRadius(fh.point3d(), r0)
    circ0.isConstruction = True
    circ0.isFixed = True
    sketch0.isVisible = False

    # draw the axes of the two gears for joint creation
    sketch1 = comp.sketches.add(comp.xYConstructionPlane)
    axis1l = line(sketch1, axis1p[0], axis1p[1])
    axis2l = line(sketch1, axis2p[0], axis2p[1])

    # draw the reference circle for the 1st gear
    # it is used to create the joint between the gear and the wrapper component
    inp = comp.constructionPlanes.createInput(comp_occurrence)
    inp.setByDistanceOnPath(axis1l, fh.value_input(0))
    plane1 = comp.constructionPlanes.add(inp)
    sketch2 = comp.sketches.add(plane1, comp_occurrence)
    circ1 = sketch2.sketchCurves.sketchCircles.addByCenterRadius(fh.point3d(), m * z1 / 2)
    circ1.isFixed = True
    sketch2.isVisible = False

    fh.comp_built_joint_revolute(
        comp,
        gear1_occurrence,
        comp_occurrence,
        sketch2.profiles[0], # the circle
        cast(adsk.fusion.JointDirections, adsk.fusion.JointDirections.ZAxisJointDirection),
    )

    # draw the reference circle for the 2nd gear
    # it is used to create the joint between the gear and the wrapper component
    inp = comp.constructionPlanes.createInput(comp_occurrence)
    inp.setByDistanceOnPath(axis2l, fh.value_input(0))
    plane2 = comp.constructionPlanes.add(inp)
    sketch3 = comp.sketches.add(plane2, comp_occurrence)
    circ2 = sketch3.sketchCurves.sketchCircles.addByCenterRadius(fh.point3d(), m * z2 / 2)
    circ2.isFixed = True
    sketch3.isVisible = False

    fh.comp_built_joint_revolute(
        comp,
        gear2_occurrence,
        comp_occurrence,
        sketch3.profiles[0], # the circle
        cast(adsk.fusion.JointDirections, adsk.fusion.JointDirections.ZAxisJointDirection),
    )

    # # creating motion link is not supported in API
    # # https://forums.autodesk.com/t5/fusion-api-and-scripts/adding-motion-link-to-the-fusion-360-api/td-p/11728121
