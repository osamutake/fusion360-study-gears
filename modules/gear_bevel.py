from math import asin, atan, tan, sin, cos, pi, log, ceil
from collections.abc import Callable
from typing import cast

import adsk.core, adsk.fusion

from . import gear_curve
from .lib import fusion_helper as fh
from .lib.fusion_helper import Vector, vec, polar
from .lib.curve import Curve


def gear_bevel(
    axes_angle: float,
    module: float,
    z1: int,
    z2: int,
    width: float,
    beta: float,
    shift: float,
    addendum: float,
    dedendum: float,
    rc: float,
    pressure_angle: float,
    fillet: float,
    backlash: float,
):
    m, sigma, alphan, betam, b = module, axes_angle, pressure_angle, beta, width

    alphat = atan(tan(alphan) / cos(betam))
    # d1 = z1 * m
    d2 = z2 * m
    delta1 = atan(sin(sigma) / (z2 / z1 + cos(sigma)))
    delta2 = sigma - delta1
    r = d2 / (2 * sin(delta2))
    h = (addendum + dedendum) * m
    ha = addendum * m
    hf = h - ha
    thetaf = atan(hf / r)
    thetaa = atan(ha / r)
    deltaa1 = delta1 + thetaa
    deltaa2 = delta2 + thetaa
    deltaf1 = delta1 - thetaf
    deltaf2 = delta2 - thetaf

    app = adsk.core.Application.get()
    design = adsk.fusion.Design.cast(app.activeProduct)

    # Create the wrapper component
    comp_occurrence = design.activeComponent.occurrences.addNewComponent(
        adsk.core.Matrix3D.create()
    )
    comp = comp_occurrence.component
    # pylint: disable=inconsistent-quotes
    comp.name = f"Bevel{format(round(m*10,2),"g")}M{z1}T{z2}T{format(round((delta1+delta2)/pi*180,2), 'g')}deg"

    def build_gear(
        gear: adsk.fusion.Component,
        deltaa: float,
        delta: float,
        deltaf: float,
        z: int,
        beta: float,
    ):
        # rotational axis is x-axis
        # the other gear's axis is tilted from x-axis towards y-axis
        sketch = gear.sketches.add(gear.xYConstructionPlane)
        sketch.isComputeDeferred = True
        draw_line = sketch.sketchCurves.sketchLines.addByTwoPoints
        extend = 1.2  # extend the line to make it easier to see

        def p3d(p: Vector):
            return fh.point3d(p.x, p.y)

        # center line
        center = draw_line(p3d(vec(0, 0)), p3d(vec(extend * r, 0)))
        center.isCenterLine = True

        # angled lines
        line_a = draw_line(p3d(vec(0, 0)), p3d(polar(extend * r, deltaa)))
        line_p = draw_line(p3d(vec(0, 0)), p3d(polar(extend * r, delta)))
        line_f = draw_line(p3d(vec(0, 0)), p3d(polar(extend * r, deltaf)))
        for line in [line_a, line_p, line_f]:
            line.isConstruction = True

        # base shape
        pp1 = polar(r, delta) + polar(ha, pi / 2 + delta)
        pp0 = pp1 + polar(h + m, -pi / 2 + delta)
        pp2 = polar(r - b, delta) + polar(ha * (r - b) / r, pi / 2 + delta)
        pp3 = pp2 + polar((h + m) * (r - b) / r, -pi / 2 + delta)
        pp4 = vec(pp3.x, 0)
        pp5 = vec(pp0.x, 0)

        lines: list[adsk.fusion.SketchLine] = []

        lines.append(draw_line(p3d(pp0), p3d(pp1)))
        lines.append(draw_line(lines[-1].endSketchPoint, p3d(pp2)))
        lines.append(draw_line(lines[-1].endSketchPoint, p3d(pp3)))
        lines.append(draw_line(lines[-1].endSketchPoint, p3d(pp4)))
        lines.append(draw_line(lines[-1].endSketchPoint, p3d(pp5)))
        lines.append(draw_line(lines[-1].endSketchPoint, lines[0].startSketchPoint))
        for line in sketch.sketchCurves.sketchLines:
            line.isFixed = True

        # rotation
        base = fh.comp_revolve(
            gear, sketch.profiles[0], center, fh.FeatureOperations.new_body
        ).bodies[0]

        inp = gear.constructionPlanes.createInput()
        inp.setByDistanceOnPath(line_p, fh.value_input(1 / extend))
        plane = gear.constructionPlanes.add(inp)

        sketch2 = gear.sketches.add(plane)
        sketch2.isComputeDeferred = True
        curves = gear_curve.gear_curve(
            gear_curve.GearParams(m, z, alphat, shift, fillet, hf / m, ha / m, rc, backlash, False)
        )

        collection = adsk.core.ObjectCollection.createWithArray

        # delta is rotation angle
        def vec2point3d(pnt: Vector, delta=0.0, flip_y=False):
            (r, t) = pnt.to_polar()
            if flip_y:
                return fh.point3d(r * cos(t - delta), r * sin(t - delta + pi), 0)
            else:
                return fh.point3d(r * cos(t + delta), r * sin(t + delta), 0)

        def draw_part(
            sketch: adsk.fusion.Sketch,
            curves: list[tuple[Curve, Vector, Vector]],
            vec2point3d: Callable[[Vector, float, bool], adsk.core.Point3D],
            delta=0,
            last=None,
            flip_y=False,
        ):
            result: list[adsk.fusion.SketchFittedSpline] = []
            for c in reversed(curves) if flip_y else curves:
                # c = (curve, tangent1, tangent2)
                c0 = list(reversed(c[0]) if flip_y else iter(c[0]))
                if last is None:
                    last = vec2point3d(c0[0], delta, flip_y)
                spline = sketch.sketchCurves.sketchFittedSplines.add(
                    collection([last] + [vec2point3d(pp, delta, flip_y) for pp in c0[1:]])
                )
                result.append(spline)
                last = spline.endSketchPoint

                # if flipped, swap the tangent vectors
                if not flip_y:
                    c1, c2 = c[1:3]
                else:
                    c2, c1 = c[1:3]

                # add tangent constraints
                line = sketch.sketchCurves.sketchLines.addByTwoPoints(
                    spline.startSketchPoint, vec2point3d(c0[0] + c1, delta, flip_y)
                )
                line.isConstruction = True
                line.isFixed = True
                sketch.geometricConstraints.addTangent(spline, line)
                line = sketch.sketchCurves.sketchLines.addByTwoPoints(
                    spline.endSketchPoint, vec2point3d(c0[-1] + c2, delta, flip_y)
                )
                line.isConstruction = True
                line.isFixed = True
                sketch.geometricConstraints.addTangent(spline, line)
            return result

        # paste the gear shape onto the cylindrical surface
        for curve in curves:
            for i in range(len(curve[0])):
                curve[0][i].y = r * tan(curve[0][i].y / r)
            curve[1].y = r * tan(curve[1].y / r)
            curve[2].y = r * tan(curve[2].y / r)

        # draw the tooth shape
        lines1 = draw_part(sketch2, curves, vec2point3d, pi / z)
        lines2 = draw_part(sketch2, curves, vec2point3d, -pi / z, flip_y=True)
        for curve in sketch2.sketchCurves:
            curve.isFixed = True
        # draw the bottom of the tooth
        if lines1[-1].endSketchPoint.geometry.y != lines2[0].startSketchPoint.geometry.y:
            sketch2.sketchCurves.sketchArcs.addByCenterStartEnd(
                fh.point3d(),
                lines2[0].startSketchPoint,
                lines1[-1].endSketchPoint,
                fh.vector3d(z=1),
            ).isFixed = True
        # extend the tooth tip
        l1 = sketch2.sketchCurves.sketchLines.addByTwoPoints(
            lines1[0].startSketchPoint,
            vec2point3d(curves[0][0][0] + vec(m, 0), pi / z),
        )
        l1.isFixed = True
        l2 = sketch2.sketchCurves.sketchLines.addByTwoPoints(
            lines2[-1].endSketchPoint,
            vec2point3d(curves[0][0][0] + vec(m, 0), -pi / z, flip_y=True),
        )
        l2.isFixed = True
        sketch2.sketchCurves.sketchArcs.addByCenterStartEnd(
            fh.point3d(), l2.endSketchPoint, l1.endSketchPoint, fh.vector3d(z=1)
        ).isFixed = True

        # create a patch for the tooth shape
        patch = fh.comp_patch(gear, [sketch2.profiles[0]], fh.FeatureOperations.new_body)

        # move the patch to the correct position
        p = p3d(polar(z * m / 2, -pi / 2 + delta))
        fh.comp_move_free(gear, list(patch.bodies), [fh.matrix_translate(p.x, p.y)])

        # determine the sweep range
        scale_start = 1.01
        scale_end = (r - b) * cos(asin(pi * m / 2 / (r - b))) / r * 0.99
        scale_n = 1 if beta == 0 else max(5, ceil(log(scale_start / scale_end) / 10))

        # create the tooth patches for sweeping
        patches: list[adsk.fusion.BRepBody] = []
        for i in range(scale_n + 1):
            scale = scale_start * (scale_end / scale_start) ** (i / scale_n)
            copy = fh.comp_copy(gear, patch.bodies[0])
            fh.comp_scale(gear, list(copy.bodies), gear.originConstructionPoint, scale)
            if (scale - 1) * tan(beta) != 0:
                matrix = fh.matrix_rotate(
                    (scale - 1) * tan(beta) / sin(delta),
                    fh.vector3d(center.geometry.endPoint),
                    fh.point3d(),
                )
                fh.comp_move_free(gear, list(copy.bodies), [matrix])

            patches.append(copy.bodies[0])

        tooth = fh.comp_loft(gear, fh.FeatureOperations.cut, [p.faces[0] for p in patches], [base])

        fh.comp_remove(gear, [patch.bodies[0]] + patches)

        fh.comp_circular_pattern(gear, [tooth], center, z)

        matrix = fh.matrix_rotate(pi / z / 2, fh.vector3d(x=1), fh.point3d())
        fh.comp_move_free(gear, [base], [matrix])

        sketch3 = gear.sketches.add(gear.yZConstructionPlane)
        sketch3.isComputeDeferred = True
        axis = sketch3.sketchCurves.sketchLines.addByTwoPoints(
            fh.point3d(z=lines[4].endSketchPoint.geometry.x),
            fh.point3d(z=lines[4].startSketchPoint.geometry.x),
        )
        axis.isConstruction = True
        axis.isFixed = True
        circle = sketch3.sketchCurves.sketchCircles.addByCenterRadius(
            fh.point3d(z=plane.geometry.origin.x), plane.geometry.origin.y
        )
        circle.isConstruction = True
        circle.isFixed = True

        sketch.isComputeDeferred = False
        sketch2.isComputeDeferred = False
        sketch3.isComputeDeferred = False
        return axis

    # generate the first gear
    gear1_occurrence = comp.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    gear1 = gear1_occurrence.component
    gear1.name = (
        f"bevel{format(round(m*10,2),"g")}M{z1}T{format(round((delta1+delta2)/pi*180,2), "g")}deg"
    )
    build_gear(gear1, deltaa1, delta1, deltaf1, z1, beta)

    # refresh the viewport
    adsk.doEvents()
    adsk.core.Application.get().activeViewport.refresh()

    # generate the second gear
    gear2_occurrence = comp.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    gear2 = gear2_occurrence.component
    gear2.name = (
        f"bevel{format(round(m*10,2),"g")}M{z2}T{format(round((delta1+delta2)/pi*180,2), "g")}deg"
    )
    build_gear(gear2, deltaa2, delta2, deltaf2, z2, -beta)

    # move to the meshing position
    for occ in comp.occurrences:
        occ.isGroundToParent = False

    design.rootComponent.transformOccurrences(
        [gear2_occurrence.createForAssemblyContext(comp_occurrence)],
        [
            fh.matrix_rotate(
                delta1 + delta2, fh.vector3d(z=1), base=fh.matrix_rotate(pi, fh.vector3d(x=1))
            )
        ],
        False,
    )
    design.snapshots.add()  # capture the position

    # draw the axis of the first gear for joint creation
    sketch1 = comp.sketches.add(comp.yZConstructionPlane)
    sketch1.sketchCurves.sketchCircles.addByCenterRadius(
        fh.point3d(z=gear1.sketches[2].sketchCurves.sketchCircles[0].centerSketchPoint.geometry.z),
        gear1.sketches[2].sketchCurves.sketchCircles[0].radius,
    )
    sketch1.isVisible = False

    # draw the axis of the second gear onto the tilted plane for joint creation
    inp = comp.constructionPlanes.createInput()
    inp.setByAngle(
        comp.zConstructionAxis,
        fh.value_input(delta1 + delta2),
        comp.yZConstructionPlane,
    )
    plane2 = comp.constructionPlanes.add(inp)

    sketch2 = comp.sketches.add(plane2)
    sketch2.sketchCurves.sketchCircles.addByCenterRadius(
        fh.point3d(z=gear2.sketches[2].sketchCurves.sketchCircles[0].centerSketchPoint.geometry.z),
        gear2.sketches[2].sketchCurves.sketchCircles[0].radius,
    )
    sketch2.isVisible = False

    # create the joints

    fh.comp_built_joint_revolute(
        comp,
        comp_occurrence.childOccurrences[1],
        comp_occurrence,
        sketch1.profiles[0],
        cast(adsk.fusion.JointDirections, adsk.fusion.JointDirections.ZAxisJointDirection),
    )

    fh.comp_built_joint_revolute(
        comp,
        comp_occurrence.childOccurrences[0],
        comp_occurrence,
        sketch2.profiles[0],
        cast(adsk.fusion.JointDirections, adsk.fusion.JointDirections.ZAxisJointDirection),
    )

    # creating motion link is not supported in API
    # https://forums.autodesk.com/t5/fusion-api-and-scripts/adding-motion-link-to-the-fusion-360-api/td-p/11728121
