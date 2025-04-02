from math import pi, ceil, asin, sin, cos

import adsk.core
import adsk.fusion

from . import gear_curve
from .lib import fusion_helper as fh
from .lib.fusion_helper import Vector


def gear_crown(
    comp: adsk.fusion.Component,
    params: gear_curve.GearParams,
    z: int,
    w1: float,
    w2: float,
):

    curves = gear_curve.gear_curve(params, params.rc)

    # convert curves to a list of Vector
    def build_curve():
        curve: list[Vector] = []
        for c in curves:
            if isinstance(c[0], Vector):
                n = ceil(abs(c[2] - c[1]) / pi * 36 * 2)
                for i in range(0, n + 1):
                    curve.append(c[0] + Vector.polar(c[3], c[1] + (c[2] - c[1]) * i / n))
            else:
                n = c[3] * 10
                for i in range(0, n + 1):
                    curve.append(c[0](c[1] + (c[2] - c[1]) * i / n))
        return curve

    curve = build_curve()

    rp = params.m * params.z / 2  # pinion
    rp2 = params.m * z / 2  # crown gear
    mk = params.mk * params.m
    mf = params.mf * params.m
    rt = rp + mf + params.shift * params.m  # extended tip circle radius

    # turn pinion and calc tooth profile
    # t: radial offset from the crown gear's reference circle
    def calc_tooth(t: float):
        # crown gear tooth profile is calculated with dx as interval
        min_y = [0 for _ in range(101)]
        dx = (mk + mf) / (len(min_y) - 1)

        # remove pinion profile from the crown gear at pinion angle theta
        def one_step(theta: float):

            # turn pinion by theta
            def trans(v: Vector, theta: float, t: float):
                v = v.rotate(-theta)
                phi = (theta * params.z) / z
                return Vector(v.x, phi * (rp2 + t) + (rp2 + t) * asin(v.y / (rp2 + t)))

            translated = [trans(v, theta, t) for v in curve]
            k = 0
            x = rt
            # @pylint: disable=consider-using-enumerate
            for j in range(len(translated)):
                xj = translated[j].x
                yj = translated[j].y
                while k < len(min_y) and xj < x:
                    if j > 0:
                        xj1 = translated[j - 1].x
                        yj1 = translated[j - 1].y
                        y = yj1 + ((x - xj1) * (yj - yj1)) / (xj - xj1)
                        if y < min_y[k]:
                            min_y[k] = y
                    x -= dx
                    k += 1
            return translated[0].x

        # turn pinion until the pinion tooth tip get out of the crown gear
        dt = pi * 2 / z**0.5 / 400
        theta = 0
        while one_step(theta) > rt - mf - mk:
            theta -= dt  # negative direction
        theta = dt
        while one_step(theta) > rt - mf - mk:
            theta += dt  # positive direction

        tooth = [Vector(rt - dx * i, y) for i, y in enumerate(min_y)]

        # # まずは均等に点を追加
        # reduced = [tooth[0]]
        # nn = 100
        # for i in range(1, nn):
        #     reduced.append(tooth[floor((len(tooth) * i) / nn)])
        # reduced.append(tooth[-1])

        # # 近似曲線から最も離れているところに点を増やす
        # # 点の密度が不均一だとロフトで誤差が発生するのでやめた
        # def add_point():
        #     err_m = 0
        #     err_i = 0
        #     reduced.sort(key=lambda v1: v1.x)
        #     c = interpolate([v.x for v in reduced], [v.y for v in reduced])
        #     for i in range(1, len(tooth) - 1):
        #         x = rt - dx * i
        #         slope = (tooth[i + 1].y - tooth[i - 1].y) / (2 * dx)
        #         err = (tooth[i].y - c(x)) * cos(atan(slope))
        #         if abs(err) > err_m:
        #             err_m = abs(err)
        #             err_i = i

        #     reduced.append(tooth[err_i])
        #     reduced.sort(key=lambda v1: v1.x)
        #     return err_m

        # while len(reduced) < 55:
        #     if add_point() < params.m / 1000:
        #         pass # break
        #
        # reduced.sort(key=lambda v1: v1.x)
        # return reduced

        tooth.sort(key=lambda v: v.x)
        return tooth

    gear_occurrence = comp.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    gear = gear_occurrence.component

    sketch = gear.sketches.add(gear.xYConstructionPlane)
    sketch.isComputeDeferred = True

    # generate a patch of tooth profile
    # t: radial offset from the crown gear's reference circle
    def generate_patch(t: float):
        tooth = calc_tooth(t)
        tooth = [
            Vector((rp2 + t) * cos(v.y / (rp2 + t)), (rp2 + t) * sin(v.y / (rp2 + t)), -v.x)
            for v in tooth  # the profile is on the cylinder surface
        ]
        c1 = sketch.sketchCurves.sketchFittedSplines.add(
            fh.collection([fh.point3d(v) for v in tooth])
        )

        mirrored = [fh.point3d(v.flip_y()) for v in reversed(tooth)]
        mirrored[0] = c1.endSketchPoint
        c2 = sketch.sketchCurves.sketchFittedSplines.add(fh.collection(mirrored))

        # tangent at the bottom of the tooth
        l0 = sketch.sketchCurves.sketchLines.addByTwoPoints(
            c1.endSketchPoint,
            fh.point3d(Vector(c1.endSketchPoint.geometry) + Vector(0, 0.1, 0)),
        )
        l0.isConstruction = True
        l0.isFixed = True
        sketch.geometricConstraints.addTangent(c1, l0)
        sketch.geometricConstraints.addTangent(c2, l0)

        # extend the profile upward to make clear intersection with the top face
        l1 = sketch.sketchCurves.sketchLines.addByTwoPoints(
            c1.startSketchPoint,
            fh.point3d(Vector(c1.startSketchPoint.geometry) + Vector(0, 0, params.m / 10)),
        )
        l2 = sketch.sketchCurves.sketchLines.addByTwoPoints(
            c2.endSketchPoint,
            fh.point3d(Vector(c2.endSketchPoint.geometry) + Vector(0, 0, params.m / 10)),
        )
        arc = sketch.sketchCurves.sketchArcs.addByCenterStartEnd(
            fh.point3d(0, 0, l1.endSketchPoint.geometry.z),
            l1.endSketchPoint,
            l2.endSketchPoint,
        )

        # create a patch from the sketch curves making a closed loop
        return fh.comp_patch(gear, arc, fh.FeatureOperations.new_body)

    patches: list[adsk.fusion.PatchFeature] = []

    # create patches of tooth profiles
    t = -w2 * 1.01
    dt = 0.1
    i = 0
    while True:
        if (-1.6 < t < -0.5) or i % 4 == 0:
            patches.append(generate_patch(t * params.m))
            fh.app_refresh()
            if t > w1 * 1.01:
                break
        t += dt
        i += 1

    # extrude the donut between inner and outer circles
    sketch.sketchCurves.sketchCircles.addByCenterRadius(
        fh.point3d(0, 0, -(rp + params.shift * params.m)), rp2 - w1 * params.m
    )
    sketch.sketchCurves.sketchCircles.addByCenterRadius(
        fh.point3d(0, 0, -(rp + params.shift * params.m)), rp2 + w2 * params.m
    )
    body = fh.comp_extrude(
        gear,
        sketch.profiles[1],
        fh.FeatureOperations.new_body,
        (params.m * params.mk, params.m * (params.mf + 1)),
    ).bodies[0]
    sketch.isComputeDeferred = False

    # cut the tooth groove from the donut
    loft = fh.comp_loft(gear, fh.FeatureOperations.cut, [p.faces[0] for p in patches], body)
    fh.app_refresh()

    # remove the used patches
    fh.comp_remove(gear, [p.bodies[0] for p in patches])

    # create a circular pattern of the tooth groove
    fh.comp_circular_pattern(gear, loft, gear.zConstructionAxis, z)

    # draw reference circle and axis of the crown gear
    sketch2 = gear.sketches.add(gear.xYConstructionPlane)
    sketch2.isComputeDeferred = True
    ref = sketch2.sketchCurves.sketchCircles.addByCenterRadius(
        fh.point3d(0, 0, -(rp + params.shift * params.m)), rp2
    )
    ref.isConstruction = True
    ref.isFixed = True
    axis = sketch2.sketchCurves.sketchLines.addByTwoPoints(
        fh.point3d(0, 0, -(rp + (params.shift - params.mk) * params.m)),
        fh.point3d(0, 0, -(rp + (params.shift + params.mf) * params.m)),
    )
    axis.isConstruction = True
    axis.isFixed = True
    sketch2.isComputeDeferred = False
