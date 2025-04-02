from collections.abc import Callable
from copy import copy
from math import atan, cos, tan, pi, ceil, trunc
import adsk.core, adsk.fusion

from . import gear_curve
from .lib import fusion_helper as fh
from .lib.fusion_helper import Vector


def gear_cylindrical(
    parent: adsk.fusion.Component,
    params: gear_curve.GearParams,
    thickness: float,
    beta: float,
    tip_fillet: float,
):
    z = params.z
    m = params.m

    # Create a new component by creating a new occurrence.
    comp = parent.occurrences.addNewComponent(adsk.core.Matrix3D.create()).component

    # Create a new sketch on the xy plane.
    sketch = comp.sketches.add(comp.xYConstructionPlane)
    sketch.isComputeDeferred = True

    # はすば補正を施した上で歯形を生成 involute & [fillet]
    # https://www.khkgears.co.jp/gear_technology/basic_guide/KHK365.html
    cos_beta = cos(beta)
    mn = m / cos_beta
    params_adjusted = copy(params)
    params_adjusted.m = mn
    params_adjusted.mk *= cos_beta
    params_adjusted.mf *= cos_beta
    params_adjusted.alpha = atan(tan(params.alpha) / cos_beta)
    params_adjusted.shift *= cos_beta
    params_adjusted.backlash *= -1 if params.inner else 1
    curves = gear_curve.gear_curve(params_adjusted, tip_fillet * cos_beta)
    # curves: [(curve, tangent1, tangent2), ...]

    # 歯先の点 (tip_fillet の分だけ延長済み)
    if isinstance(curves[0][0], Vector):
        pt = curves[0][0] + Vector.polar(curves[0][3], curves[0][1])
    else:
        pt = curves[0][0](curves[0][1])

    # 歯先円 (tip_fillet の分だけ延長済み)
    tip_circle = sketch.sketchCurves.sketchCircles.addByCenterRadius(
        fh.point3d(),
        pt.norm(),
    )
    tip_circle.isFixed = True

    # 歯の切り取り部分を描く
    lines1 = draw_part(sketch, curves, pi / z)
    sketch.geometricConstraints.addCoincident(lines1[0][1], tip_circle)
    lines2 = draw_part(sketch, curves, -pi / z, flip_y=True)
    sketch.geometricConstraints.addCoincident(lines2[-1][2], tip_circle)
    for curve in sketch.sketchCurves:
        curve.isFixed = True

    # 歯底部分を描く
    if lines1[-1][2].geometry.y != lines2[0][1].geometry.y:
        sketch.sketchCurves.sketchArcs.addByCenterStartEnd(
            fh.point3d(),
            lines2[0][1],
            lines1[-1][2],
        ).isFixed = True

    # 歯先を飛び出させる
    l1 = sketch.sketchCurves.sketchLines.addByTwoPoints(
        lines1[0][1],
        fh.point3d(Vector(lines1[0][1].geometry) * 1.01),
    )
    l1.isFixed = True
    l2 = sketch.sketchCurves.sketchLines.addByTwoPoints(
        lines2[-1][2],
        fh.point3d(Vector(lines2[-1][2].geometry) * 1.01),
    )
    l2.isFixed = True
    sketch.sketchCurves.sketchArcs.addByCenterStartEnd(
        fh.point3d(),
        l2.endSketchPoint,
        l1.endSketchPoint,
    ).isFixed = True

    # 円板を押し出す
    profiles = sorted(sketch.profiles, key=lambda p: p.boundingBox.minPoint.x)[
        0 : 2 if tip_fillet == 0 else 3
    ]
    disk = fh.comp_extrude(
        comp, profiles, fh.FeatureOperations.new_body, thickness, True, True
    ).bodies[0]

    # 歯車の回転軸を描画する
    sketch2 = comp.sketches.add(comp.xYConstructionPlane)
    center_axis = sketch2.sketchCurves.sketchLines.addByTwoPoints(
        fh.point3d(z=-thickness / 2), fh.point3d(z=thickness / 2)
    )
    center_axis.isFixed = True

    # 基準円を描画する
    circle = sketch2.sketchCurves.sketchCircles.addByCenterRadius(fh.point3d(), mn * z / 2)
    circle.isConstruction = True
    d = mn * z / 2
    sketch2.sketchDimensions.addDiameterDimension(circle, fh.point3d(d, d), isDriving=True)
    circle.isFixed = True

    # 歯形を生成
    teeth_profiles = sorted(  # 右から２つ取り出す
        sketch.profiles,
        key=lambda p: p.areaProperties().centroid.x,
        reverse=True,
    )[0 : 2]
    if beta == 0:
        # 直歯を押し出して切り取る
        cut = fh.FeatureOperations.cut
        tooth_feature = fh.comp_extrude(
            comp, teeth_profiles, cut, thickness, True, True, participants=[disk]
        )
    else:
        # このやり方だとなぜかずれが生じるので一旦方法を変える
        #
        # # 通常のはすば歯車はひねりながら押し出せばいいのだが、
        # # そのままやると元の図形がある中央位置に継ぎ目ができて
        # # しまうので一旦端にずらしてから押し出す
        # patch1 = fh.comp_patch(comp, teeth_profiles, fh.FeatureOperations.new_body)
        # matrix = fh.matrix_rotate(
        #     -thickness * tan(beta) / (mn * z),
        #     comp.zConstructionAxis.geometry.direction,
        #     comp.originConstructionPoint.geometry,
        #     fh.vector3d(0, 0, -thickness / 2),
        # )
        # patch1 = fh.comp_move_free(comp, patch1.bodies, matrix)

        # tooth_feature = fh.comp_sweep(
        #         comp,
        #         patch1.bodies[0].faces[0],
        #         center_axis,
        #         fh.FeatureOperations.cut,
        #         participants=[disk],
        #         twist=2 * thickness * tan(beta) / (mn * z),
        #     )
        # )

        # fh.comp_remove(comp, patch1.bodies)

        patches: list[adsk.fusion.BRepBody] = []
        n = max(3, ceil(thickness * tan(beta) / (mn * z) / (2 * pi / 18)))
        for i in range(n):
            zz = i / (n - 1) * thickness / 2
            patches.append(
                fh.comp_patch(comp, teeth_profiles, fh.FeatureOperations.new_body).bodies[0]
            )

            if i > 0:
                matrix = fh.matrix_rotate(
                    -2 * zz * tan(beta) / (mn * z),
                    comp.zConstructionAxis.geometry.direction,
                    comp.originConstructionPoint.geometry,
                    fh.vector3d(0, 0, -zz),
                )
                fh.comp_move_free(comp, patches[-1], matrix)

                patches.insert(
                    0,
                    fh.comp_patch(comp, teeth_profiles, fh.FeatureOperations.new_body).bodies[0],
                )
                matrix = fh.matrix_rotate(
                    2 * zz * tan(beta) / (mn * z),
                    comp.zConstructionAxis.geometry.direction,
                    comp.originConstructionPoint.geometry,
                    fh.vector3d(0, 0, zz),
                )
                fh.comp_move_free(comp, patches[0], matrix)

        tooth_feature = fh.comp_loft(
            comp, fh.FeatureOperations.cut, [p.faces[0] for p in patches], disk
        )
        fh.comp_remove(comp, patches)

    # 円周方向に歯を複製する
    fh.comp_circular_pattern(comp, tooth_feature, comp.zConstructionAxis, trunc(z))

    # 歯車の回転軸＆基準円を表示する
    sketch2.isVisible = True


def draw_part(
    sketch: adsk.fusion.Sketch,
    curves: list[tuple[Callable[[float], Vector] | Vector, float, float, int | float]],
    rot: float,
    last: adsk.fusion.SketchPoint | None = None,
    flip_y=False,
):
    def point3d(v: Vector):
        if not flip_y:
            return fh.point3d(v.rotate(rot))
        else:
            return fh.point3d(v.flip_y().rotate(rot))

    result: list[
        tuple[
            adsk.fusion.SketchArc | adsk.fusion.SketchFittedSpline,
            adsk.fusion.SketchPoint,
            adsk.fusion.SketchPoint,
        ]
    ] = []
    for c in reversed(curves) if flip_y else curves:
        if not flip_y:
            s = c[1]
            e = c[2]
        else:
            s = c[2]
            e = c[1]
        if isinstance(c[0], Vector):
            # (center, start, end, radius)
            arc = sketch.sketchCurves.sketchArcs.addByCenterStartSweep(
                point3d(c[0]),
                point3d(c[0] + Vector.polar(c[3], s)) if last is None else last,
                e - s if not flip_y else s - e,
            )
            if (e - s if not flip_y else s - e) > 0:
                result.append((arc, arc.startSketchPoint, arc.endSketchPoint))
            else:
                result.append((arc, arc.endSketchPoint, arc.startSketchPoint))
        else:
            # c = (func, ts, te, n_points)

            def curve(func: Callable[[float], Vector], s: float, e: float, n: int):
                points = [func(s + (e - s) * i / (n * 30 - 1)) for i in range(n * 30)]
                length = [0]
                for i in range(1, len(points)):
                    length.append(length[-1] + (points[i] - points[i - 1]).norm())
                result = [points[0]]
                j = 0
                for i in range(1, n):
                    l = length[-1] * (i / (n - 1))
                    while length[j] < l:
                        j += 1
                    result.append(points[j])
                return result

            points = curve(c[0], s, e, int(c[3]))

            points3d = [point3d(p) for p in points]
            if last is not None:
                points3d[0] = last
            spline = sketch.sketchCurves.sketchFittedSplines.add(fh.collection(points3d))

            # 接線制約を追加
            delta = (e - s) / c[3] / 1000

            pss = points[0] + (c[0](s + delta) - points[0]) * 500
            line = sketch.sketchCurves.sketchLines.addByTwoPoints(
                spline.startSketchPoint, point3d(pss)
            )
            line.isConstruction = True
            line.isFixed = True
            sketch.geometricConstraints.addTangent(spline, line)

            pee = points[-1] + (c[0](e - delta) - points[-1]) * 500
            line = sketch.sketchCurves.sketchLines.addByTwoPoints(
                spline.endSketchPoint, point3d(pee)
            )
            line.isConstruction = True
            line.isFixed = True
            sketch.geometricConstraints.addTangent(spline, line)

            result.append((spline, spline.startSketchPoint, spline.endSketchPoint))

        last = result[-1][2]

    return result
