from collections.abc import Callable
from copy import copy
from math import atan, cos, sin, tan, pi, ceil, trunc
import adsk.core, adsk.fusion

from . import gear_curve
from .lib import fusion_helper as fh
from .lib.fusion_helper import Vector, vec
from .lib.curve import Curve

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

    # delta は回転角
    def point3d(pnt: Vector, delta=0.0, flip_y=False):
        (r, t) = pnt.to_polar()
        if flip_y:
            return fh.point3d(r * cos(t - delta), r * sin(t - delta + pi))
        else:
            return fh.point3d(r * cos(t + delta), r * sin(t + delta))

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
    curves = gear_curve.gear_curve(params_adjusted)
    # curves: [(curve, tangent1, tangent2), ...]

    # 歯先円
    tip_circle = sketch.sketchCurves.sketchCircles.addByCenterRadius(
        fh.point3d(),
        point3d(curves[0][0][0]).distanceTo(fh.point3d()),
    )
    tip_circle.isFixed = True

    # 歯の切り取り部分を描く
    lines1 = draw_part(sketch, curves, point3d, pi / z)
    sketch.geometricConstraints.addCoincident(lines1[0].startSketchPoint, tip_circle)
    lines2 = draw_part(sketch, curves, point3d, -pi / z, flip_y=True)
    sketch.geometricConstraints.addCoincident(lines2[-1].endSketchPoint, tip_circle)
    for curve in sketch.sketchCurves:
        curve.isFixed = True

    if tip_fillet > 0:
        # 歯先の開始点
        p1 = Vector(lines1[0].startSketchPoint.geometry)
        # 歯の中心点＆中心線
        p2 = Vector.polar(p1.norm() + tip_fillet * m, pi / z)
        p3 = p1.rotate(-pi / z).flip_y().rotate(pi / z)
        tip_center_line = sketch.sketchCurves.sketchLines.addByTwoPoints(
            fh.point3d(),
            fh.point3d(p2),
        )
        tip_center_line.isConstruction = True
        tip_center_line.isFixed = True
        # フィレットの先端円
        tip_fillet_circle = sketch.sketchCurves.sketchCircles.addByCenterRadius(
            fh.point3d(),
            p1.norm() + tip_fillet * m,
        )
        tip_circle.isConstruction = True
        sketch.geometricConstraints.addCoincident(
            tip_fillet_circle.centerSketchPoint, sketch.originPoint
        )
        # フィレットの円弧
        tip_fillet_arc = sketch.sketchCurves.sketchArcs.addByThreePoints(
            lines1[0].startSketchPoint,
            fh.point3d(p2),
            fh.point3d(p3),
        )
        # インボリュート曲線に接する
        sketch.geometricConstraints.addTangent(tip_fillet_arc, lines1[0])
        # 中心線上に中心を置き、外周円に接し、歯先円まで
        cons1 = sketch.geometricConstraints.addCoincident(
            tip_fillet_arc.centerSketchPoint, tip_center_line
        )
        cons2 = sketch.geometricConstraints.addTangent(tip_fillet_arc, tip_fillet_circle)
        cons3 = sketch.geometricConstraints.addCoincident(
            tip_fillet_arc.endSketchPoint, tip_circle
        )

        # それだと大きすぎるときは２つに分割する
        if tip_fillet_circle.radius > tip_circle.radius + tip_fillet * m:
            # 条件を削除
            cons1.deleteMe()
            cons2.deleteMe()
            cons3.deleteMe()
            # 外周円の大きさを先に決めて
            tip_fillet_circle.radius = tip_circle.radius + tip_fillet * m
            sketch.sketchDimensions.addRadialDimension(
                tip_fillet_circle, fh.point3d(tip_fillet_circle.radius, tip_fillet_circle.radius)
            )
            # 外周円で接するところで終端
            sketch.geometricConstraints.addTangent(tip_fillet_arc, tip_fillet_circle)
            sketch.geometricConstraints.addCoincident(
                tip_fillet_arc.endSketchPoint, tip_fillet_circle
            )
            # 反対側は別の円弧を用意して対称に配置する
            tip_fillet_arc2 = sketch.sketchCurves.sketchArcs.addByCenterStartEnd(
                tip_fillet_arc.centerSketchPoint.geometry,
                tip_fillet_arc.startSketchPoint.geometry,
                tip_fillet_arc.endSketchPoint.geometry,
            )
            sketch.geometricConstraints.addSymmetry(
                tip_fillet_arc.startSketchPoint, tip_fillet_arc2.endSketchPoint, tip_center_line
            )
            sketch.geometricConstraints.addSymmetry(
                tip_fillet_arc.endSketchPoint, tip_fillet_arc2.startSketchPoint, tip_center_line
            )
            sketch.geometricConstraints.addSymmetry(
                tip_fillet_arc.centerSketchPoint,
                tip_fillet_arc2.centerSketchPoint,
                tip_center_line,
            )
        else:
            tip_fillet_arc2 = None

    # 歯底部分を描く
    if lines1[-1].endSketchPoint.geometry.y != lines2[0].startSketchPoint.geometry.y:
        sketch.sketchCurves.sketchArcs.addByCenterStartEnd(
            fh.point3d(),
            lines2[0].startSketchPoint,
            lines1[-1].endSketchPoint,
        ).isFixed = True
    # 歯先を飛び出させる
    if tip_fillet == 0:
        l1 = sketch.sketchCurves.sketchLines.addByTwoPoints(
            lines1[0].startSketchPoint,
            point3d(curves[0][0][0] + vec(m, 0), pi / z),
        )
    else:
        l1 = sketch.sketchCurves.sketchLines.addByTwoPoints(
            (
                tip_fillet_arc.endSketchPoint
                if not tip_fillet_arc2
                else tip_fillet_arc2.endSketchPoint
            ),
            point3d(p3.normalize(p1.norm() + m)),
        )
    l1.isFixed = True
    l2 = sketch.sketchCurves.sketchLines.addByTwoPoints(
        lines2[-1].endSketchPoint,
        point3d(curves[0][0][0] + vec(m, 0), -pi / z, flip_y=True),
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
    )[0 : 2 if tip_fillet == 0 else 3]
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
    curves: list[tuple[Curve, Vector, Vector]],
    point3d: Callable[[Vector, float, bool], adsk.core.Point3D],
    delta=0,
    last=None,
    flip_y=False,
):
    result: list[adsk.fusion.SketchFittedSpline] = []
    for c in reversed(curves) if flip_y else curves:
        # c = (curve, tangent1, tangent2)
        c0 = list(reversed(c[0]) if flip_y else c[0])
        if last is None:
            last = point3d(c0[0], delta, flip_y)
        spline = sketch.sketchCurves.sketchFittedSplines.add(
            fh.collection([last] + [point3d(pp, delta, flip_y) for pp in c0[1:]])
        )
        result.append(spline)
        last = spline.endSketchPoint

        # flip されていたら接線ベクトルを入れ替える
        if not flip_y:
            c1, c2 = c[1:3]
        else:
            c2, c1 = c[1:3]

        # 接線制約を追加
        line = sketch.sketchCurves.sketchLines.addByTwoPoints(
            spline.startSketchPoint, point3d(c0[0] + c1, delta, flip_y)
        )
        line.isConstruction = True
        line.isFixed = True
        sketch.geometricConstraints.addTangent(spline, line)
        line = sketch.sketchCurves.sketchLines.addByTwoPoints(
            spline.endSketchPoint, point3d(c0[-1] + c2, delta, flip_y)
        )
        line.isConstruction = True
        line.isFixed = True
        sketch.geometricConstraints.addTangent(spline, line)
    return result
