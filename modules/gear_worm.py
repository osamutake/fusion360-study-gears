from math import asin, cos, pi, ceil
from typing import cast

import adsk.core, adsk.fusion

from .gear_curve import rack_geometry
from .lib import fusion_helper as fh
from .lib.fusion_helper import vec, Vector
from .gear_rack import RackParams

# ウォームはラックの歯をらせん状に回転させて作る
def gear_worm(param: RackParams, tip_fillet: float, direction: int):
    sin_beta = param.worm * param.m / param.thickness
    angle = asin(sin_beta)

    m = param.m
    mf = param.mf
    mk = param.mk
    thickness = param.thickness
    length = param.length

    # ラック形状を求め、対称性のいい位置に移動する
    r1, r2, fillet_center, fillet_radius = rack_geometry(
        param.m, param.alpha, param.fillet, mf, mk, param.rc, param.backlash
    )
    [r1, r2, fillet_center] = [vec(0, pi * m / 4) + p for p in [r1, r2, fillet_center]]

    app = adsk.core.Application.get()
    design = adsk.fusion.Design.cast(app.activeProduct)

    def p3d(v: Vector):
        return adsk.core.Point3D.create(v.x, v.y, 0)

    # 親コンポーネントを作成
    comp = design.activeComponent.occurrences.addNewComponent(
        adsk.core.Matrix3D.create()
    ).component

    # 歯車コンポーネントを作成
    gear_occurrence = comp.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    gear_occurrence.isGroundToParent = False
    gear = gear_occurrence.component
    gear.name = f"worm{format(param.m*10, '.2g')}M{format(round(param.length*10,2), 'g')}mm"
    comp.name = gear.name[0:1].capitalize() + gear.name[1:]

    # 歯先円を描画
    sketch = gear.sketches.add(gear.xYConstructionPlane)
    bar_circle = sketch.sketchCurves.sketchCircles.addByCenterRadius(
        fh.point3d(), thickness / 2 + m * mk
    )

    # 歯先円を押し出す
    rod = fh.comp_extrude(gear, sketch.profiles, fh.FeatureOperations.new_body, length).bodies[0]

    # 歯を描画
    teeth: list[adsk.fusion.SketchLine | adsk.fusion.SketchArc] = []
    offset = vec(thickness / 2, pi * m / 2)
    sketch2 = gear.sketches.add(gear.xZConstructionPlane)
    draw_line = sketch2.sketchCurves.sketchLines.addByTwoPoints
    draw_arc = sketch2.sketchCurves.sketchArcs.addByCenterStartEnd
    teeth.append(draw_line(p3d(r1 + offset), p3d(r2 + offset)))
    if fillet_center.y == 0:
        if fillet_radius > 0:
            teeth.append(
                draw_arc(
                    p3d(fillet_center + offset),
                    teeth[-1].endSketchPoint,
                    p3d(r2.flip_y() + offset),
                )
            )
    else:
        if fillet_radius > 0:
            teeth.append(
                draw_arc(
                    p3d(fillet_center + offset),
                    teeth[-1].endSketchPoint,
                    p3d(vec(-m * mf, fillet_center.y) + offset),
                )
            )
        teeth.append(
            draw_line(teeth[-1].endSketchPoint, p3d(vec(-m * mf, -fillet_center.y) + offset))
        )
        if fillet_radius > 0:
            teeth.append(
                draw_arc(
                    p3d(fillet_center.flip_y() + offset),
                    teeth[-1].endSketchPoint,
                    p3d(r2.flip_y() + offset),
                )
            )
    teeth.append(draw_line(teeth[-1].endSketchPoint, p3d(r1.flip_y() + offset)))
    for teeth_line in teeth:
        teeth_line.isFixed = True

    # 基準円を描画
    sketch3 = gear.sketches.add(gear.xYConstructionPlane)
    circle = sketch3.sketchCurves.sketchCircles.addByCenterRadius(
        adsk.core.Point3D.create(0, 0, 0), param.thickness / 2
    )
    sketch3.sketchDimensions.addDiameterDimension(
        circle, adsk.core.Point3D.create(param.thickness / 2, param.thickness / 2, 0)
    ).isDriving = True
    circle.isFixed = True
    circle.isConstruction = True

    # 中心線を描画
    axis_line = sketch3.sketchCurves.sketchLines.addByTwoPoints(
        adsk.core.Point3D.create(0, 0, 0),
        adsk.core.Point3D.create(0, 0, length),
    )
    axis_line.isFixed = True
    axis_line.isConstruction = True

    # 中心線を描画
    axis_line2 = sketch.sketchCurves.sketchLines.addByTwoPoints(
        adsk.core.Point3D.create(0, 0, 0),
        adsk.core.Point3D.create(0, 0, +pi * m * param.worm / cos(angle) + length),
    )
    axis_line2.isFixed = True
    axis_line2.isConstruction = True

    # 先端フィレットの処理
    if tip_fillet > 0:
        center = draw_line(p3d(vec(0, pi * m)), p3d(vec(thickness / 2 + m * mk, pi * m)))
        center.isConstruction = True
        center.isFixed = True

        tip_line = draw_line(
            p3d(vec(thickness / 2 + mk * m, 0)),
            p3d(vec(thickness / 2 + mk * m, pi * m)),
        )
        tip_line.isConstruction = True
        tip_line.isFixed = True

        fillet_line = draw_line(
            p3d(vec(thickness / 2 + (mk + tip_fillet) * m, 0)),
            p3d(vec(thickness / 2 + (mk + tip_fillet) * m, pi * m)),
        )
        fillet_line.isConstruction = True
        sketch2.geometricConstraints.addVertical(fillet_line)

        teeth[0].startSketchPoint.isFixed = True
        fillet = draw_arc(
            p3d(vec(thickness / 2, pi * m)),
            teeth[0].startSketchPoint,
            p3d(vec(thickness / 2 + (mk + tip_fillet) * m, pi * m)),
        )
        sketch2.geometricConstraints.addTangent(fillet, teeth[0])
        sketch2.geometricConstraints.addTangent(fillet, fillet_line)
        sketch2.geometricConstraints.addCoincident(fillet.endSketchPoint, fillet_line)
        const2 = sketch2.geometricConstraints.addCoincident(fillet.centerSketchPoint, center)
        if (
            fillet_line.endSketchPoint.geometry.x
            > tip_line.endSketchPoint.geometry.x + tip_fillet * m
        ):
            const2.deleteMe()
            dim = sketch2.sketchDimensions.addDistanceDimension(
                tip_line.startSketchPoint,
                fillet_line.startSketchPoint,
                cast(
                    adsk.fusion.DimensionOrientations,
                    adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation,
                ),
                fillet_line.startSketchPoint.geometry,
            )
            dim.value = tip_fillet * m

        center2 = draw_line(p3d(vec(0, pi * m / 2)), p3d(vec(thickness / 2 + m * mk, pi * m / 2)))
        center2.isConstruction = True
        center2.isFixed = True

        fillet2 = draw_arc(
            fillet.centerSketchPoint.geometry,
            fillet.startSketchPoint.geometry,
            fillet.endSketchPoint.geometry,
        )
        sketch2.geometricConstraints.addSymmetry(
            fillet2.centerSketchPoint, fillet.centerSketchPoint, center2
        )
        sketch2.geometricConstraints.addSymmetry(
            fillet2.endSketchPoint, fillet.startSketchPoint, center2
        )
        sketch2.geometricConstraints.addSymmetry(
            fillet2.startSketchPoint, fillet.endSketchPoint, center2
        )

        # 歯先円からはみ出して閉じる
        p = fillet.endSketchPoint.geometry
        p1 = draw_line(p, p3d(vec(p.x + m, p.y - tip_fillet / 1000)))
        p1.endSketchPoint.isFixed = True
        sketch2.geometricConstraints.addHorizontal(p1)
        sketch2.geometricConstraints.addCoincident(p1.startSketchPoint, fillet)

        p = fillet2.startSketchPoint.geometry
        p2 = draw_line(p, p3d(vec(p.x + m, p.y + tip_fillet / 1000)))
        p2.endSketchPoint.isFixed = True
        sketch2.geometricConstraints.addHorizontal(p2)
        sketch2.geometricConstraints.addCoincident(p2.startSketchPoint, fillet2)

        draw_line(p1.endSketchPoint, p2.endSketchPoint)

        # 歯先円柱をフィレット先端まで広げる
        bar_circle.radius = p2.startSketchPoint.geometry.x
        bar_circle.isFixed = True

        # これだとフィレット先端に本来必要ない継ぎ目ができてしまう
        # のだけれど、それを避けようとするとプロファイルに重なりが
        # 生じてスイープできなくなったり、プロファイル同士の継ぎ目が
        # きれいにつながらずグリッチが生じたりするのでこのままにする

    else:
        # 歯先円からはみ出して閉じる
        teeth.append(draw_line(teeth[-1].endSketchPoint, p3d(r1.flip_y() + offset + vec(m, 0))))
        teeth.append(draw_line(teeth[-1].endSketchPoint, p3d(r1 + offset + vec(m, 0))))
        teeth.append(draw_line(teeth[-1].endSketchPoint, teeth[0].startSketchPoint))

    fh.app_refresh()

    # すべてのカーブを固定
    for curve in sketch2.sketchCurves:
        curve.isFixed = True

    # # らせんを２つに分割する平面
    # inp = gear.constructionPlanes.createInput()
    # inp.setByAngle(
    #     gear.xConstructionAxis, fh.value_input(angle), gear.xZConstructionPlane
    # )
    # plane = gear.constructionPlanes.add(inp)

    # def cut_spiral(profile: adsk.fusion.Profile):
    #     # ラック形状のパッチを作成
    #     patches = fh.comp_patch(gear, profile, fh.FeatureOperations.new_body)

    #     # 1/cos(angle) 分だけ z 方向へ延ばす
    #     patches = fh.comp_scale(
    #         gear,
    #         patches.bodies,
    #         gear.originConstructionPoint,
    #         (1, 1, (1 - 1e-3) / cos(angle)),
    #     )

    #     # 円周方向にコピー
    #     n = 3
    #     patches = list(
    #         fh.comp_circular_pattern(
    #             gear, patches.bodies, gear.zConstructionAxis, n, pi / 4
    #         ).bodies
    #     )
    #     patches = [patches[-1]] + patches[0:-1]

    #     # z 方向にずらす
    #     for i in range(1, n):
    #         matrix = fh.matrix_translate(
    #             0, 0, pi * param.worm * m / cos(angle) / 4 * i / (n - 1) / 2
    #         )
    #         fh.comp_move_free(gear, [patches[i]], matrix)

    #     # らせん状に並べたパッチをロフトで繋ぐ
    #     spiral = fh.comp_loft(
    #         gear, fh.FeatureOperations.new_body, [p.faces[0] for p in patches]
    #     )

    #     # 使い終わったパッチを削除
    #     fh.comp_remove(gear, patches)

    #     # 画面を更新
    #     fh.app_refresh()

    #     spiral = fh.comp_split_body(gear, spiral.bodies[0], plane, True)
    #     bottom, top = sorted(
    #         spiral.bodies, key=lambda b: b.boundingBox.minPoint.z
    #     )  # 下部の
    #     fh.comp_remove(gear, top)
    #     section = [
    #         f
    #         for f in bottom.faces
    #         if Vector(f.geometry.evaluator.getNormalAtPoint(f.pointOnFace)[1])
    #         .cross(Vector(plane.geometry.normal))
    #         .norm()
    #         < EPS
    #     ][0]

    #     # 回転しながら押し出す
    #     spiral = fh.comp_sweep(
    #         gear,
    #         section,
    #         axis_line2,
    #         fh.FeatureOperations.cut,
    #         2 * pi * (1 + length / (param.worm * pi * m / cos(angle))),
    #         [rod],
    #         orientation=adsk.fusion.SweepOrientationTypes.PerpendicularOrientationType,
    #     )

    #     if param.worm > 1:
    #         fh.comp_circular_pattern(gear, spiral, gear.zConstructionAxis, param.worm)

    #     # 画面を更新
    #     fh.app_refresh()

    # # とても無駄だが２回に分けて切らないと「自己交差しています」のエラーが出る
    # for profile in sorted(sketch2.profiles, key=lambda b: b.boundingBox.minPoint.y):
    #     cut_spiral(profile)

    # plane.isLightBulbOn = False
    # sketch3.isVisible = True

    # # 断面を取得するために作ったらせんを削除
    # fh.comp_remove(gear, [b for b in gear.bRepBodies if b != rod])

    patch = fh.comp_patch(gear, sketch2.profiles, fh.FeatureOperations.new_body)

    # 1/cos(angle) 分だけ z 方向へ延ばす
    patch = fh.comp_scale(
        gear,
        patch.bodies,
        gear.originConstructionPoint,
        (1, 1, (1 - 1e-3) / cos(angle)),
    )

    # 半周分だけ円周方向にコピー
    n = 9
    rotation = pi
    patch = fh.comp_circular_pattern(
        gear, patch.bodies, gear.zConstructionAxis, n, direction * rotation
    )
    patches = [list(patch.bodies)[-1]] + list(patch.bodies)[0:-1]

    # z 方向にずらす
    for i in range(1, n):
        matrix = fh.matrix_translate(
            0, 0, rotation * param.worm * m / cos(angle) * i / (n - 1) / 2
        )
        fh.comp_move_free(gear, [patches[i]], matrix)

    # らせん状に並べたパッチをロフトで繋ぐ
    spiral = fh.comp_loft(gear, fh.FeatureOperations.new_body, [p.faces[0] for p in patches])

    # 使い終わったパッチを削除
    fh.comp_remove(gear, patches)

    # もう半周を作成
    spiral2 = fh.comp_copy(gear, spiral.bodies)
    fh.comp_move_free(
        gear,
        spiral2.bodies[0],
        fh.matrix_rotate(pi, translation=fh.vector3d(z=(pi * param.worm * m / cos(angle)) / 2)),
    )

    # 画面を更新
    fh.app_refresh()

    spiral = fh.comp_rectangular_pattern(
        gear,
        [spiral.bodies[0], spiral2.bodies[0]],
        gear.zConstructionAxis,
        ceil(length / (pi * param.worm * m / cos(angle))) + 1,
        pi * param.worm * m / cos(angle),
        True,
    )

    if param.worm > 1:
        spiral = fh.comp_circular_pattern(
            gear,
            spiral.bodies,
            gear.zConstructionAxis,
            param.worm,
        )

    fh.comp_combine(gear, rod, spiral.bodies, fh.FeatureOperations.cut)

    # ジョイントを作成
    fh.comp_joint_revolute(
        comp,
        gear.originConstructionPoint,
        comp.originConstructionPoint,
        adsk.fusion.JointDirections.ZAxisJointDirection,
    )
