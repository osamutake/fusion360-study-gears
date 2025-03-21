from dataclasses import dataclass
from math import cos, floor, pi, tan
from typing import cast

import adsk.core, adsk.fusion

from .gear_curve import rack_geometry
from .lib import fusion_helper as fh
from .lib.fusion_helper import vec, Vector


@dataclass
class RackParams:
    m: float
    height: float
    thickness: float
    length: float
    angle: float
    worm: int
    mk: float
    mf: float
    rc: float
    alpha: float
    backlash: float
    fillet: float


def gear_rack(param: RackParams, tip_fillet: float):
    m = param.m
    mf = param.mf
    height = param.height
    thickness = param.thickness
    length = param.length
    angle = param.angle

    # ラック形状を求める
    r1, r2, fillet_center, fillet_radius = rack_geometry(
        param.m, param.alpha, param.fillet, mf, param.mk, param.rc, param.backlash
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
    gear.name = f"rack{format(param.m*10, '.2g')}M{format(round(param.length*10,2), 'g')}mm"
    comp.name = gear.name[0:1].capitalize() + gear.name[1:]

    # ラックの外形を描画
    outer: list[adsk.fusion.SketchLine] = []
    offset = vec(0, 0) if angle == 0 else vec(0, thickness / 2 * tan(abs(angle)))
    sketch = gear.sketches.add(gear.xYConstructionPlane)
    draw_line = sketch.sketchCurves.sketchLines.addByTwoPoints
    draw_arc = sketch.sketchCurves.sketchArcs.addByCenterStartEnd
    # 右辺、下辺、左辺、上辺の順に描画
    outer.append(draw_line(p3d(vec(r1.x, 0) + offset), p3d(vec(r1.x, -length) + offset)))
    outer.append(draw_line(outer[-1].endSketchPoint, p3d(vec(r1.x - height, -length) + offset)))
    outer.append(draw_line(outer[-1].endSketchPoint, p3d(vec(r1.x - height, 0) + offset)))
    outer.append(draw_line(outer[-1].endSketchPoint, outer[0].startSketchPoint))

    if tip_fillet > 0:
        outer[0].isConstruction = True
        for l in outer:
            l.isFixed = True
        outer.append(
            draw_line(outer[0].startSketchPoint, p3d(vec(r1.x + tip_fillet * m, 0) + offset))
        )
        outer.append(
            draw_line(
                outer[-1].endSketchPoint,
                p3d(vec(r1.x + tip_fillet * m, -length) + offset),
            )
        )
        outer.append(draw_line(outer[-1].endSketchPoint, outer[0].endSketchPoint))
        outer[4].isFixed = True
        outer[6].isFixed = True
        sketch.geometricConstraints.addVertical(outer[5])

    # 歯を描画
    teeth: list[adsk.fusion.SketchLine | adsk.fusion.SketchArc] = []
    offset = -vec(0, pi * m / 2)
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
        teeth.append(
            draw_arc(
                p3d(fillet_center.flip_y() + offset),
                teeth[-1].endSketchPoint,
                p3d(r2.flip_y() + offset),
            )
        )
    teeth.append(draw_line(teeth[-1].endSketchPoint, p3d(r1.flip_y() + offset)))

    if tip_fillet > 0:
        for teeth_line in teeth:
            teeth_line.isFixed = True
        teeth[0].startSketchPoint.isFixed = True

        # 中心線
        center = draw_line(p3d(vec(0, pi * m / 2) + offset), p3d(vec(m, pi * m / 2) + offset))
        center.startSketchPoint.isFixed = True
        sketch.geometricConstraints.addCoincident(center.endSketchPoint, outer[5])
        sketch.geometricConstraints.addHorizontal(center)
        center.isConstruction = True

        fillet = draw_arc(
            p3d(vec(r1.x - m, pi * m / 2) + offset),
            teeth[0].startSketchPoint,
            p3d(vec(r1.x, r1.y + pi * m / 2) + offset),
        )
        sketch.geometricConstraints.addTangent(fillet, teeth[0])
        sketch.geometricConstraints.addTangent(fillet, outer[5])
        cons1 = sketch.geometricConstraints.addCoincident(fillet.centerSketchPoint, center)
        cons2 = sketch.geometricConstraints.addCoincident(fillet.endSketchPoint, outer[0])

        if (
            outer[5].endSketchPoint.geometry.x
            > outer[0].startSketchPoint.geometry.x + tip_fillet * m
        ):
            cons1.deleteMe()
            cons2.deleteMe()
            dim = sketch.sketchDimensions.addDistanceDimension(
                outer[5].startSketchPoint,
                outer[0].startSketchPoint,
                cast(
                    adsk.fusion.DimensionOrientations,
                    adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation,
                ),
                outer[0].startSketchPoint.geometry,
            )
            dim.value = tip_fillet * m
            sketch.geometricConstraints.addCoincident(fillet.endSketchPoint, outer[5])

            fillet2 = draw_arc(
                fillet.centerSketchPoint.geometry,
                fillet.startSketchPoint.geometry,
                fillet.endSketchPoint.geometry,
            )
            sketch.geometricConstraints.addSymmetry(
                fillet2.centerSketchPoint, fillet.centerSketchPoint, center
            )
            sketch.geometricConstraints.addSymmetry(
                fillet2.startSketchPoint, fillet.endSketchPoint, center
            )
            sketch.geometricConstraints.addSymmetry(
                fillet2.endSketchPoint, fillet.startSketchPoint, center
            )

            draw_line(fillet.endSketchPoint, fillet2.startSketchPoint)
        else:
            fillet2 = None
        teeth.append(
            draw_line(
                teeth[-1].endSketchPoint,
                p3d(vec(teeth[-1].endSketchPoint.geometry) + vec(tip_fillet + m, 0)),
            )
        )
        p = fillet.endSketchPoint if fillet2 is None else fillet2.endSketchPoint
        teeth.append(draw_line(p, p3d(vec(r1.x + offset.x + tip_fillet + m, p.geometry.y))))
        teeth.append(draw_line(teeth[-1].endSketchPoint, teeth[-2].endSketchPoint))
    for c in sketch.sketchCurves:
        c.isFixed = True
    for p in sketch.sketchPoints:
        p.isFixed = True

    # 棒を押し出す
    if tip_fillet == 0:
        rack = fh.comp_extrude(
            gear, sketch.profiles, fh.FeatureOperations.new_body, thickness, True, True
        ).bodies[0]
    else:
        profiles = sorted(sketch.profiles, key=lambda p: p.boundingBox.minPoint.x)[0:3]
        rack = fh.comp_extrude(
            gear, profiles, fh.FeatureOperations.new_body, thickness, True, True
        ).bodies[0]

    # 歯を押し出して切り取る
    if angle == 0:
        profiles = sorted(sketch.profiles, key=lambda p: p.boundingBox.minPoint.x)[1:]
        tooth = fh.comp_extrude(
            gear,
            profiles,
            fh.FeatureOperations.cut,
            thickness,
            True,
            True,
            participants=[rack],
        )
    else:
        # はすばラックは歯形のパッチを作成
        profiles = sorted(sketch.profiles, key=lambda p: p.boundingBox.minPoint.x)[1:]
        patch = fh.comp_patch(gear, profiles, fh.FeatureOperations.new_body)
        # y方向へ延ばす
        patch = fh.comp_scale(
            gear,
            [patch.bodies[0]],
            gear.originConstructionPoint,
            (1, 1 / cos(angle), 1),
        )
        # 奥へ押す
        matrix = fh.matrix_translate(thickness / 2 * Vector(0, tan(angle), -1))
        patch = fh.comp_move_free(gear, patch.bodies, matrix)
        # 鏡像を作成
        patch2 = fh.comp_mirror(gear, [patch.bodies[0]], gear.xYConstructionPlane)
        # 角度分だけずらす
        matrix = fh.matrix_translate(0, -thickness * tan(angle), 0)
        patch2 = fh.comp_move_free(gear, [patch2.bodies[0]], matrix)
        # 繋ぐ
        tooth = fh.comp_loft(
            gear, fh.FeatureOperations.cut, [patch.faces[0], patch2.faces[0]], [rack]
        )
        # いらななくなったサーフェスを削除
        fh.comp_remove(gear, [p.bodies[0] for p in [patch, patch2]])

    # 歯を複製する
    quantity = floor((length - thickness * tan(abs(angle))) / (pi * m / cos(angle)))
    fh.comp_rectangular_pattern(
        gear, [tooth], gear.yConstructionAxis, quantity, -pi * m / cos(angle), True
    )

    # ジョイントを作成
    fh.comp_joint_slider(
        comp,
        gear.originConstructionPoint,
        comp.originConstructionPoint,
        adsk.fusion.JointDirections.YAxisJointDirection,
    )

    sketch2 = gear.sketches.add(gear.xYConstructionPlane)
    line = sketch2.sketchCurves.sketchLines.addByTwoPoints(
        adsk.core.Point3D.create(0, 0, 0),
        adsk.core.Point3D.create(0, -length, 0),
    )
    line.isConstruction = True

    line = sketch2.sketchCurves.sketchLines.addByTwoPoints(
        adsk.core.Point3D.create(param.mk * param.m, 0, 0),
        adsk.core.Point3D.create(param.mk * param.m, -param.m * pi / cos(param.angle), 0),
    )
    line.isConstruction = True

    for c in sketch2.sketchCurves:
        c.isFixed = True
    for p in sketch2.sketchPoints:
        p.isFixed = True
