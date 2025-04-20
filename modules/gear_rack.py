from dataclasses import dataclass
from math import cos, ceil, pi, tan
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

    # calculate the rack geometry
    r1, r2, fillet_center, fillet_radius = rack_geometry(
        param.m, param.alpha, param.fillet, mf, param.mk, param.rc, param.backlash
    )
    # translate points by 1/4 pitch
    [r1, r2, fillet_center] = [vec(0, pi * m / 4) + p for p in [r1, r2, fillet_center]]

    app = adsk.core.Application.get()
    design = adsk.fusion.Design.cast(app.activeProduct)

    def p3d(v: Vector):
        return adsk.core.Point3D.create(v.x, v.y, 0)

    # wrapper component
    wrapper = design.activeComponent.occurrences.addNewComponent(
        adsk.core.Matrix3D.create()
    ).component

    # rack component
    rack_occurrence = wrapper.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    rack_occurrence.isGroundToParent = False
    rack = rack_occurrence.component
    rack.name = f"rack{format(param.m*10, '.2g')}M{format(round(param.length*10,2), 'g')}mm"
    wrapper.name = rack.name[0:1].capitalize() + rack.name[1:]

    # outline of the rack
    outline: list[adsk.fusion.SketchLine] = []
    offset = vec(0, 0) if angle == 0 else vec(0, thickness / 2 * tan(abs(angle)))
    sketch = rack.sketches.add(rack.xYConstructionPlane)
    draw_line = sketch.sketchCurves.sketchLines.addByTwoPoints
    draw_arc = sketch.sketchCurves.sketchArcs.addByCenterStartEnd

    # draw right, bottom, left and top edges
    outline.append(draw_line(p3d(vec(r1.x, 0) + offset), p3d(vec(r1.x, -length) + offset)))
    outline.append(
        draw_line(outline[-1].endSketchPoint, p3d(vec(r1.x - height, -length) + offset))
    )
    outline.append(draw_line(outline[-1].endSketchPoint, p3d(vec(r1.x - height, 0) + offset)))
    outline.append(draw_line(outline[-1].endSketchPoint, outline[0].startSketchPoint))

    # tip fillet
    if tip_fillet > 0:
        outline[0].isConstruction = True
        for l in outline:
            l.isFixed = True
        outline.append(
            draw_line(outline[0].startSketchPoint, p3d(vec(r1.x + tip_fillet * m, 0) + offset))
        )
        outline.append(
            draw_line(
                outline[-1].endSketchPoint,
                p3d(vec(r1.x + tip_fillet * m, -length) + offset),
            )
        )
        outline.append(draw_line(outline[-1].endSketchPoint, outline[0].endSketchPoint))
        outline[4].isFixed = True
        outline[6].isFixed = True
        sketch.geometricConstraints.addVertical(outline[5])

    # tooth groove shape
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
        sketch.geometricConstraints.addCoincident(center.endSketchPoint, outline[5])
        sketch.geometricConstraints.addHorizontal(center)
        center.isConstruction = True

        fillet = draw_arc(
            p3d(vec(r1.x - m, pi * m / 2) + offset),
            teeth[0].startSketchPoint,
            p3d(vec(r1.x, r1.y + pi * m / 2) + offset),
        )
        sketch.geometricConstraints.addTangent(fillet, teeth[0])
        sketch.geometricConstraints.addTangent(fillet, outline[5])
        cons1 = sketch.geometricConstraints.addCoincident(fillet.centerSketchPoint, center)
        cons2 = sketch.geometricConstraints.addCoincident(fillet.endSketchPoint, outline[0])

        if (
            outline[5].endSketchPoint.geometry.x
            > outline[0].startSketchPoint.geometry.x + tip_fillet * m
        ):
            cons1.deleteMe()
            cons2.deleteMe()
            dim = sketch.sketchDimensions.addDistanceDimension(
                outline[5].startSketchPoint,
                outline[0].startSketchPoint,
                cast(
                    adsk.fusion.DimensionOrientations,
                    adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation,
                ),
                outline[0].startSketchPoint.geometry,
            )
            dim.value = tip_fillet * m
            sketch.geometricConstraints.addCoincident(fillet.endSketchPoint, outline[5])

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

    fh.sketch_fix_all(sketch)

    # extrude the outline
    if tip_fillet == 0:
        body = fh.comp_extrude(
            rack, sketch.profiles, fh.FeatureOperations.new_body, thickness, True, True
        ).bodies[0]
    else:
        profiles = sorted(sketch.profiles, key=lambda p: p.boundingBox.maxPoint.x)[0:-1]
        body = fh.comp_extrude(
            rack, profiles, fh.FeatureOperations.new_body, thickness, True, True
        ).bodies[0]

    # cut the tooth groove
    if angle == 0:
        # straight rack
        profiles = sorted(sketch.profiles, key=lambda p: p.boundingBox.minPoint.x)[1:]
        tooth = fh.comp_extrude(
            rack,
            profiles,
            fh.FeatureOperations.cut,
            thickness,
            True,
            True,
            participants=[body],
        )
    else:
        # helical rack
        profiles = sorted(sketch.profiles, key=lambda p: p.boundingBox.minPoint.x)[1:]
        patch = fh.comp_patch(rack, profiles, fh.FeatureOperations.new_body)
        # scale in the Y direction to find transverse section
        patch = fh.comp_scale(
            rack,
            [patch.bodies[0]],
            rack.originConstructionPoint,
            (1, 1 / cos(angle), 1),
        )
        # push the patch to the right position
        matrix = fh.matrix_translate(thickness / 2 * Vector(0, tan(angle), -1))
        patch = fh.comp_move_free(rack, patch.bodies, matrix)
        # generate the mirror image of the patch
        patch2 = fh.comp_mirror(rack, [patch.bodies[0]], rack.xYConstructionPlane)
        # shift the patch2 to the right position
        matrix = fh.matrix_translate(0, -thickness * tan(angle), 0)
        patch2 = fh.comp_move_free(rack, [patch2.bodies[0]], matrix)
        # connect the two patches
        tooth = fh.comp_loft(
            rack, fh.FeatureOperations.cut, [patch.faces[0], patch2.faces[0]], [body]
        )
        # remove used patches
        fh.comp_remove(rack, [p.bodies[0] for p in [patch, patch2]])

    # duplicate the tooth groove
    quantity = ceil((length - thickness * tan(abs(angle))) / (pi * m / cos(angle)))
    fh.comp_rectangular_pattern(
        rack, [tooth], rack.yConstructionAxis, 2 * quantity, -pi * m / cos(angle), True, True
    )

    # create joint
    fh.comp_joint_slider(
        wrapper,
        rack.originConstructionPoint,
        wrapper.originConstructionPoint,
        adsk.fusion.JointDirections.YAxisJointDirection,
    )

    # draw the reference line
    sketch2 = rack.sketches.add(rack.xYConstructionPlane)
    line = sketch2.sketchCurves.sketchLines.addByTwoPoints(
        adsk.core.Point3D.create(0, 0, 0),
        adsk.core.Point3D.create(0, -length, 0),
    )
    line.isConstruction = True

    # indicate one pitch
    line = sketch2.sketchCurves.sketchLines.addByTwoPoints(
        adsk.core.Point3D.create(param.mk * param.m, 0, 0),
        adsk.core.Point3D.create(param.mk * param.m, -param.m * pi / cos(param.angle), 0),
    )
    line.isConstruction = True

    fh.sketch_fix_all(sketch2)
