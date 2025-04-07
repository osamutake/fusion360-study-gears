from math import pi, ceil, asin, sin, cos, tan, sqrt, floor
from typing import TypeVar

import adsk.core
import adsk.fusion

from . import gear_curve
from .lib import fusion_helper as fh
from .lib.fusion_helper import Vector
from .lib.function import find_root, minimize
from .lib.spline import interpolate as spline


def gear_crown(
    comp_occurrence: adsk.fusion.Occurrence,
    params: gear_curve.GearParams,  # transverse parameters for pinion gear
    z: int,  # number of teeth of crown gear
    w1: float,  # outer width
    w2: float,  # inner width
    helix_angle: float,
):

    def generate_pinion():
        """Returns pinion tooth profile as a list of Vector"""
        # half of pinion tooth profile
        pinion_curves = gear_curve.gear_curve(params, params.rc)

        # convert curves to a list of Vector
        def build_curve():
            curve: list[Vector] = []
            for j, c in enumerate(pinion_curves):
                if isinstance(c[0], Vector):
                    n = ceil(abs(c[2] - c[1]) / pi * 18 * 4)  # max 2.5 deg step
                    for i in range(0, n + 1):
                        curve.append(c[0] + Vector.polar(c[3], c[1] + (c[2] - c[1]) * i / n))
                else:
                    n = c[3] * 10
                    for i in range(0, n + 1):
                        curve.append(c[0](c[1] + (c[2] - c[1]) * i / n))
                if j < len(pinion_curves) - 1:
                    curve.pop()  # remove the last point of the previous curve
            return curve

        # append flipped half of the tooth profile
        pinion_half = build_curve()
        pinion_half2 = [v.flip_y() for v in reversed(pinion_half)]
        pinion_half.pop(0)
        return pinion_half2 + pinion_half

    pinion = generate_pinion()

    rp = params.m * params.z / 2  # pinion
    rp2 = params.m * z / 2  # crown gear
    mk = params.mk * params.m
    mf = params.mf * params.m
    rt = rp + mf + params.shift * params.m  # extended tip circle radius
    tan_beta = tan(helix_angle)
    factor = tan_beta / rp

    # calc tooth profile at radial offset t from the crown gear's reference circle
    # t: radial offset from the crown gear's reference circle
    def calc_tooth(t: float):
        """Calc tooth profile at radial offset t from the crown gear's reference circle"""
        rp2t = rp2 + t  # radius of interest
        rp2t2 = rp2t**2

        # turn the pinion and remove the pinion profile from the crown gear
        # crown gear tooth profile is calculated with dx as interval
        # removed area is between min_y and max_y
        # x is the height of the crown tooth profile in mm
        # x is measured from the bottom of the crown gear tooth profile
        # y is the angle of the pinion tooth profile in radian around the crown gear axis

        nx = 201  # number of points along the height direction
        dx = (mk + mf) / (nx - 1)
        min_y = [pi / 2 for _ in range(nx)]
        max_y = [-pi / 2 for _ in range(nx)]

        # remove pinion profile at pinion angle theta from the crown gear
        def one_step(theta: float):

            # turn pinion by -theta with turning the crown gear by phi
            def trans(v: Vector, theta: float, t: float):
                v = v.rotate(-theta + t * factor)
                phi = (theta * params.z) / z
                if factor == 0:
                    return Vector(v.x, phi + asin(v.y / rp2t))
                vx = v.x
                vy = v.y

                # v からピニオンの軸方向に dt だけ
                # はすばピニオンの歯筋に沿って移動する
                # 半径 rp2t の円筒面上でその y 座標を持つ点と
                # 歯筋上の点との距離を求める
                def dt_error(dt: float):
                    a = -dt * factor
                    y = vx * sin(a) + vy * cos(a)
                    return rp2t - dt - sqrt(rp2t2 - y**2)

                # 歯筋と円筒面との交点を求める
                ddt = params.m / 10
                dt = 0.0
                while dt_error(dt) > 0:
                    dt += ddt
                dt = find_root(dt - ddt, dt, dt_error)
                v = v.rotate(-dt * factor)
                return Vector(v.x, phi + asin(v.y / rp2t))

            # linear interpolation of section j for x[k]
            # and remove the point from the crown gear profile
            def cut_crown_gear(j: int, k: int):
                xj = translated[j].x
                yj = translated[j].y
                xj1 = translated[j - 1].x
                yj1 = translated[j - 1].y
                y = yj1 + ((rt - k * dx - xj1) * (yj - yj1)) / (xj - xj1)
                if y < min_y[k]:
                    min_y[k] = y
                if y > max_y[k]:
                    max_y[k] = y

            translated = [trans(v, theta, t) for v in pinion]
            max_x = translated[0].x
            for j in range(1, len(translated)):
                k1 = (rt - translated[j - 1].x) / dx
                k2 = (rt - translated[j].x) / dx
                for k in range(ceil(min(k1, k2)), floor(max(k1, k2)) + 1):
                    if 0 <= k < nx:
                        cut_crown_gear(j, k)
                max_x = max(max_x, translated[j].x)

            return max_x

        def distance(v1: Vector, v2: Vector):
            """y is the angle around the crown gear axis with
            radius (rp2 + t)"""
            return sqrt((v1.x - v2.x) ** 2 + rp2t2 * (v1.y - v2.y) ** 2)

        def create_profile():
            """create the tooth profile as list[Vector] from min_y and max_y"""
            # append x axis
            # y is still the angle around the crown gear axis with radius (rp2 + t)
            result = [Vector(rt - dx * i, y) for i, y in reversed(list(enumerate(min_y)))] + [
                Vector(rt - dx * i, y) for i, y in enumerate(max_y)
            ]

            # remove duplicated points
            result = [v for v in result if -pi / 2 < v.y < pi / 2]
            for i in range(len(result) - 1, 0, -1):
                if distance(result[i], result[i - 1]) < params.m / 1000:
                    result.pop(i)

            return result

        def interpolate(profile: list[Vector]):
            """approximate the tooth profile with a spline with reduced the number of control points"""
            n = 20  # number of evenly spaced points
            m = 30  # number of total points including the additional points

            # first, n points evenly spaced in x direction are selected
            reduced = [(0, profile[0])]
            for j in range(1, n):
                i = floor((len(profile) * j) / n)
                reduced.append((i, profile[i]))
            reduced.append((len(profile) - 1, profile[len(profile) - 1]))

            # 近似曲線から最も離れているところに点を増やす
            def add_one_point():
                """add a point that is farthest from the approximated curve
                to the list of control points"""
                err_max = 0.0
                err_i = 0

                # spline curve is created from the control points
                ts = [0.0]
                for i in range(1, len(reduced)):
                    ts.append(ts[-1] + distance(reduced[i][1], reduced[i - 1][1]))
                cx = spline(ts, [r[1].x for r in reduced])
                cy = spline(ts, [r[1].y for r in reduced])

                # the point that is farthest from the spline curve for every profile[i]
                for j in range(1, len(reduced)):
                    i1 = reduced[j - 1][0]
                    i2 = reduced[j][0]
                    for i in range(i1 + 1, i2):
                        x, y = profile[i].x, profile[i].y
                        t = minimize(
                            # @pylint: disable=cell-var-from-loop
                            ts[j - 1],
                            ts[j],
                            lambda t: (x - cx(t)) ** 2 + (y - cy(t)) ** 2,
                        )
                        err = sqrt((x - cx(t)) ** 2 + (y - cy(t)) ** 2)
                        if err > err_max:
                            err_max = err
                            err_i = i

                # append the point to the list of control points
                reduced.append((err_i, profile[err_i]))
                reduced.sort(key=lambda a: a[0])
                return err_max

            while len(reduced) < m:
                add_one_point()
            return [r[1] for r in reduced]

        dt = pi * 2 / z**0.5 / 600

        # make pinion's tooth is normal to the crown gear at rp2 + t
        theta = t * factor
        min_y = [pi / 2 for _ in range(nx)]
        max_y = [-pi / 2 for _ in range(nx)]
        # turn pinion until the pinion tooth tip get out of the crown gear top surface
        while one_step(theta) > rt - mf - mk:
            theta -= dt
        profile1 = create_profile()

        # make pinion's tooth is normal to the crown gear at rp2 + t
        theta = t * factor
        min_y = [pi / 2 for _ in range(nx)]
        max_y = [-pi / 2 for _ in range(nx)]
        # turn pinion until the pinion tooth tip get out of the crown gear top surface
        while one_step(theta) > rt - mf - mk:
            theta += dt
        profile2 = create_profile()

        profile1 = interpolate(profile1)
        profile2 = interpolate(profile2)
        return profile1, profile2

    # wrapper component
    comp = comp_occurrence.component

    gear_occurrence = comp.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    gear_occurrence.createForAssemblyContext(comp_occurrence)
    gear = gear_occurrence.component

    sketch11 = gear.sketches.add(gear.xYConstructionPlane)
    sketch11.isComputeDeferred = True
    sketch21 = gear.sketches.add(gear.xYConstructionPlane)
    sketch21.isComputeDeferred = True

    # move the camera to allow user to see the generation of patches
    app = adsk.core.Application.get()
    view = app.activeViewport
    cam = view.camera
    T = TypeVar('T', adsk.core.Point3D, adsk.core.Vector3D)
    def transform(p: T) -> T:
        p.transformBy(comp_occurrence.transform2)
        return p
    cam.eye = transform(adsk.core.Point3D.create(-rp2, 0, 2 * rp2 - rt))
    cam.target = transform(adsk.core.Point3D.create(rp2, 0, -rt))
    cam.upVector = transform(adsk.core.Vector3D.create(0, 0, 1))
    cam.isSmoothTransition = True
    cam.perspectiveAngle = 2 * pi / z
    view.camera = cam

    # generate a patch of tooth profile
    # t: radial offset from the crown gear's reference circle
    def generate_patch(t: float):
        rp2t = rp2 + t
        rp2t2 = rp2t**2

        # calc evenly spaced control points along the spline curve
        def evenly_spaced(points: list[Vector], n: int):

            def distance(v1: Vector, v2: Vector):
                return sqrt((v1.x - v2.x) ** 2 + rp2t2 * (v1.y - v2.y) ** 2)

            # パラメータを 0 から 1 としてスプライン曲線を作成
            ts = [0.0]
            for i in range(1, len(points)):
                ts.append(ts[-1] + distance(points[i], points[i - 1]))
            total = ts[-1]
            ts = [t / total for t in ts]  # 0 to 1
            cx = spline(ts, [r.x for r in points])
            cy = spline(ts, [r.y for r in points])

            # n * 10 個の点を作って距離を求める
            m = n * 10
            lengths = [0.0]
            last = points[0]
            for i in range(1, m):
                t = i / (m - 1)
                curr = Vector(cx(t), cy(t))
                lengths.append(lengths[-1] + distance(curr, last))
                last = curr
            total = lengths[-1]
            lengths = [l / total for l in lengths]  # 0 to 1

            result = [points[0]]
            j = 1
            for i in range(1, n - 1):
                t = i / (n - 1)
                while lengths[j] < t:
                    j += 1
                t1 = (t - lengths[j - 1]) / (lengths[j] - lengths[j - 1])
                result.append(Vector(cx((j + t1) / m), cy((j + t1) / m)))
            result.append(points[-1])

            return result

        def generate_patch_core(profile: list[Vector], sketch: adsk.fusion.Sketch):
            profile = evenly_spaced(profile, 50)
            tooth = [Vector(rp2t * cos(v.y), rp2t * sin(v.y), -v.x) for v in profile]
            c1 = sketch.sketchCurves.sketchFittedSplines.add(
                fh.collection([fh.point3d(v) for v in tooth])
            )

            # extend the profile upward to make clear intersection with the top face
            l1 = sketch.sketchCurves.sketchLines.addByTwoPoints(
                c1.startSketchPoint,
                fh.point3d(Vector(c1.startSketchPoint.geometry) + Vector(0, 0, params.m / 10)),
            )
            l2 = sketch.sketchCurves.sketchLines.addByTwoPoints(
                c1.endSketchPoint,
                fh.point3d(Vector(c1.endSketchPoint.geometry) + Vector(0, 0, params.m / 10)),
            )
            arc = sketch.sketchCurves.sketchArcs.addByCenterStartEnd(
                fh.point3d(0, 0, l1.endSketchPoint.geometry.z),
                l1.endSketchPoint,
                l2.endSketchPoint,
            )
            fh.sketch_fix_all(sketch)

            # create a patch from the sketch curves making a closed loop
            return fh.comp_patch(gear, arc, fh.FeatureOperations.new_body).bodies[0]

        profile1, profile2 = calc_tooth(t)
        return generate_patch_core(profile1, sketch11), generate_patch_core(profile2, sketch21)

    patches: list[tuple[adsk.fusion.BRepBody, adsk.fusion.BRepBody]] = []

    # generate the tooth profile
    extension = 1.02
    phi_range = extension * (w2 + w1) * params.m * factor * params.z / z
    n = max(ceil((w2 + w1) / 0.5), ceil(phi_range / pi * 180 / 5))
    for i in range(0, n + 1):
        t = extension * (-w2 + (w2 + w1) * i / n)
        patches.append(generate_patch(t * params.m))
        fh.app_refresh()

    for sk in [sketch11, sketch21]:
        sk.isComputeDeferred = False
        sk.isVisible = False

    # extrude the donut between inner and outer circles
    sketch1 = comp.sketches.add(comp.xYConstructionPlane)
    sketch1.isComputeDeferred = True
    sketch1.sketchCurves.sketchCircles.addByCenterRadius(
        fh.point3d(0, 0, -(rp + params.shift * params.m)), rp2 - w2 * params.m
    )
    sketch1.sketchCurves.sketchCircles.addByCenterRadius(
        fh.point3d(0, 0, -(rp + params.shift * params.m)), rp2 + w1 * params.m
    )
    fh.sketch_fix_all(sketch1)
    sketch1.isComputeDeferred = False
    body = fh.comp_extrude(
        gear,
        sketch1.profiles[1],
        fh.FeatureOperations.new_body,
        (params.m * params.mk, params.m * (params.mf + 1)),
    ).bodies[0]

    # cut the tooth groove from the donut
    loft1 = fh.comp_loft(gear, fh.FeatureOperations.cut, [p[0].faces[0] for p in patches], body)
    loft2 = fh.comp_loft(gear, fh.FeatureOperations.cut, [p[1].faces[0] for p in patches], body)
    fh.app_refresh()

    # remove the used patches
    fh.comp_remove(gear, [p[0] for p in patches])
    fh.comp_remove(gear, [p[1] for p in patches])

    # create a circular pattern of the tooth groove
    fh.comp_circular_pattern(gear, [loft1, loft2], gear.zConstructionAxis, z)

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
    fh.sketch_fix_all(sketch2)
    sketch2.isComputeDeferred = False
