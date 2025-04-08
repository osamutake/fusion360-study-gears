from math import pi, ceil, asin, sin, cos, tan, sqrt, floor
from typing import TypeVar

import adsk.core
import adsk.fusion

from . import gear_curve
from .lib import fusion_helper as fh
from .lib.fusion_helper import Vector
from .lib.function import find_root, minimize
from .lib.spline import interpolate as spline, evenly_spaced_points_on_spline


def gear_crown(
    wrapper_occurrence: adsk.fusion.Occurrence,
    params: gear_curve.GearParams,  # transverse parameters for pinion gear
    z: int,  # number of teeth of crown gear
    w1: float,  # outer width
    w2: float,  # inner width
    helix_angle: float,
):
    num_rotation_step = 600
    num_point_in_profile = 50

    def generate_pinion():
        """Returns pinion tooth profile as a list of Vector"""
        curves = gear_curve.gear_curve(params, params.rc)
        pinion_half: list[Vector] = []
        for j, c in enumerate(curves):  # curves contains arc and function
            if isinstance(c[0], Vector):  # arc
                n = ceil(abs(c[2] - c[1]) / pi * 18 * 4)  # max 2.5 deg step
                for i in range(0, n + 1):
                    pinion_half.append(c[0] + Vector.polar(c[3], c[1] + (c[2] - c[1]) * i / n))
            else:  # function
                n = c[3] * 10
                for i in range(0, n + 1):
                    pinion_half.append(c[0](c[1] + (c[2] - c[1]) * i / n))
            if j < len(curves) - 1:
                pinion_half.pop()  # remove the last point of the previous curve

        # append flipped half of the tooth profile
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

    # Generate two patches of groove profile of crown gear
    # at radial offset t from the crown gear's reference circle
    def generate_patch(t: float, sketch1: adsk.fusion.Sketch, sketch2: adsk.fusion.Sketch):
        """Generate two patches of tooth profile
        at radial offset t from the crown gear's reference circle"""

        rp2t = rp2 + t  # radius of interest

        def calc_tooth_profiles():
            """Calc groove profiles"""
            rp2t2 = rp2t**2

            # Turn the pinion and remove the pinion profile from the crown gear profile.
            # Crown gear profile is calculated with dx as interval.
            # Removed area is between min_y and max_y.
            # x is the height of the crown tooth profile in mm.
            # x is measured from the bottom of the crown gear tooth profile.
            # y is the angle of the pinion tooth profile in radian around the crown gear axis.

            nx = 201  # number of points along the height direction
            # Calculating region is extended by m/10 to make the groove profile
            # protruding from the top surface by this amount.
            dx = (mk + mf + params.m / 10) / (nx - 1)

            def one_step(theta: float, min_y: list[float], max_y: list[float]):
                """remove pinion profile at pinion angle theta from the crown gear"""

                def rotate_pinion(v: Vector, theta: float, t: float):
                    """rotate pinion by -theta with turning the crown gear by phi"""

                    v = v.rotate(-theta + t * factor)
                    phi = (theta * params.z) / z
                    if factor == 0:
                        return Vector(v.x, phi + asin(v.y / rp2t))
                    vx = v.x
                    vy = v.y

                    # move v along thread helix by dt in the direction of the pinion axis
                    # return the distance between the point on the thread helix
                    # and the cylindrical surface that has the same y coordinate as the point
                    def dt_error(dt: float):
                        a = -dt * factor
                        y = vx * sin(a) + vy * cos(a)
                        return rp2t - dt - sqrt(rp2t2 - y**2)

                    # find intersection of thread helix with cylindrical surface
                    ddt = params.m / 10
                    dt = 0.0
                    while dt_error(dt) > 0:
                        dt += ddt
                    dt = find_root(dt - ddt, dt, dt_error)
                    v = v.rotate(-dt * factor)
                    return Vector(v.x, phi + asin(v.y / rp2t))

                def cut_crown_gear(rotated: list[Vector]):
                    def cut_one_point(j: int, k: int):
                        """linear interpolate the segment j for x[k]
                        and remove the point from the crown gear profile"""
                        xj = rotated[j].x
                        yj = rotated[j].y
                        xj1 = rotated[j - 1].x
                        yj1 = rotated[j - 1].y
                        y = yj1 + ((rt - k * dx - xj1) * (yj - yj1)) / (xj - xj1)
                        if y < min_y[k]:
                            min_y[k] = y
                        if y > max_y[k]:
                            max_y[k] = y

                    max_x = rotated[0].x
                    for j in range(1, len(rotated)):
                        k1 = (rt - rotated[j - 1].x) / dx
                        k2 = (rt - rotated[j].x) / dx
                        for k in range(ceil(min(k1, k2)), floor(max(k1, k2)) + 1):
                            if 0 <= k < nx:
                                cut_one_point(j, k)
                        max_x = max(max_x, rotated[j].x)

                    return max_x

                rotated = [rotate_pinion(v, theta, t) for v in pinion]
                max_x = cut_crown_gear(rotated)
                return max_x

            def create_profile(min_y: list[float], max_y: list[float]):
                """create the tooth profile as list[Vector] from min_y and max_y"""
                # append x axis
                # y is still the angle around the crown gear axis with radius (rp2 + t)
                result = [
                    Vector(rt - dx * i, rp2t * y)
                    for i, y in reversed(list(enumerate(min_y)))
                    if -pi / 2 < y < pi / 2
                ] + [
                    Vector(rt - dx * i, rp2t * y)
                    for i, y in enumerate(max_y)  #
                    if -pi / 2 < y < pi / 2
                ]

                # remove duplicated points
                for i in range(len(result) - 1, 0, -1):
                    if abs(result[i] - result[i - 1]) < params.m / 1000:
                        result.pop(i)

                return result

            # step size of theta
            d_theta = pi * 2 / z**0.5 / num_rotation_step

            # make pinion's tooth is normal to the crown gear at rp2 + t and
            # turn pinion until the pinion tooth tip get out of the crown gear top surface
            min_y1 = [pi / 2 for _ in range(nx)]
            max_y1 = [-pi / 2 for _ in range(nx)]
            theta = t * factor
            while one_step(theta, min_y1, max_y1) > rt - mf - mk:
                theta -= d_theta

            # make pinion's tooth is normal to the crown gear at rp2 + t and
            # turn pinion until the pinion tooth tip get out of the crown gear top surface
            min_y2 = [pi / 2 for _ in range(nx)]
            max_y2 = [-pi / 2 for _ in range(nx)]
            theta = t * factor
            while one_step(theta, min_y2, max_y2) > rt - mf - mk:
                theta += d_theta

            involute = create_profile(min_y1, max_y2)
            undercut = create_profile(min_y2, max_y1)

            # Set back the undercut curve by params.m / 200
            # to avoid the coincidence of the two profiles
            undercut = [Vector(v.x + params.m / 200, v.y) for v in undercut]

            return involute, undercut

        def calc_spline(profile: list[Vector]):
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
                    ts.append(ts[-1] + abs(reduced[i][1] - reduced[i - 1][1]))
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

        # def evenly_spaced(profile: list[Vector], n: int):
        #     """Calc evenly spaced points along the curve"""
        #     lengths = [0.0]
        #     for i in range(1, len(profile)):
        #         lengths.append(lengths[-1] + abs(profile[i] - profile[i - 1]))
        #     lengths = [l / lengths[-1] for l in lengths]  # 0 to 1
        #     result = [profile[0]]
        #     j = 0
        #     for i in range(1, n - 1):
        #         t = i / (n - 1)
        #         while lengths[j] < t:
        #             j += 1
        #         t1 = (t - lengths[j - 1]) / (lengths[j] - lengths[j - 1])
        #         result.append(profile[j - 1] + t1 * (profile[j] - profile[j - 1]))
        #     result.append(profile[-1])
        #     return result

        def generate_patch_core(profile: list[Vector], sketch: adsk.fusion.Sketch):
            curve = calc_spline(profile)  # create spline
            curve = evenly_spaced_points_on_spline(
                curve, num_point_in_profile
            )  # evenly spaced points
            # profile = evenly_spaced(profile, 50)

            curve = [
                Vector(
                    rp2t * cos(v.y / rp2t),
                    rp2t * sin(v.y / rp2t),
                    -v.x,
                )
                for v in curve
            ]  # transform to the cylindrical surface

            # draw the profile on the cylindrical surface
            c1 = fh.sketch_fitted_splines(sketch, curve)
            arc = sketch.sketchCurves.sketchArcs.addByCenterStartEnd(
                fh.point3d(0, 0, c1.endSketchPoint.geometry.z),
                c1.startSketchPoint,
                c1.endSketchPoint,
            )
            fh.sketch_fix_all(sketch)

            # create a patch from the sketch curves making a closed loop
            return fh.comp_patch(gear, arc, fh.FeatureOperations.new_body).bodies[0]

        involute, undercut = calc_tooth_profiles()
        return generate_patch_core(involute, sketch1), generate_patch_core(undercut, sketch2)

    def generate_donut_body():
        """extrude the donut between inner and outer circles"""
        sketch = gear.sketches.add(gear.xYConstructionPlane)
        sketch = sketch.createForAssemblyContext(gear_occurrence)
        sketch.isComputeDeferred = True
        sketch.sketchCurves.sketchCircles.addByCenterRadius(
            fh.point3d(0, 0, -(rp + params.shift * params.m)), rp2 - w2 * params.m
        )
        sketch.sketchCurves.sketchCircles.addByCenterRadius(
            fh.point3d(0, 0, -(rp + params.shift * params.m)), rp2 + w1 * params.m
        )
        fh.sketch_fix_all(sketch)
        sketch.isComputeDeferred = False

        return fh.comp_extrude(
            gear,
            sketch.profiles[1],
            fh.FeatureOperations.new_body,
            (params.m * params.mk, params.m * (params.mf + 1)),
        ).bodies[0]

    def draw_reference_circle_and_axis():
        """Draw reference circle and axis of the crown gear.
        Returns the center of the circle."""

        sketch = gear.sketches.add(gear.xYConstructionPlane)
        sketch = sketch.createForAssemblyContext(gear_occurrence)
        sketch.isComputeDeferred = True
        ref = sketch.sketchCurves.sketchCircles.addByCenterRadius(
            fh.point3d(0, 0, -(rp + params.shift * params.m)), rp2
        )
        ref.isConstruction = True
        ref.isFixed = True
        axis = sketch.sketchCurves.sketchLines.addByTwoPoints(
            fh.point3d(0, 0, -(rp + (params.shift - params.mk) * params.m)),
            fh.point3d(0, 0, -(rp + (params.shift + params.mf) * params.m)),
        )
        axis.isConstruction = True
        fh.sketch_fix_all(sketch)
        sketch.isComputeDeferred = False

        return ref.centerSketchPoint

    def adjust_camera():
        """move the camera to allow user to see the generation of patches"""

        app = adsk.core.Application.get()
        view = app.activeViewport
        cam = view.camera
        T = TypeVar("T", adsk.core.Point3D, adsk.core.Vector3D)

        def transform(p: T) -> T:
            p.transformBy(wrapper_occurrence.transform2)
            return p

        cam.eye = transform(adsk.core.Point3D.create(-rp2, 0, 2 * rp2 - rt))
        cam.target = transform(adsk.core.Point3D.create(rp2, 0, -rt))
        cam.upVector = transform(adsk.core.Vector3D.create(0, 0, 1))
        cam.isSmoothTransition = True
        cam.perspectiveAngle = 2 * pi / z
        view.camera = cam

    adjust_camera()

    # wrapper component
    wrapper = wrapper_occurrence.component
    gear_occurrence = wrapper.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    gear_occurrence = gear_occurrence.createForAssemblyContext(wrapper_occurrence)
    gear_occurrence.isGroundToParent = False
    gear = gear_occurrence.component

    donut = generate_donut_body()
    center = draw_reference_circle_and_axis()
    fh.comp_built_joint_revolute(wrapper, gear_occurrence, wrapper_occurrence, center)

    sketch1 = gear.sketches.add(gear.xYConstructionPlane)
    sketch1.isComputeDeferred = True
    sketch2 = gear.sketches.add(gear.xYConstructionPlane)
    sketch2.isComputeDeferred = True

    patches: list[tuple[adsk.fusion.BRepBody, adsk.fusion.BRepBody]] = []

    # generate the groove cross sections
    extension = 1.02
    phi_range = extension * (w2 + w1) * params.m * factor * params.z / z
    n = max(ceil((w2 + w1) / 0.5), ceil(phi_range / pi * 180 / 5))
    for i in range(0, n + 1):
        t = extension * (-w2 + (w2 + w1) * i / n)
        patches.append(generate_patch(t * params.m, sketch1, sketch2))
        fh.app_refresh()

    for sk in [sketch1, sketch2]:
        sk.isComputeDeferred = False
        sk.isVisible = False

    # cut the tooth groove from the donut
    loft1 = fh.comp_loft(gear, fh.FeatureOperations.cut, [p[0].faces[0] for p in patches], donut)
    loft2 = fh.comp_loft(gear, fh.FeatureOperations.cut, [p[1].faces[0] for p in patches], donut)
    fh.app_refresh()

    # remove the used patches
    fh.comp_remove(gear, [p[0] for p in patches])
    fh.comp_remove(gear, [p[1] for p in patches])

    # create a circular pattern of the tooth groove
    fh.comp_circular_pattern(gear, [loft1, loft2], gear.zConstructionAxis, z)

    return gear
