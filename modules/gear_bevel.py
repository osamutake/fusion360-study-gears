from math import asin, acos, atan, tan, sin, cos, pi, atan2, sqrt, ceil, log
from collections.abc import Iterable
from typing import Literal

import adsk.core, adsk.fusion

from .lib import fusion_helper as fh
from .lib.fusion_helper import Vector
from .lib.function import minimize


class Params:
    def __init__(
        self,
        axes_angle: float,
        module: float,
        z1: int,
        z2: int,
        beta: float,  # helix angle
        addendum: float,
        dedendum: float,
        pressure_angle: float,
        width: float,
        backlash: float,
    ):
        # calculate transverse values for spiral gears
        self.sigma = axes_angle
        self.m = module / cos(beta)
        self.mf = dedendum * cos(beta)
        self.mk = addendum * cos(beta)
        self.alpha = atan(tan(pressure_angle) / cos(beta))
        self.beta = beta

        self.width = width
        self.backlash = backlash

        # larger gear is z1, smaller gear is z2
        self.z1 = max(z1, z2)
        self.z2 = min(z1, z2)

        self.calculate_params()

    def calculate_params(self):
        self.gamma_p1 = atan2(sin(self.sigma), self.z2 / self.z1 + cos(self.sigma))
        self.gamma_p2 = self.sigma - self.gamma_p1

        self.internal = 0
        if self.gamma_p1 > pi / 2:
            self.gamma_p1 = pi - self.gamma_p1
            self.internal = 1

        self.r = self.z1 / (2 * sin(self.gamma_p1))
        self.r0 = self.r * self.m
        self.rm = 1 / self.r

        self.gamma_f1 = self.gamma_p1 - self.mf * atan(self.rm)
        self.gamma_k1 = self.gamma_p1 + self.mk * atan(self.rm)
        self.gamma_t1 = self.gamma_p1 + self.mf * atan(self.rm)
        self.gamma_b1 = asin(cos(self.alpha) * sin(self.gamma_p1))

        self.gamma_f2 = self.gamma_p2 - self.mf * atan(self.rm)
        self.gamma_k2 = self.gamma_p2 + self.mk * atan(self.rm)
        self.gamma_t2 = self.gamma_p2 + self.mf * atan(self.rm)
        self.gamma_b2 = asin(cos(self.alpha) * sin(self.gamma_p2))

        if self.internal == 1:
            self.gamma_k1 = self.gamma_p1 + self.mf * atan(self.rm)
            self.gamma_f1 = max(self.gamma_f1, self.gamma_b1)

        self.axis1 = Vector(cos(self.gamma_p1), 0, sin(self.gamma_p1))
        self.axis2 = Vector(cos(self.gamma_p2), 0, -sin(self.gamma_p2))


def gear_curves(params: Params):
    """Generate the involute and trochoid curves for the two gears."""

    def trans(epsilon: float, v: Vector, axis: Vector, gamma_b: float):
        """transform v along base circle by angle specified by epsilon
        and then pushed back along the great circle by same distance"""
        axis = axis.normalize(cos(gamma_b))
        v = v.normalize()
        oq = v - axis
        om = oq.rotate_axis(axis, epsilon)
        oom = axis + om
        axis2 = oom - axis.normalize(1 / cos(gamma_b))
        op = oom.rotate_axis(axis2, epsilon * sin(gamma_b))
        return op

    def gamma2theta(gamma: float, gamma_b: float):
        """gamma_b is the angular radius of the base circle.
        gamma is the angular radius of the point of interest.
        theta is the angle around the axis of the gear
        between the point of interest and the original point."""
        varphi = acos(cos(gamma) / cos(gamma_b))
        return varphi / sin(gamma_b) - atan2(tan(varphi), sin(gamma_b))

    def gamma2epsilon(gamma: float, gamma_b: float):
        """epsilon is the angle of the gear rotation
        needed to bring the original point to the point of interest."""
        varphi = acos(cos(gamma) / cos(gamma_b))
        return varphi / sin(gamma_b)

    def spherical_involute(
        axis: Vector, v: Vector, gamma_b: float, gamma_s: float, gamma_e: float, n=20
    ):
        """v should be a point on the base circle.
        gamma_b is the angular radius of the base circle.
        returns a list of points on the involute curve
        from the angular radius gamma_s to gamma_e."""
        points: list[Vector] = []
        ts = gamma2epsilon(gamma_s, gamma_b)
        te = gamma2epsilon(gamma_e, gamma_b)
        for i in range(0, n + 1):
            t = ts + ((te - ts) / n) * i
            points.append(trans(t, v, axis, gamma_b))
        return points

    # calculate the trace of point t1 on the rotational coordinate system of gear2
    def spherical_trochoid(
        t1: Vector,  # tip point on the extended tip circle
        q2: Vector,  # original point on the base circle of the 2nd gear
        axis1: Vector,  # axis of the 1st gear
        axis2: Vector,  # axis of the 2nd gear
        z1: float,  # number of teeth of the 1st gear
        z2: float,  # number of teeth of the 2nd gear
        tooth2: list[Vector],  # involute curve of the 2nd gear
        gamma_b2: float,  # angular radius of the base circle of the 2nd gear
        gamma_f2: float,  # angular radius of the bottom circle of the 2nd gear
        n=10,
    ):
        def trochoid_core(t: float):
            return t1.rotate_axis(axis1, -t).rotate_axis(axis2, (-t / z2) * z1)

        # sweep t from 0 until getting out of the tip circle
        i = 0
        while True:
            t = (0.02 * (i * pi)) / sqrt(z1)
            if (trochoid_core(t) - axis2).norm() > (tooth2[-1] - axis2).norm():
                t1b = t  # tip circle
                break
            i += 1

        # bottom circle: find the point with minimum gamma
        t1a = minimize(0, t1b, lambda t: (trochoid_core(t) - axis2).norm())

        if gamma_b2 > gamma_f2:
            # cross section with the base circle
            t1c = minimize(
                t1a,
                t1b,
                lambda t: abs((trochoid_core(t) - axis2).norm() - (q2 - axis2).norm()),
            )
        else:
            # bottom circle
            t1c = t1a

        def distance_from_involute(t: float):
            """Measure the distance from the involute curve at the same gamma value"""
            v = trochoid_core(t)
            gamma = acos(v.dot(axis2))
            # involute curve of gear2
            u = trans(gamma2epsilon(gamma, gamma_b2), q2, axis2, gamma_b2)
            return (u - v).norm()

        # find the point on the trochoid curve that is closest to the involute curve
        # it may not a cross point but a tangent point.
        t1d = minimize(t1c, t1b, distance_from_involute)

        # use the part from bottom circle (t1a) to the contact point with involute (t1d)
        c1: list[Vector] = []  # trochoid curve
        for i in range(0, n + 1):
            t = t1a + ((t1d - t1a) * i) / n
            c1.append(trochoid_core(t))

        gamma = acos(c1[-1].dot(axis2))  # for contact point with involute curve
        return (c1, gamma)

    ext = params.rm * 0.2  # extension of the involute curve

    # p1 is a point on the base circle of the 1st gear
    p1 = Vector(x=1).rotate_axis(Vector(y=1), params.gamma_p1 - params.gamma_b1)
    # q1 is the point on the base circle of the 1st gear,
    # the involute curve from which pass through the reference point (1, 0, 0),
    # where the two gears touch each other on the reference circle.
    q1 = p1.rotate_axis(params.axis1, -gamma2theta(params.gamma_p1, params.gamma_b1))
    # get involute curve from q1,
    # from the base circle (gamma_b1) or the bottom circle (gamma_f1)
    # to the tip circle (gamma_t1).
    involute1 = spherical_involute(
        params.axis1,
        q1,
        params.gamma_b1,
        max(params.gamma_b1, params.gamma_f1),
        params.gamma_k1 + ext,
    )
    # t1 is the point at the cross section of the involute curve and the extended tip circle
    t1 = trans(gamma2epsilon(params.gamma_t1, params.gamma_b1), q1, params.axis1, params.gamma_b1)

    # same procedure for the 2nd gear
    p2 = Vector(x=1).rotate_axis(Vector(y=1), -(params.gamma_p2 - params.gamma_b2))
    q2 = p2.rotate_axis(params.axis2, -gamma2theta(params.gamma_p2, params.gamma_b2))
    involute2 = spherical_involute(
        params.axis2,
        q2,
        params.gamma_b2,
        max(params.gamma_b2, params.gamma_f2),
        params.gamma_k2 + ext,
    )
    t2 = trans(gamma2epsilon(params.gamma_t2, params.gamma_b2), q2, params.axis2, params.gamma_b2)

    # generate trochoid curve and determine the intersection with involute curve
    if params.internal != 1:
        trochoid1, gamma1 = spherical_trochoid(
            t2,
            q1,
            params.axis2,
            params.axis1,
            params.z2,
            params.z1,
            involute1,
            params.gamma_b1,
            params.gamma_f1,
        )
        involute1 = spherical_involute(
            params.axis1, q1, params.gamma_b1, gamma1, params.gamma_k1 + ext
        )
    else:
        trochoid1 = [involute1[0]]

    trochoid2, gamma2 = spherical_trochoid(
        t1,
        q2,
        params.axis1,
        params.axis2,
        params.z1,
        params.z2,
        involute2,
        params.gamma_b2,
        params.gamma_f2,
    )
    involute2 = spherical_involute(
        params.axis2, q2, params.gamma_b2, gamma2, params.gamma_k2 + ext
    )

    return trochoid1, involute1, trochoid2, involute2


def tooth_groove(
    params: Params,
    trochoid: list[Vector],
    involute: list[Vector],
    axis: Vector,
    z: float,
) -> list[Iterable[Vector]]:
    """Generate the tooth groove profile by scaling and
    connecting the involute and trochoid curves.
    Returns a list of curves that defines one closed loop."""

    r0 = params.r0
    m = params.m
    backlash = params.backlash
    internal = params.internal

    def apply_backlash(curve: list[Vector], axis: Vector, z: float):
        """Set back the curve by backlash around the axis."""

        def project(v: Vector):
            """Project p onto the plane normal to the axis and normalize."""
            return (v - axis.normalize(v.dot(axis))).normalize()

        def angle(p: Vector):
            """Measure the angle around axis between p and x-axis."""
            return acos(project(p).dot(project(Vector(x=1))))

        # scale by r0 and apply backlash
        b = -backlash if internal else backlash
        curve = [r0 * p.rotate_axis(axis, b / (m * z / 2)) for p in curve]

        # if tip or bottom are overlapped, remove the part
        while len(curve) > 0 and angle(curve[0]) > pi / z / 2:
            curve.pop(0)
        while len(curve) > 0 and angle(curve[-1]) > pi / z / 2:
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
        if great_arc:
            center = Vector()
        else:
            center = axis.normalize(axis.normalize().dot(p1))
        c: list[Vector] = []
        v1 = p1 - center
        v2 = p2 - center
        t = acos(v1.dot(v2) / (v1.norm() * v2.norm()))
        if abs(v1.rotate_axis(axis, -t) - v2) < abs(v1.rotate_axis(axis, t) - v2):
            t = -t  # determine the direction of the arc
        for j in range(0, n + 1):
            t2 = (t * j) / n
            c.append(center + v1.rotate_axis(axis, t2))
        return c

    trochoid1 = apply_backlash(trochoid, axis, z)
    involute1 = apply_backlash(involute, axis, z)
    involute2 = [p.flip_y().rotate_axis(axis, -pi / z) for p in involute1]
    trochoid2 = [p.flip_y().rotate_axis(axis, -pi / z) for p in trochoid1]
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


def generate_gear(
    wrapper_occurrence: adsk.fusion.Occurrence,
    params: Params,
    axis: Vector,
    z: int,
    groove_shape: list[Iterable[Vector]],
    flip: Literal[1, -1],
    phi: float,
    internal: bool,
):
    r0 = params.r0
    m = params.m
    mk = params.mk
    mf = params.mf
    rm = params.rm
    r = params.r
    width = params.width
    beta = params.beta

    wrapper = wrapper_occurrence.component
    gear_occurrence = wrapper.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    gear_occurrence = gear_occurrence.createForAssemblyContext(wrapper_occurrence)
    gear_occurrence.isGroundToParent = False
    gear = gear_occurrence.component

    def generate_patch(groove_shape: list[Iterable[Vector]]):
        """generate the patch for the tooth groove shape"""

        def connect_by_splines(sketch: adsk.fusion.Sketch, tooth: list[Iterable[Vector]]):
            """Generate splines and connect them with each other to make a closed loop."""

            result: list[adsk.fusion.SketchFittedSpline] = []
            for i, curve in enumerate(tooth):
                points = [fh.point3d(p) for p in curve]
                if len(points) < 2:
                    continue
                if i > 0:
                    points[0] = result[-1].endSketchPoint  # replace the first point
                    if i == len(tooth) - 1:
                        points[-1] = result[0].startSketchPoint  # replace the last point
                result.append(sketch.sketchCurves.sketchFittedSplines.add(fh.collection(points)))
            return result

        sketch1 = gear.sketches.add(gear.xYConstructionPlane)
        sketch1.isComputeDeferred = True
        connect_by_splines(sketch1, groove_shape)
        fh.sketch_fix_all(sketch1)
        sketch1.isComputeDeferred = False

        patch = fh.comp_patch(
            gear,
            sketch1.sketchCurves[0],
            fh.FeatureOperations.new_body,
        ).bodies[0]
        sketch1.isVisible = False
        return patch

    def base_donut_and_axis():
        """generate the base body donut of the gear and rotational axis"""

        # draw cross section of the donut
        sketch2 = gear.sketches.add(gear.xYConstructionPlane)
        sketch2.isComputeDeferred = True
        if not internal:
            va = Vector(r0, 0, -mk * m * flip)  # tip
            vb = Vector(r0, 0, (mf + 1) * m * flip)  # bottom
        else:
            va = Vector(r0, 0, mk * m * flip)  # tip
            vb = Vector(r0, 0, -(mf + 1) * m * flip)  # bottom
        vas = va * ((r0 - width) / r0)  # scale the tip point
        vbs = vb * ((r0 - width) / r0)  # scale the bottom point
        la = fh.sketch_line(sketch2, va, vb)
        lb = fh.sketch_line(sketch2, vas, vbs)
        fh.sketch_line(sketch2, la.startSketchPoint, lb.startSketchPoint)  # va, vas
        vax0 = axis.normalize(r0 * sqrt(1 - (rm * z / 2) ** 2))
        vax1 = axis.normalize(axis.normalize().dot(vb))  # projected bottom
        vax2 = axis.normalize(axis.normalize().dot(vbs))  # projected bottom scaled
        if not internal:
            lc = fh.sketch_line(sketch2, la.endSketchPoint, vax1)  # vb, vax1
            ld = fh.sketch_line(sketch2, lb.endSketchPoint, vax2)  # vbs,  vax2
            if abs(vax2 - vax1) > abs(vax0 - vax1):
                le = fh.sketch_line(sketch2, lc.endSketchPoint, ld.endSketchPoint)  # vax1, vax2
            else:
                le = fh.sketch_line(sketch2, lc.endSketchPoint, vax0)  # vax1, vax0
                sketch2.geometricConstraints.addCoincident(ld.endSketchPoint, le)
        else:
            fh.sketch_line(sketch2, la.endSketchPoint, lb.endSketchPoint)  # vb, vbs
            if abs(vax1 - vax2) > abs(vax0 - vax2):
                le = fh.sketch_line(sketch2, vax2, vax1)
            else:
                le = fh.sketch_line(sketch2, vax2, vax0)
            vc = Vector(r0, 0, -mf * m * flip)  # tooth bottom
            vcs = vc * ((r0 - width) / r0)  # scale vc
            fh.sketch_line(sketch2, vc, vcs)

        fh.sketch_fix_all(sketch2)
        sketch2.isComputeDeferred = False

        # create the donut by revolving the sketch around the axis
        donut = fh.comp_revolve(
            gear,
            sketch2.profiles,
            le,
            fh.FeatureOperations.new_body,
        ).bodies[0]

        donut2 = None
        if internal:  # thinner donut upto tooth bottom
            donut2 = fh.comp_revolve(
                gear,
                sketch2.profiles[0],
                le,
                fh.FeatureOperations.new_body,
            ).bodies[0]

        # construct plane for sketch to show the reference circle and axis of the gear
        distance = abs(Vector(le.startSketchPoint.geometry) - vax0)
        length = le.startSketchPoint.geometry.distanceTo(le.endSketchPoint.geometry)
        inp = gear.constructionPlanes.createInput(gear_occurrence)
        inp.setByDistanceOnPath(le, fh.value_input(distance / length))
        plane = gear.constructionPlanes.add(inp)

        # draw the reference circle and axis of the gear
        sketch3 = gear.sketches.add(plane)
        sketch3.isComputeDeferred = True
        reference_circle = sketch3.sketchCurves.sketchCircles.addByCenterRadius(
            fh.point3d(), r0 * rm * z / 2
        )
        reference_circle.isConstruction = True
        ax = sketch3.sketchCurves.sketchLines.addByTwoPoints(
            fh.point3d(z=-distance),
            fh.point3d(z=max(0, length - distance)),
        )
        ax.isConstruction = True
        fh.sketch_fix_all(sketch3)
        sketch3.isComputeDeferred = False

        return donut, donut2, ax.createForAssemblyContext(gear_occurrence)

    def duplicate_and_loft(
        patch: adsk.fusion.BRepBody,
        axis_line: adsk.fusion.SketchLine,
        donut: adsk.fusion.BRepBody,
        operation: int,
    ):
        def spiral_angle(scale: float):
            return flip * 2 * r * tan(beta) / z * log(scale)

        # determine the lofting range of the tooth groove shape
        extension = 1.02
        scale_start = extension
        scale_end = (r0 - width) / r0 / extension
        scale_n = 1 if beta == 0 else max(5, ceil(abs(spiral_angle(scale_end)) / pi * 40))

        # create the tooth patches for lofting by scaling and rotating the original patch
        patches: list[adsk.fusion.BRepBody] = []
        for i in range(scale_n + 1):
            scale = scale_start * (scale_end / scale_start) ** (i / scale_n)
            copy = fh.comp_copy(gear, patch)
            fh.comp_scale(gear, copy.bodies[0], gear.originConstructionPoint, scale)
            if (scale - 1) * tan(beta) != 0:
                fh.comp_move_rotate(gear, copy.bodies[0], axis_line, spiral_angle(scale))
            patches.append(copy.bodies[0])
        fh.comp_remove(gear, patch)  # remove the original

        # loft the patches to create the tooth shape and circular pattern it
        tooth = fh.comp_loft(gear, operation, [p.faces[0] for p in patches], donut)
        fh.comp_remove(gear, patches)
        return tooth

    # generate the gear by cutting the donut with the tooth groove shape
    patch = generate_patch(groove_shape)
    donut, donut2, axis_line = base_donut_and_axis()
    fh.app_refresh()

    op = adsk.fusion.FeatureOperations.CutFeatureOperation
    if internal:
        op = adsk.fusion.FeatureOperations.IntersectFeatureOperation
    profile = duplicate_and_loft(patch, axis_line, donut, op)
    fh.app_refresh()

    if not internal:
        # circular pattern the groove cutting
        gear_body = fh.comp_circular_pattern(gear, profile, axis_line, z).bodies[0]
    else:
        fh.app_refresh()
        # circular pattern the teeth and combine them to base donut
        teeth = fh.comp_circular_pattern(gear, profile.bodies[0], axis_line, z).bodies
        gear_body = fh.comp_combine(gear, donut2, teeth, fh.FeatureOperations.join).bodies[0]
        fh.app_refresh()

    if phi != 0:
        if z % 2 == 1:  # adjust phase of the gear
            fh.comp_move_rotate(gear, gear_body, axis_line, pi / z)
        # rotate the gear for meshing the internal gear
        gear_occurrence.transform2 = fh.matrix_rotate(
            phi, fh.vector3d(0, 1, 0), base=gear_occurrence.transform2
        )
        app = adsk.core.Application.get()
        design = adsk.fusion.Design.cast(app.activeProduct)
        design.snapshots.add()  # capture the position

    fh.app_refresh()

    # generate joint between the gear and wrapper
    # This does not work for the internal gear at this moment.
    # Somehow, the axis_line before transformation is referred
    # even after design.snapshots.add() call.
    fh.comp_built_joint_revolute(
        wrapper,
        gear_occurrence,
        wrapper_occurrence,
        axis_line,
        point_type=adsk.fusion.JointKeyPointTypes.EndKeyPoint,
    )

    return gear_occurrence
