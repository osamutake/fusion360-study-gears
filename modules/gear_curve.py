"""Tooth profile generator for spur gears and racks."""

from copy import copy
from collections.abc import Callable
from dataclasses import dataclass
from math import ceil, sin, cos, tan, pi

from .lib.curve import Curve, CurveType, closest_point_x_desc, cross_point_x_desc
from .lib.function import find_root, minimize
from .lib.fusion_helper import Vector, vec


@dataclass
class GearParams:
    m: float
    z: float
    alpha: float
    shift: float
    fillet: float
    mf: float
    mk: float
    rc: float
    backlash: float  # mm
    inner: bool

    def to_list(self, *args):
        return [getattr(self, arg) for arg in args]


def gear_curve(
    params: GearParams,
    tip_fillet: float = 0.0,
):
    params = copy(params)
    if params.inner:
        [params.mk, params.mf] = [params.mf, params.mk]
        params.fillet = 0

    [m, z] = params.to_list("m", "z")
    rp = m * z / 2

    (r1, r2, fillet_center, fillet_radius) = rack_geometry(
        params.m,
        params.alpha,
        params.fillet,
        params.mf,
        params.mk,
        params.rc,
        params.backlash,
        params.z,
        params.shift,
    )

    # shift and rotate
    def shift_rotate(t: float, p: Vector):
        return (p + vec(0, -rp * t)).rotate(t)

    # differential of shift and rotate
    def d_shift_rotate(t: float, p: Vector):
        return p.rotate(t + pi / 2) + vec(rp * (sin(t) + t * cos(t)), rp * (-cos(t) + t * sin(t)))

    # envelope of the straight part of the rack geometry
    # which corresponds to the involute part of the tooth profile
    def rack_trace(t: float):
        p1 = shift_rotate(t, r1)
        # @pylint: disable=invalid-name
        Dp = shift_rotate(t, r2) - p1
        dp1 = d_shift_rotate(t, r1)
        # @pylint: disable=invalid-name
        dDp = d_shift_rotate(t, r2) - dp1
        s = dp1.cross(Dp).z / dDp.cross(Dp).z
        return (p1 - Dp * s, s)

    # envelope of the tip fillet of the rack geometry
    # which corresponds to the root fillet of the tooth profile
    def fillet_trace(t: float):
        p0 = shift_rotate(t, fillet_center)
        # v1 is the tangent vector
        v1 = d_shift_rotate(t, fillet_center).normalize(fillet_radius)
        p2 = p0.add_rotated(-pi / 2, v1)
        p3 = p0.add_rotated(pi / 2, v1)
        return p2 if p2.x - p2.y < p3.x - p3.y else p3

    # addendum shape
    (involute_c, involute_t) = calculate_involute_curve(params, rack_trace, tip_fillet)

    fillet_c = Curve(CurveType.SPLINE)
    fillet_t: list[float] = []
    if not params.inner:
        # dedendum shape
        (fillet_c, fillet_t) = calculate_fillet_curve(
            params, shift_rotate, fillet_trace, fillet_center, fillet_radius, r2
        )

        (involute_c, involute_t, fillet_c, fillet_t) = combine_curves_at_intersection(
            involute_c, involute_t, rack_trace, fillet_c, fillet_t, fillet_trace
        )

    # make the tooth center parallel to the x-axis
    rot = -pi / z / 2
    addendum = lambda t: rack_trace(t)[0].rotate(rot)
    dedendum = lambda t: fillet_trace(t).rotate(rot)
    result: list[
        tuple[Callable[[float], Vector], float, float, int] | tuple[Vector, float, float, float]
    ] = [(addendum, involute_t[0], involute_t[-1], 10)]
    if len(fillet_c) > 1:
        result.append(
            (
                dedendum,
                fillet_t[0],
                fillet_t[-1],  # tangent: Vector.polar(0.1, (fillet_c[-1].angle() + pi / 2))
                7,
            )
        )

    dt = 1 / 10000
    pt = involute_c[0].rotate(rot)  # involute point on the extended tip circle
    if tip_fillet > 0 and pt.y < -dt:
        mk = params.mk * params.m
        tf = tip_fillet * params.m
        shift = params.shift * params.m

        rk = rp + shift + mk  # radius of tip circle
        rt = rk + tf  # radius of extended tip circle
        tk = find_root(  # find the parameter of the involute curve at the tip circle
            involute_t[0], involute_t[-1], lambda t: addendum(t).norm() - rk
        )
        pk = addendum(tk)  # intersection point of the involute curve and the tip circle
        n = (addendum(tk + dt) - pk).rotate(pi / 2)  # normal vector of the involute curve at pk
        n = n * (pk.y / n.y)  # extend n to the x-axis
        c = pk - n  # center of the circle that is tangent to the involute curve at pk
        r_max = rp + mk + tf + shift - c.norm()  # maximum radius of the fillet circle

        # Calculate distance between the involute curve and a circle with radius r that
        # has its center on x-axis and is tangent to the extended tip circle.
        def distance_from_circle_to_curve(r: float):
            c = Vector(rt - r, 0)
            t = minimize(involute_t[0], involute_t[-1], lambda t: (c - addendum(t)).norm())
            r2 = (c - addendum(t)).norm()
            return r2 - r

        def arc(r: float, st: float, et: float, c=vec(0, 0)):
            return (c, st, et, r)

        if distance_from_circle_to_curve(r_max) > 0:
            # Find the radius of the circle that is tangent to the involute curve at pk
            # and the extended tip circle. The circle has the center on the normal vector of
            # the involute curve at pk
            r = find_root(0, n.norm(), lambda r: rt - (pk - n.normalize(r)).norm() - r)
            c = pk - n.normalize(r)  # fillet center
            result[0] = (addendum, tk, involute_t[-1], 10)
            result.insert(0, arc(r, c.angle(), (pk - c).angle(), c))
            result.insert(0, arc(rt, 0, c.angle()))
        else:
            # Find the radius of the circle that is tangent to the involute curve
            # in the region of the extended part of the tooth profile and the
            # extended tip circle. The circle has the center on the x-axis.
            r = find_root(0, r_max, distance_from_circle_to_curve)
            c = Vector(rt - r, 0)
            t0 = minimize(involute_t[0], involute_t[-1], lambda t: (c - addendum(t)).norm())
            result[0] = (addendum, t0, involute_t[-1], 10)
            result.insert(0, arc(r, 0, (addendum(t0) - c).angle(), c))

    return result


# ラック形状を求める
def rack_geometry(
    m: float,
    alpha: float,
    fillet: float,
    mf: float,
    mk: float,
    rc: float,
    backlash: float,
    z: float = 0,  # For cylindrical gears, it offsets the origin.
    shift: float = 0,
):
    fr = fillet * m  # fillet radius
    offset = vec(m * z / 2 + m * shift, backlash / 2)
    slope = vec(1, tan(alpha))

    # linear part
    r1 = offset + (m * mk) * slope  # upper limit of involute part
    r2 = offset - (m * (mf - rc)) * slope  # lower limit of involute part
    r3 = offset - (m * mf) * slope  # bottom of the tooth

    # condition for being tangent to bottom
    fr = min(fr, m * rc / (1 - sin(alpha)))
    # center of the circle that is tangent to the tooth line and to bottom line
    fc = r3 + vec(fr, -fr / tan((pi / 2 + alpha) / 2))
    # if it is beyond the center line
    if fc.y < (-pi * m) / 4:
        # move it on the center line
        fc += (r3 - fc) * (((-pi * m) / 4 - fc.y) / (r3.y - fc.y))
        fr = fc.x - r3.x

    return (
        r1,  # upper left
        r2,  # lower left
        fc,  # fillet center
        fr,  # fillet radius
    )


def calculate_involute_curve(
    params: GearParams,
    rack_trace: Callable[[float], tuple[Vector, float]],
    tip_fillet: float = 0.0,
):
    [alpha, mk, mf, shift, z, m] = params.to_list("alpha", "mk", "mf", "shift", "z", "m")
    rp = (m * z) / 2

    involute_t: list[float] = []
    involute_n = 300
    # parameter for the point on the tip circle
    involute_s = minimize(
        0,
        90 - alpha,
        lambda t: abs(rack_trace(t)[0].norm() - (rp + m * (shift + mk + tip_fillet))),
    )
    # if it is beyond the center line, move it onto the center line
    if rack_trace(involute_s)[0].angle() > pi / z / 2:
        involute_s = minimize(
            0,
            involute_s,
            lambda t: abs(rack_trace(t)[0].angle() - (pi / z / 2)),
        )

    # find the bottom point
    involute_e = minimize(0, -2 * alpha, lambda t: rack_trace(t)[0].norm())
    if rack_trace(involute_e)[0].norm():
        involute_e = find_root(
            0, involute_e, lambda t: rack_trace(t)[0].norm() - (rp + m * (shift - mf))
        )

    # generate the curve
    c_involute = Curve(CurveType.SPLINE)
    for i in range(0, involute_n):
        t = involute_s + ((involute_e - involute_s) * i) / involute_n
        c_involute.points.append(rack_trace(t)[0])
        involute_t.append(t)

    return (c_involute, involute_t)


def calculate_fillet_curve(
    params: GearParams,
    shift_rotate: Callable[[float, Vector], Vector],
    fillet_trace: Callable[[float], Vector],
    center: Vector,
    radius: float,
    r2: Vector,
):
    [mk, shift, alpha, m, z] = params.to_list("mk", "shift", "alpha", "m", "z")
    rp = (m * z) / 2
    rk = rp + m * mk

    fillet_d = 0.05

    # move the rack fillet while its envelop is in the tip circle (r < rk + m * shift)
    fillet_s = 0.0
    while shift_rotate(fillet_s, center).norm() < rk + m * shift:
        fillet_s -= fillet_d
    fillet_e = fillet_d
    while shift_rotate(fillet_e, center).norm() < rk + m * shift:
        fillet_e += fillet_d

    # bottom of the trace
    fillet_m = minimize(fillet_s, fillet_e, lambda t: fillet_trace(t).norm())

    # If bottom of the fillet center is outside the tip reference circle,
    # the direction of parameter sweep should be reversed.
    if shift_rotate(fillet_m, center).norm() > rp:
        [fillet_s, fillet_e] = [fillet_e, fillet_s]

    fillet_e = fillet_m
    fillet_f = False
    last: Vector | None = None
    di = 1.0
    fillet_s2 = fillet_s
    fillet_n = 60
    fillet_t: list[float] = []

    # sweep the fillet circle and find the envelope
    fillet_c = Curve(CurveType.SPLINE)
    i = 0.0
    while i <= fillet_n:
        t = fillet_s + ((fillet_e - fillet_s) * i) / fillet_n
        p = fillet_trace(t)
        if not fillet_f and not (p.norm() > r2.norm() and (p - r2).angle() < alpha):
            # start recording
            fillet_f = True
            fillet_s2 = t  # - ((filletE - filletS) * di) / filletN;
            if last:
                fillet_c.append(last)
                fillet_t.append(t - ((fillet_e - fillet_s) * di) / fillet_n)

        if fillet_f:
            fillet_c.append(p)
            fillet_t.append(t)

        # Process the region around the non-differentiable point
        c = shift_rotate(t, center)
        if i > 0 and (c - shift_rotate(fillet_m, center)).norm() < m / 1000:
            ts = abs((p - shift_rotate(fillet_m, center)).angle())
            te = pi - pi / z / 2
            tn = ceil((abs(te - ts) / pi) * 180)
            for j in range(1, tn + 1):
                tt = ts + ((te - ts) * j) / tn
                fillet_c.points.append(shift_rotate(fillet_m, center) + Vector.polar(radius, tt))
            break

        # if separation is too large, reduce the step.
        if last and (p - last).norm() > m / 20:
            di /= 2
        last = p
        i += di

    fillet_s = fillet_s2
    return (fillet_c, fillet_t)


def combine_curves_at_intersection(
    involute_c: Curve,
    involute_t: list[float],
    rack_trace: Callable[[float], tuple[Vector, float]],
    fillet_c: Curve,
    fillet_t: list[float],
    fillet_trace: Callable[[float], Vector],
):
    (p, i, j) = cross_point_x_desc(involute_c, fillet_c)
    if p:
        involute_c.points[i + 1 :] = [p]
        involute_t[i + 1 :] = [
            minimize(involute_t[i], involute_t[i + 1], lambda t: (rack_trace(t)[0] - p).norm())
        ]
        fillet_c.points[0:j] = [p]
        fillet_t[0:j] = [
            minimize(fillet_t[j], fillet_t[j + 1], lambda t: (fillet_trace(t) - p).norm())
        ]
    else:
        (p, i, j, f) = closest_point_x_desc(involute_c, fillet_c)
        if f:
            # cFillet[j] is the closest point
            involute_c.points[i + 1 :] = [fillet_c[j]]
            fillet_c.points[0 : j - 1] = []
            involute_t[i + 1 :] = [
                minimize(
                    involute_t[i],
                    involute_t[i + 1],
                    lambda t: (rack_trace(t)[0] - fillet_c[j]).norm(),
                )
            ]
            fillet_t[0 : j - 1] = []
        else:
            # cInvolute[i] is the closest point
            involute_c.points[i + 1 :] = []
            fillet_c.points[0:j] = [involute_c[i]]
            involute_t[i + 1 :] = []
            fillet_t[0 : j - 1] = [
                minimize(
                    fillet_t[j],
                    fillet_t[j + 1],
                    lambda t: (fillet_trace(t) - involute_c[i]).norm(),
                )
            ]

    return (involute_c, involute_t, fillet_c, fillet_t)
