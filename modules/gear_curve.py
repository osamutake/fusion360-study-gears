"""歯車の歯形曲線を描く"""

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
):
    params = copy(params)
    if params.inner:
        [params.mk, params.mf] = [params.mf, params.mk]
        params.fillet = 0

    params.fillet *= params.m

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
    (involute_c, involute_t) = calculate_involute_curve(params, rack_trace)

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
    involute_c.points = [v.rotate(rot) for v in involute_c]
    fillet_c.points = [v.rotate(rot) for v in fillet_c]

    # 全周の歯形を生成
    dt = 1 / 1000
    result = [
        (
            involute_c.extract_points(10),
            (rack_trace(involute_t[0] + dt)[0] - rack_trace(involute_t[0])[0])
            .rotate(rot)
            .normalize(0.1),
            (rack_trace(involute_t[-1] - dt)[0] - rack_trace(involute_t[-1])[0])
            .rotate(rot)
            .normalize(0.1),
        )
    ]
    if len(fillet_c) > 1:
        result.append(
            (
                fillet_c.extract_points(6),
                (fillet_trace(fillet_t[0] + dt) - fillet_trace(fillet_t[0]))
                .rotate(rot)
                .normalize(0.1),
                Vector.polar(0.1, (fillet_c[-1].angle() + pi / 2)),
            )
        )
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
    z: float = 0,  # そのままラックとして使うならゼロにする
    shift: float = 0,
):
    fr = fillet * m  # フィレット半径
    offset = vec(m * z / 2 + m * shift, backlash / 2)
    slope = vec(1, tan(alpha))

    # 斜め線部分
    r1 = offset + (m * mk) * slope  # インボリュート上限
    r2 = offset - (m * (mf - rc)) * slope  # インボリュート下限
    r3 = offset - (m * mf) * slope  # 歯先

    # 底辺に接するための条件
    fr = min(fr, m * rc / (1 - sin(alpha)))
    # r3 を基準に２辺に接する円の中心点を求める
    fc = r3 + vec(fr, -fr / tan((pi / 2 + alpha) / 2))
    # 中心線を超えてしまったら
    if fc.y < (-pi * m) / 4:
        # r3 方向に動かして中心線に合わせる
        fc += (r3 - fc) * (((-pi * m) / 4 - fc.y) / (r3.y - fc.y))
        fr = fc.x - r3.x

    return (
        r1,  # 左上
        r2,  # 左下
        fc,  # フィレットの中心
        fr,  # フィレット半径
    )


# 歯末の形状を求める
def calculate_involute_curve(
    params: GearParams,
    rack_trace: Callable[[float], tuple[Vector, float]],
):
    [alpha, mk, mf, shift, z, m] = params.to_list("alpha", "mk", "mf", "shift", "z", "m")
    rp = (m * z) / 2

    # 歯末のインボリュート曲線
    involute_t: list[float] = []
    involute_n = 300
    # 歯先に触れる点
    involute_s = minimize(
        0, 90 - alpha, lambda t: abs(rack_trace(t)[0].norm() - (rp + m * (shift + mk)))
    )
    # 中心線を超えたら戻す
    if rack_trace(involute_s)[0].angle() > pi / z / 2:
        involute_s = minimize(
            0,
            involute_s,
            lambda t: abs(rack_trace(t)[0].angle() - (pi / z / 2)),
        )

    # 最も中心に近づく点
    involute_e = minimize(0, -2 * alpha, lambda t: rack_trace(t)[0].norm())
    if rack_trace(involute_e)[0].norm():
        involute_e = find_root(
            0, involute_e, lambda t: rack_trace(t)[0].norm() - (rp + m * (shift - mf))
        )

    # 曲線を生成
    c_involute = Curve(CurveType.SPLINE)
    for i in range(0, involute_n):
        t = involute_s + ((involute_e - involute_s) * i) / involute_n
        c_involute.points.append(rack_trace(t)[0])
        involute_t.append(t)

    return (c_involute, involute_t)


# 歯元の形状を求める
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

    # 半径 rk + shift 内でのラック頂点の移動範囲を求める
    fillet_s = 0.0
    while shift_rotate(fillet_s, center).norm() < rk + m * shift:
        fillet_s -= fillet_d
    fillet_e = fillet_d
    while shift_rotate(fillet_e, center).norm() < rk + m * shift:
        fillet_e += fillet_d

    # 歯先軌跡の先端
    fillet_m = minimize(fillet_s, fillet_e, lambda t: fillet_trace(t).norm())

    # 見るべき方向が入れ替わる
    if shift_rotate(fillet_m, center).norm() > rp:
        [fillet_s, fillet_e] = [fillet_e, fillet_s]

    fillet_e = fillet_m
    fillet_f = False
    last: Vector | None = None
    di = 1.0
    fillet_s2 = fillet_s
    fillet_n = 60
    fillet_t: list[float] = []

    # 歯元曲線
    fillet_c = Curve(CurveType.SPLINE)
    i = 0.0
    while i <= fillet_n:
        t = fillet_s + ((fillet_e - fillet_s) * i) / fillet_n
        p = fillet_trace(t)
        if not fillet_f and not (p.norm() > r2.norm() and (p - r2).angle() < alpha):
            # 記録開始
            fillet_f = True
            fillet_s2 = t  # - ((filletE - filletS) * di) / filletN;
            if last:
                fillet_c.append(last)
                fillet_t.append(t - ((fillet_e - fillet_s) * di) / fillet_n)

        if fillet_f:
            fillet_c.append(p)
            fillet_t.append(t)

        # 微分不能点近くの処理
        c = shift_rotate(t, center)
        if i > 0 and (c - shift_rotate(fillet_m, center)).norm() < m / 1000:
            ts = abs((p - shift_rotate(fillet_m, center)).angle())
            te = pi - pi / z / 2
            tn = ceil((abs(te - ts) / pi) * 180)
            for j in range(1, tn + 1):
                tt = ts + ((te - ts) * j) / tn
                fillet_c.points.append(shift_rotate(fillet_m, center) + Vector.polar(radius, tt))
            break

        # 間が空きすぎたら更新ステップを小さくする
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
