"""
Akima spline interpolation.
interpolation(xs: list[float], ys: list[float], periodic: bool = False) -> Callable[[float], float]
"""
# https://en.wikipedia.org/wiki/Akima_spline
# https://github.com/Juul/akima-interpolator/tree/master

from .fusion_helper import Vector


def interpolate(xs: list[float], ys: list[float], periodic: bool = False):
    if len(xs) != len(ys):
        raise ValueError("xs and ys must have the same length.")
    if periodic and ys[0] != ys[-1]:
        raise ValueError("Periodic ys is required.")

    xs = xs.copy()  # clone
    ys = ys.copy()  # clone

    # 同じ値があれば削除する
    for i in reversed(range(1, len(xs))):
        if xs[i] == xs[i - 1]:
            xs.pop(i)
            ys.pop(i)

    if len(xs) < 5:
        # 線形補間する
        def linear(x: float):
            i = find_segment(x, xs)
            i = max(0, min(i, len(coefs)))
            return ys[i - 1] + ((ys[i] - ys[i - 1]) * (x - xs[i - 1])) / (xs[i] - xs[i - 1])

        return linear

    slopes = calc_slopes(xs, ys, periodic)
    coefs = calc_coefs(xs, ys, slopes)

    def spline(x: float):
        i = find_segment(x, xs)
        i = max(0, min(i, len(coefs) - 1))
        return polynomial(x - xs[i], coefs[i])

    return spline


def find_segment(x: float, xs: list[float]):
    l, r = 0, len(xs) - 1
    while l <= r:
        m = (l + r) // 2
        mx = xs[m]
        if mx < x:
            l = m + 1
        elif mx > x:
            r = m - 1
        elif mx == x:
            return m
        else:
            raise ValueError("NaN found in xs.")
    return l - 1


def polynomial(x: float, coefs: list[float]):
    return sum(coef * (x**i) for i, coef in enumerate(reversed(coefs)))


def calc_slopes(xs: list[float], ys: list[float], periodic: bool) -> list[float]:
    n = len(xs)
    dydx = [(ys[i + 1] - ys[i]) / (xs[i + 1] - xs[i]) for i in range(n - 1)]
    if periodic:
        weights = [abs(dydx[i % (n - 1)] - dydx[i - 1]) for i in range(1, n)]
        result = []
        for i in range(n):
            i0 = i % (n - 1)
            im1 = (i - 1 + (n - 1)) % (n - 1)
            ip1 = (i + 1) % (n - 1)
            wp1, wm1 = weights[ip1], weights[im1]
            if abs(wp1) < float("1e-10") and abs(wm1) < float("1e-10"):
                dx, dxm1 = xs[ip1] - xs[i0], xs[i0] - xs[im1]
                result.append((dx * dydx[im1] + dxm1 * dydx[i0]) / (dx + dxm1))
            else:
                result.append((wp1 * dydx[im1] + wm1 * dydx[i0]) / (wp1 + wm1))
    else:
        weights = [abs(dydx[i] - dydx[i - 1]) for i in range(1, n - 1)]
        result = [
            slope_from_3points(xs, ys, 0, 0, 1, 2),
            slope_from_3points(xs, ys, 1, 0, 1, 2),
        ]
        for i in range(2, n - 2):
            wp1, wm1 = weights[i], weights[i - 1]
            if abs(wp1) < float("1e-10") and abs(wm1) < float("1e-10"):
                dx, dxm1 = xs[i + 1] - xs[i], xs[i] - xs[i - 1]
                result.append((dx * dydx[i - 1] + dxm1 * dydx[i]) / (dx + dxm1))
            else:
                result.append((wp1 * dydx[i - 1] + wm1 * dydx[i]) / (wp1 + wm1))
        result.extend(
            [
                slope_from_3points(xs, ys, n - 2, n - 3, n - 2, n - 1),
                slope_from_3points(xs, ys, n - 1, n - 3, n - 2, n - 1),
            ]
        )
    return result


def slope_from_3points(
    xs: list[float], ys: list[float], i: int, i1: int, i2: int, i3: int
) -> float:
    dx, dx2, dx3 = xs[i] - xs[i1], xs[i2] - xs[i1], xs[i3] - xs[i1]
    dydx2, dydx3 = (ys[i2] - ys[i1]) / dx2, (ys[i3] - ys[i1]) / dx3
    return ((dydx3 - dydx2) * (2 * dx - dx2)) / (dx3 - dx2) + dydx2


def calc_coefs(xs: list[float], ys: list[float], slopes: list[float]) -> list[list[float]]:
    n = len(xs)
    coefs = []
    for i in range(n - 1):
        dx = xs[i + 1] - xs[i]
        dydx = (ys[i + 1] - ys[i]) / dx
        dfi, dfi1 = slopes[i], slopes[i + 1]
        coefs.append(
            [
                (-2 * dydx + dfi + dfi1) / (dx * dx),
                (3 * dydx - 2 * dfi - dfi1) / dx,
                dfi,
                ys[i],
            ]
        )
    return coefs


def evenly_spaced_points_on_spline(points: list[Vector], n: int):
    """Calc evenly spaced control points along the spline curve"""

    # パラメータを 0 から 1 としてスプライン曲線を作成
    ts = [0.0]
    for i in range(1, len(points)):
        ts.append(ts[-1] + abs(points[i] - points[i - 1]))
    total = ts[-1]
    ts = [t / total for t in ts]  # 0 to 1
    cx = interpolate(ts, [r.x for r in points])
    cy = interpolate(ts, [r.y for r in points])

    # n * 10 個の点を作って距離を求める
    m = n * 10
    lengths = [0.0]
    last = points[0]
    for i in range(1, m):
        t = i / (m - 1)
        curr = Vector(cx(t), cy(t))
        lengths.append(lengths[-1] + abs(curr - last))
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
