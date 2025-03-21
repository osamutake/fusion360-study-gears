from collections.abc import Callable
from copy import copy
from dataclasses import dataclass
from functools import reduce
from math import pi, cos, asin, atan, tan, floor

import adsk.core, adsk.fusion

from .lib.function import minimize, find_root

from . import gear_curve
from .lib import fusion_helper as fh
from .lib.fusion_helper import Vector, vec, radius_from_3points
from .lib.segment import Line, Arc, Segments
from .lib.spline import interpolate


def gear_worm_wheel(
    parent: adsk.fusion.Component,
    params: gear_curve.GearParams,
    worm_diameter: float,
    worm_spirals: int,
    thickness: float,
    helix_angle: float,
):
    occurrence = parent.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    occurrence.isGroundToParent = False
    comp = occurrence.component

    sketch = comp.sketches.add(comp.xYConstructionPlane)
    sketch.isComputeDeferred = True

    # 高さごとの歯溝形状を生成する
    n = 9
    for i in range(n):
        z = i / (n - 1) * thickness * 1.01 / 2
        shape = worm_wheel_shape(params, helix_angle, z, worm_diameter / 2, worm_spirals)
        spline = sketch.sketchCurves.sketchFittedSplines.add(
            fh.collection([fh.point3d(v.x, v.y, z) for v in shape])
        )
        sketch.sketchCurves.sketchArcs.addByCenterStartEnd(
            fh.point3d(0, 0, z),
            spline.endSketchPoint,
            spline.startSketchPoint,
        )
    for c in sketch.sketchCurves:
        c.isFixed = True
    for p in sketch.sketchPoints:
        p.isFixed = True

    # パッチに変換
    # プロファイルから作ろうとすると取りこぼすことがあるようなのでカーブから作る
    curves = sorted(sketch.sketchCurves, key=lambda c: c.boundingBox.minPoint.z)
    patches = [
        fh.comp_patch(comp, curves[i * 2 : (i + 1) * 2], fh.FeatureOperations.new_body)
        for i in range(1, floor(len(curves) / 2))
    ]
    sketch.isVisible = False

    # 反対側へコピーする
    for i, patch in enumerate(patches.copy()):
        if patch.bodies[0].boundingBox.minPoint.z > 0:
            patch2 = fh.comp_mirror(comp, patch.bodies[0], comp.xZConstructionPlane)
            fh.comp_move_free(
                comp,
                patch2.bodies[0],
                fh.matrix_translate(z=-patch.bodies[0].boundingBox.minPoint.z * 2),
            )
            patches.insert(0, patch2)

    # 歯先円を押し出す
    sketch2 = comp.sketches.add(comp.xYConstructionPlane)
    sketch2.sketchCurves.sketchCircles.addByCenterRadius(
        fh.point3d(),
        params.m / cos(helix_angle) * params.z / 2 + params.m * params.mk,
    )
    disk = fh.comp_extrude(
        comp,
        sketch2.profiles[0],
        fh.FeatureOperations.new_body,
        thickness,
        True,
        True,
    )

    # 歯溝を切り込む
    teeth = fh.comp_loft(
        comp,
        fh.FeatureOperations.cut,
        [p.faces[0] for p in patches],
        disk.bodies[0],
    )
    for patch in patches:
        fh.comp_remove(comp, patch.bodies)

    # 全周に渡って複製する
    fh.comp_circular_pattern(comp, teeth, comp.zConstructionAxis, round(params.z))

    # 歯車の回転軸を描画する
    sketch3 = comp.sketches.add(comp.xYConstructionPlane)
    center_axis = sketch3.sketchCurves.sketchLines.addByTwoPoints(
        fh.point3d(z=-thickness / 2), fh.point3d(z=thickness / 2)
    )
    center_axis.isFixed = True

    # 基準円を描画する
    rp = params.m / cos(helix_angle) * params.z / 2
    circle = sketch3.sketchCurves.sketchCircles.addByCenterRadius(fh.point3d(), rp)
    circle.isConstruction = True
    sketch3.sketchDimensions.addDiameterDimension(circle, fh.point3d(rp, rp), isDriving=True)
    for p in sketch3.sketchPoints:
        p.isFixed = True


def worm_wheel_shape(
    params: gear_curve.GearParams, beta: float, thick: float, rw: float, wn: int
) -> list[Vector]:
    cos_beta = cos(beta)
    params = copy(params)
    params.m /= cos(beta)
    params.alpha = atan(tan(params.alpha) / cos_beta)
    params.mf *= cos_beta
    params.mk *= cos_beta

    rp = params.m * params.z / 2  # pitch radius
    rk = params.m * (params.z / 2 + params.mk)  # addendum circle radius
    eps = params.m / 1000

    # ラック形状を求めて対称性の良い位置へ移動
    def generate_rack_geometry() -> Segments:
        (r1, _, center, radius) = gear_curve.rack_geometry(
            params.m,
            params.alpha,
            params.fillet,
            params.mf,
            1.1 * params.mk,
            params.rc,
            params.backlash,
            params.z,
            params.shift,
        )
        r1 += vec(0, (pi * params.m) / 4)
        center += vec(0, (pi * params.m) / 4)
        r3 = center + Vector.polar(radius, params.alpha + pi / 2)
        r4 = center + Vector.polar(radius, pi)
        if abs(center.y) < eps:
            # 歯先に中心が来るときは１つの円弧で表現
            return Segments(
                Line(r1, r3),
                Arc(
                    center,
                    radius,
                    params.alpha + pi / 2,
                    2 * pi - (params.alpha + pi / 2),
                ),
                Line(r3.flip_y(), r1.flip_y()),
            )

        return Segments(
            # 歯先に中心が来ないときは２つの円弧と直線
            Line(r1, r3),
            Arc(center, radius, params.alpha + pi / 2, pi),
            Line(r4, r4.flip_y()),
            Arc(center.flip_y(), radius, pi, 2 * pi - (params.alpha + pi / 2)),
            Line(r3.flip_y(), r1.flip_y()),
        )

    rack_geometry = generate_rack_geometry()

    # 厚さ方向変位に応じた変形を加えた数値曲線に変換
    def calc_worm_curve(n_points: int):
        worm_axis_x = (params.z / 2 + params.shift) * params.m + rw
        trans_coef = pi * params.m * wn / (2 * pi)
        if beta < 0:
            trans_coef *= -1

        def translate(v: Vector):
            t = -asin(thick / (v.x - worm_axis_x))
            return vec(
                worm_axis_x + (v.x - worm_axis_x) * cos(t),
                v.y + trans_coef * t,
            )

        return rack_geometry.to_curve(n_points, translate)

    worm_curve = calc_worm_curve(100)

    @dataclass
    class TranslatedCurve:
        t: float
        curve: Segments
        spline: Callable[[float], Vector]
        length: float
        closest: float
        closest_p: Vector

    # ウォームホイールの回転によるラック形状の移動を計算
    def generate_translated_curves() -> list[TranslatedCurve]:

        # ウォームホイールの回転によるラック形状の移動を計算
        def calc_translated(t: float):
            c = worm_curve.duplicate()
            c.translate(vec(0, t * rp))
            c.rotate(-t)

            translated = c.translated()
            ts = [cp[1] for cp in c.points]
            xs = [cp.x for cp in translated]
            ys = [cp.y for cp in translated]
            spline_x = interpolate(ts, xs)
            spline_y = interpolate(ts, ys)
            spline = lambda t: vec(spline_x(t), spline_y(t))

            length = c.points[-1][1]
            closest = minimize(0, length, lambda t: spline(t).norm(), eps)
            closest_p = spline(closest)
            return TranslatedCurve(t, c, spline, length, closest, closest_p)

        # 移動した曲線を求める
        curves: list[TranslatedCurve] = []
        dt = pi / params.z / 2
        curves.append(calc_translated(0))
        curves.append(calc_translated(dt))

        while curves[0].closest_p.norm() < curves[1].closest_p.norm():
            curves.insert(0, calc_translated(curves[0].t - dt))
        while curves[0].closest_p.norm() < rk:
            curves.insert(0, calc_translated(curves[0].t - dt))
        while curves[-1].closest_p.norm() < curves[-2].closest_p.norm():
            curves.append(calc_translated(curves[-1].t + dt))
        while curves[-1].closest_p.norm() < rk:
            curves.append(calc_translated(curves[-1].t + dt))

        # 曲率半径と比べて離れすぎているところに点を追加する
        i = 1
        while i < len(curves) - 1:
            p1 = curves[i - 1].spline(curves[i - 1].length / 2)
            p2 = curves[i].spline(curves[i].length / 2)
            p3 = curves[i + 1].spline(curves[i + 1].length / 2)
            v1 = p2 - p1
            v2 = p3 - p2
            r = radius_from_3points(p1, p2, p3)
            if v1.norm() > params.m / 1000 and v1.norm() > r / 10:
                c1 = calc_translated((curves[i].t + curves[i - 1].t) / 2)
                curves.insert(i, c1)
                continue  # no increment
            elif v2.norm() > params.m / 1000 and v2.norm() > r / 10:
                c2 = calc_translated((curves[i].t + curves[i + 1].t) / 2)
                curves.insert(i + 1, c2)
                continue  # no increment
            i += 1

        return curves

    curves = generate_translated_curves()

    # 歯底円の半径を求める
    rf = reduce(lambda rf, c: min(rf, c.closest_p.norm()), curves, rk)

    # 半径ごとにウォーム歯形との交点を求める
    def find_intersections(initial_division: int):
        @dataclass
        class Intersection:
            r: float
            a1: float
            a2: float

            def values(self):
                return (self.r, self.a1, self.a2)

        # 与えられた半径におけるウォーム歯形との交点を求める
        def intersect(r: float):
            def red_max(
                a: float,
                c: TranslatedCurve,
            ):
                if c.closest_p.norm() > r:
                    return a
                t = find_root(0, c.closest, lambda t: c.spline(t).norm() - r, eps)
                return max(a, c.spline(t).angle())

            def red_min(
                a: float,
                c: TranslatedCurve,
            ):
                if c.closest_p.norm() > r:
                    return a
                t = find_root(c.closest, c.length, lambda t: c.spline(t).norm() - r, eps)
                return min(a, c.spline(t).angle())

            # 上側の交点の角度
            a1 = reduce(red_max, curves, -float("inf"))
            # 下側の交点の角度
            a2 = reduce(red_min, curves, float("inf"))
            return Intersection(r, a1, a2)

        # まずは initial_division 個に分けて計算
        intersections: list[Intersection] = []
        for i in range(1, initial_division + 2):
            r = rf + ((rk - rf) * i) / initial_division
            intersections.append(intersect(r))

        # 歯底に近い部分を細かく分ける
        intersections.insert(0, intersect(rf + (rk - rf) / (initial_division * 2)))
        intersections.insert(0, intersect(rf + (rk - rf) / (initial_division * 8)))
        intersections.insert(0, intersect(rf + (rk - rf) / (initial_division * 128)))

        # 曲率半径に比べて点が離れすぎているところに点を追加する
        i = 1
        while i < len(intersections) - 1:
            r1, a1, b1 = intersections[i - 1].values()
            r2, a2, b2 = intersections[i].values()
            r3, a3, b3 = intersections[i + 1].values()

            # 上側をチェック
            p1 = Vector.polar(r1, a1)
            p2 = Vector.polar(r2, a2)
            p3 = Vector.polar(r3, a3)
            v1 = p2 - p1
            v2 = p3 - p2
            rr1 = radius_from_3points(p1, p2, p3)
            m = 40
            mm = 5
            if v1.norm() > params.m / m and v1.norm() > rr1 / mm:
                intersections.insert(i, intersect((r1 + r2) / 2))
                continue  # no increment
            if v2.norm() > params.m / m and v2.norm() > rr1 / mm:
                intersections.insert(i + 1, intersect((r2 + r3) / 2))
                continue  # no increment

            # 下側をチェック
            q1 = Vector.polar(r1, b1)
            q2 = Vector.polar(r2, b2)
            q3 = Vector.polar(r3, b3)
            w1 = q2 - q1
            w2 = q3 - q2
            rr2 = radius_from_3points(q1, q2, q3)
            if w1.norm() > params.m / m and w1.norm() > rr2 / mm:
                intersections.insert(i, intersect((r1 + r2) / 2))
                continue  # no increment
            if w2.norm() > params.m / m and w2.norm() > rr2 / mm:
                intersections.insert(i + 1, intersect((r2 + r3) / 2))
                continue  # no increment

            i += 1

        return intersections

    intersections = find_intersections(16)

    result = [Vector.polar(p.r, p.a1) for p in reversed(intersections)]
    result += [Vector.polar(rf, (intersections[0].a1 + intersections[0].a2) / 2)]
    result += [Vector.polar(p.r, p.a2) for p in intersections]

    return result
