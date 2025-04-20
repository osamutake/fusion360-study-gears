from collections.abc import Callable
import math
from typing import override
from .lib.fusion_helper import Vector

Translation = tuple[float, float, float, float, float, float]
identical_translation = (1.0, 0.0, 0.0, 0.0, 1.0, 0.0)


class Curve:
    def __init__(self: "Curve"):
        self.points: list[tuple[Vector, float]] = []
        self._translated: list[Vector] | None = None
        self._translation = identical_translation
        self.param2point: Callable[[float], Vector] = lambda _: Vector(0, 0)

    @property
    def translation(self: "Curve"):
        return self._translation

    @translation.setter
    def translation(self, t: Translation):
        self._translated = None
        self._translation = t

    def param2translated(self, param: float) -> Vector:
        p = self.param2point(param)
        return self.apply_translation(p)

    def rotate(self, angle: float, origin: Vector | None = None):
        cos = math.cos(angle)
        sin = math.sin(angle)
        a, b, c, d, e, f = self.translation
        ox, oy = (origin.x, origin.y) if origin else (0, 0)
        self.translation = (
            cos * a - sin * d,
            cos * b - sin * e,
            cos * (c - ox) - sin * (f - oy) + ox,
            sin * a + cos * d,
            sin * b + cos * e,
            sin * (c - ox) + cos * (f - oy) + oy,
        )

    def translate(self, v: Vector):
        a, b, c, d, e, f = self.translation
        self.translation = (a, b, c + v.x, d, e, f + v.y)

    def duplicate(self) -> "Curve":
        c = Curve()
        c.points = self.points[:]
        c.translation = self.translation
        c.param2point = self.param2point
        return c

    def apply_translation(self, p: Vector) -> Vector:
        a, b, c, d, e, f = self.translation
        return Vector(a * p.x + b * p.y + c, d * p.x + e * p.y + f)

    def translated(self) -> list[Vector]:
        if not self._translated:
            self._translated = [self.apply_translation(point) for point, _ in self.points]
        return self._translated


class Segment:
    @property
    def length(self) -> float:
        raise NotImplementedError

    @property
    def start(self) -> Vector:
        raise NotImplementedError

    @property
    def end(self) -> Vector:
        raise NotImplementedError

    def point_at(self, t: float) -> Vector:
        raise NotImplementedError

    def tangent_at(self, t: float) -> Vector:
        raise NotImplementedError

    def translate(self, v: Vector) -> "Segment":
        raise NotImplementedError

    def rotate(self, angle: float, origin: Vector | None = None) -> "Segment":
        raise NotImplementedError


class Line(Segment):
    def __init__(self, start: Vector, end: Vector):
        self._start = start
        self._end = end
        self._tangent = (end - start).normalize()
        self._length = (end - start).norm()

    @property
    @override
    def start(self):
        return self._start

    @property
    @override
    def end(self):
        return self._end

    @property
    @override
    def length(self) -> float:
        return self._length

    @override
    def point_at(self, t: float) -> Vector:
        return self.start + self._tangent * t

    @override
    def tangent_at(self, t: float) -> Vector:
        return self._tangent

    @override
    def translate(self, v: Vector) -> "Line":
        return Line(self.start + v, self.end + v)

    @override
    def rotate(self, angle: float, origin: Vector | None = None) -> "Line":
        return Line(self.start.rotate(angle, origin), self.end.rotate(angle, origin))


class Arc(Segment):
    def __init__(self, center: Vector, radius: float, start_angle: float, end_angle: float):
        self.center = center
        self.radius = radius
        self.start_angle = start_angle
        self.end_angle = end_angle
        self._start = center + Vector.polar(radius, start_angle)
        self._end = center + Vector.polar(radius, end_angle)
        self._length = abs(end_angle - start_angle) * radius

    @property
    @override
    def start(self) -> Vector:
        return self._start

    @property
    @override
    def end(self) -> Vector:
        return self._end

    @property
    @override
    def length(self) -> float:
        return self._length

    @override
    def point_at(self, t: float) -> Vector:
        angle = self.start_angle + (self.end_angle - self.start_angle) * t / self.length
        return self.center + Vector.polar(self.radius, angle)

    @override
    def tangent_at(self, t: float) -> Vector:
        angle = self.start_angle + t / self.radius + math.pi / 2
        return Vector.polar(self.radius, angle)

    @override
    def translate(self, v: Vector) -> "Arc":
        return Arc(self.center + v, self.radius, self.start_angle, self.end_angle)

    @override
    def rotate(self, angle: float, origin: Vector | None = None) -> "Arc":
        return Arc(
            self.center.rotate(angle, origin),
            self.radius,
            self.start_angle + angle,
            self.end_angle + angle,
        )


class Segments(Segment):
    def __init__(self, *segments: Segment):
        self._segments = segments
        self._length = sum(seg.length for seg in segments)

    @property
    @override
    def length(self) -> float:
        return self._length

    @property
    @override
    def start(self) -> Vector:
        return self._segments[0].start

    @property
    @override
    def end(self) -> Vector:
        return self._segments[-1].end

    @override
    def point_at(self, t: float) -> Vector:
        for seg in self._segments:
            if t < seg.length:
                return seg.point_at(t)
            t -= seg.length
        return self.end

    @override
    def tangent_at(self, t: float) -> Vector:
        for seg in self._segments:
            if t < seg.length:
                return seg.tangent_at(t)
            t -= seg.length
        return self._segments[-1].tangent_at(self._segments[-1].length)

    @override
    def translate(self, v: Vector) -> "Segments":
        return Segments(*[seg.translate(v) for seg in self._segments])

    @override
    def rotate(self, angle: float, origin: Vector | None = None) -> "Segments":
        return Segments(*[seg.rotate(angle, origin) for seg in self._segments])

    def to_curve(self, n: int, translation: Callable[[Vector], Vector] | None = None) -> Curve:
        curve = Curve()
        if translation is not None:
            # mypy は translation が後から None になる可能性を考えてエラーを出すが無視する
            curve.param2point = lambda param: translation(self.point_at(param))  # type: ignore
        else:
            curve.param2point = self.point_at
        translation = translation or (lambda v: v)

        delta = self.length / n
        length_acc = 0.0
        for seg in self._segments:
            n_points = math.ceil(seg.length / delta)
            if isinstance(seg, Arc):
                n_points = max(
                    n_points,
                    math.ceil((seg.length / seg.radius / (2 * math.pi)) * 36),
                )
            for j in range(n_points + 1):
                t = seg.length * j / n_points
                curve.points.append((translation(seg.point_at(t)), length_acc + t))
            length_acc += seg.length
        return curve
