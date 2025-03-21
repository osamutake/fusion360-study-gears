from __future__ import annotations
from copy import copy
from math import inf
from enum import Enum

from .fusion_helper import Vector


class CurveType(Enum):
    """type of curves"""

    LINE = "line"
    SPLINE = "spline"
    CIRCLE = "circle"
    ARC = "arc"


class Curve:
    """ "
    A class used to represent a Curve.
    Attributes:
        points (List[Vector]): A list of Vector objects representing
        the points of the curve.
    Methods:
        total_length() -> float:
            Calculate the total length of the curve by summing
            the absolute differences between consecutive points.
    """

    def __init__(self, curve_type: CurveType, points: list[Vector] | None = None):
        self.curve_type = curve_type
        self.points = points if points is not None else []

    @classmethod
    def arc(cls, p1: Vector, p2: Vector, p3: Vector):
        return Curve(CurveType.ARC, [p1, p2, p3])

    def total_length(self) -> float:
        """
        Calculate the total length of the curve.

        This method computes the sum of the absolute differences
        between consecutive points in the curve.

        Returns:
            float: The total length of the curve.
        """
        return sum(abs(p1 - p2) for p1, p2 in zip(self.points, self.points[1:]))

    def append(self, *points: Vector):
        """
        Append points to the curve.

        Args:
            *points (List[Vector]): The points to append to the curve.
        """
        self.points.extend(points)

    def __len__(self):
        return len(self.points)

    def __getitem__(self, index: int):
        return self.points[index]

    def __setitem__(self, index: int, value: Vector):
        self.points[index] = value

    def __iter__(self):
        return iter(self.points)

    def __reversed__(self):
        return reversed(self.points)

    def __copy__(self):
        return Curve(self.curve_type, self.points.copy())

    def __deepcopy__(self, memo):
        return Curve(self.curve_type, self.points.copy())

    def __format__(self, format_spec):
        return f"{self.curve_type.value}: {self.points}"

    def __str__(self):
        return f"{self.curve_type.value}: {self.points}"

    def __repr__(self):
        return f"{self.curve_type.value}: {self.points}"

    def __eq__(self, other: object):
        return (
            isinstance(other, Curve)
            and self.curve_type == other.curve_type
            and self.points == other.points
        )

    def __hash__(self):
        return hash((self.curve_type, tuple(self.points)))

    def __ne__(self, other: object):
        return not self == other

    # 均等に nPoints 個の点を取り出して新しいカーブとして返す
    def extract_points(self, n_points: int):
        if len(self) <= n_points:
            return copy(self)
        total = self.total_length()
        c2 = Curve(self.curve_type)
        nxt = 0.0
        ln = 0.0
        for i in range(len(self) - 1):
            if ln >= nxt:
                c2.append(self[i])
                nxt = (total * len(c2)) / n_points
            ln += abs(self[i] - self[i + 1])
        c2.append(self[-1])
        return c2


# ２つの線分の交点を求める
def cross_point(p1: Vector, p2: Vector, q1: Vector, q2: Vector):
    a = p2 - p1
    b = q2 - q1
    c = q1 - p1
    d = a.cross(b).z
    if d == 0:
        return
    s = c.cross(a).z / d
    t = c.cross(b).z / d
    if s < 0 or s > 1 or t < 0 or t > 1:
        return
    return p1 - a * t


# X が降順に並ぶ２つのカーブの交点を求める
def cross_point_x_desc(c1: Curve, c2: Curve):
    i = 0
    j = 0
    while i < len(c1) - 1 and j < len(c2) - 1:
        if c1[i + 1].x > c2[j].x:
            i += 1
            continue

        if c1[i].x < c2[j + 1].x:
            j += 1
            continue

        p = cross_point(c1[i], c1[i + 1], c2[j], c2[j + 1])
        if p:
            return (p, i, j)
        if c1[i + 1].x > c2[j + 1].x:
            i += 1
        else:
            j += 1
    return (None, -1, -1)


# q1, q2 を通る直線から p までの距離
def distance_from_line(p: Vector, q1: Vector, q2: Vector):
    a = q2 - q1
    b = p - q1
    return abs(a.cross(b).z / a.norm())


def closest_point_x_desc(c1: Curve, c2: Curve):
    i = 0
    j = 0
    minimum = (inf, -1, -1, False)
    while i < len(c1) - 1 and j < len(c2) - 1:
        if c1[i + 1].x > c2[j].x:
            i += 1
            continue

        if c1[i].x < c2[j + 1].x:
            j += 1
            continue

        if c1[i].x < c2[j].x:
            d = distance_from_line(c1[i], c2[j], c2[j + 1])
            if d < minimum[0]:
                minimum = (d, i, j, False)
        else:
            d = distance_from_line(c2[j], c1[i], c1[i + 1])
            if d < minimum[0]:
                minimum = (d, i, j, True)

        if c1[i + 1].x > c2[j + 1].x:
            i += 1
        else:
            j += 1

    if minimum[1] == -1:
        if i < len(c1) - 1:
            d = distance_from_line(c2[j], c1[i], c1[i + 1])
            return (d, i, j, True)
        else:
            d = distance_from_line(c1[i], c2[j], c2[j + 1])
            return (d, i, j, False)

    return minimum
