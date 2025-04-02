from collections.abc import Callable


def minimize(l: float, r: float, f: Callable[[float], float], tol=1e-6) -> float:
    """
    Find the minimum value of a function f(x) using the golden section search algorithm.

    This function uses the golden section search algorithm to find the minimum value of a function f(x) on the interval [l, r].

    Args:
        l (float): The left endpoint of the interval.
        r (float): The right endpoint of the interval.
        f (function): The function to minimize.
        tol (float): The tolerance for the minimum value.

    Returns:
        float: The x-value that minimizes the function.
    """
    phi = (1 + 5**0.5) / 2
    m1 = r - (r - l) / phi
    m2 = l + (r - l) / phi
    f1 = f(m1)
    f2 = f(m2)
    while abs(l - r) > tol:
        if f1 < f2:
            r = m2
            m2 = m1
            f2 = f1
            m1 = r - (r - l) / phi
            f1 = f(m1)
        else:
            l = m1
            m1 = m2
            f1 = f2
            m2 = l + (r - l) / phi
            f2 = f(m2)
    return (l + r) / 2


def find_root(l: float, r: float, f: Callable[[float], float], tol: float = 1e-6) -> float:
    """
    Find the root of a function f(x) using the bisection method.

    This function uses the bisection method to find the root of a function f(x) on the interval [l, r].

    Args:
        l (float): The left endpoint of the interval.
        r (float): The right endpoint of the interval.
        f (function): The function to find the root of.
        tol (float): The tolerance for the root.

    Returns:
        float: The x-value that is a root of the function.
    """
    fl = f(l)
    fr = f(r)
    if fl * fr > 0:
        return l if abs(fl) < abs(fr) else r

    if fl > fr:
        [l, r] = [r, l]
    while abs(l - r) > tol:
        m = (l + r) / 2
        if f(m) < 0:
            l = m
        else:
            r = m

    return (l + r) / 2
