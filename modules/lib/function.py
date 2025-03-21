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
    a = l
    b = r
    c = b - (b - a) / phi
    d = a + (b - a) / phi
    while abs(c - d) > tol:
        if f(c) < f(d):
            b = d
        else:
            a = c
        c = b - (b - a) / phi
        d = a + (b - a) / phi
    return (b + a) / 2


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
    a = l
    b = r
    fa = f(a)
    while abs(b - a) > tol:
        c = (a + b) / 2
        fc = f(c)
        if fa * fc < 0:
            b = c
        else:
            a = c
            fa = fc
    return (b + a) / 2
