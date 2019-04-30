from math import pi, sqrt, exp
from typing import Tuple

def delta_axis(x_1: int, y_1: int, x_2: int, y_2: int) -> Tuple[int, int]:
    return x_2 - x_1, y_2 - y_1


def gaussian(r: float, d_const: float) -> float:
    return exp(-(r ** 2 / (2 * d_const ** 2)))


def norm(d_x: int, d_y: int) -> float:
    return sqrt(d_x ** 2 + d_y ** 2)


def wrapToPi(angle: float) -> float:
    
    if angle > pi:
        return angle - 2 * pi
    if angle < -pi:
        return 2 * pi + angle
    else:
        return angle

