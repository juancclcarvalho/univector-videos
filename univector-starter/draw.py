import cv2
import measures
import numpy as np
from random import randint
from typing import List, Tuple

def toggleY(x: int, y: int, matrix_h: int) -> Tuple[int, int]:

    return x, matrix_h - y


def drawVector(field: list, origin: Tuple[int, int], end: Tuple[int, int]) -> None:
    cv2.arrowedLine(field, origin, end, (0, 0, 255), 1)


def drawPoint(field: list, pos: Tuple[int, int], h: int) -> None:

    x, y = pos
    cv2.circle(field, toggleY(measures.convertToPixel(x),measures.convertToPixel(y), measures.convertToPixel(h)), 5, (randint(0, 255), randint(0, 255), randint(0, 255)), -1)


def drawVectorField(field: list, vectors: List[List[float]], w: int, h: int, step: int, ball: Tuple[int, int], obstacles: List[Tuple[int, int]]) -> list:

    length = 10
    k = 0
    if ball is not None:
        drawPoint(field, ball, h)
    if obstacles is not None:
        for obstacle in obstacles:
            drawPoint(field, obstacle, h)
    for x in range(0, w, step):
        for y in range(0, h, step):
            origin = toggleY(measures.convertToPixel(x), measures.convertToPixel(y), measures.convertToPixel(h))
            v = [vectors[k][0], vectors[k][1]]
            v[1] = -v[1]
            end = np.array(list(origin)) + length * np.array(v)
            end = (int(end[0]), int(end[1]))
            drawVector(field, origin, end)
            k += 1
            if k >= len(vectors) : return field
                
    return field
