from typing import List, Tuple

def getVectors(w: int, h: int, step: int, get_vec, ball: Tuple[int, int], obstacles: List[Tuple[int, int]]=None) -> List[List[float]]:
    
    vectors = []
    for x in range(0, w, step):
        for y in range(0, h, step):
            if obstacles is None:
                vector = get_vec(x, y, ball)
            else:
                vector = get_vec(x, y, ball, obstacles)
            vectors.append(vector)

    return vectors