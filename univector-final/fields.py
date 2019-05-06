from copy import copy
import cv2
import draw
import numpy as np
from get_vectors import getVectors
import measures
import math_utils
from typing import List, Tuple
import univector

ball = measures.ball
obstacles = list([(50, 90), (100, 70), (20, 100)])

def avoidObstacles(r_x: int, r_y: int, __, obs_pos: Tuple[int, int]) -> List[float]:
    
    obs_x, obs_y = obs_pos[0]
    robot_obs_x, robot_obs_y = math_utils.delta_axis(obs_x, obs_y, r_x, r_y)
    robot_obs_dist = math_utils.norm(robot_obs_x, robot_obs_y)
    phi_auf = univector.phiAuf(obs_x, obs_y, r_x, r_y, robot_obs_dist)

    return univector.Nh(phi_auf)

def composition(r_x: int, r_y: int, ball_pos: Tuple[int, int], obs_pos: List[Tuple[int, int]]) -> List[float]:

    ball_x, ball_y = ball_pos
    d_ball_x, d_ball_y = math_utils.delta_axis(ball_x, ball_y, r_x, r_y)
    theta = univector.phiR(d_ball_x, d_ball_y)
    phi_tuf = univector.phiTuf(theta, d_ball_x, d_ball_y)

    obstacle = univector.closestObstacle(r_x, r_y, obs_pos)
    obs_x, obs_y = obstacle

    robot_obs_x, robot_obs_y = math_utils.delta_axis(obs_x, obs_y, r_x, r_y)
    R = math_utils.norm(robot_obs_x, robot_obs_y)
    robot_obs_dist = math_utils.norm(robot_obs_x, robot_obs_y)
    
    phi_auf = univector.phiAuf(obs_x, obs_y, r_x, r_y, robot_obs_dist)
    phi_composed = univector.phiComposed(phi_tuf, phi_auf, R, obstacle)

    return univector.Nh(phi_composed)

def hyperbolic(r_x: int, r_y: int, ball_pos: Tuple[int, int]) -> List[float]:

    ball_x, ball_y = ball_pos
    delta_x, delta_y = math_utils.delta_axis(ball_x, ball_y, r_x, r_y)
    theta = univector.phiR(delta_x, delta_y)
    ro = math_utils.norm(delta_x, delta_y)
    phi_h = univector.phiH(ro, theta)

    return univector.Nh(phi_h)

def moveToGoal(r_x: int, r_y: int, ball_pos: Tuple[int, int]) -> List[float]:

    ball_x, ball_y = ball_pos
    delta_x, delta_y = math_utils.delta_axis(ball_x, ball_y, r_x, r_y)
    theta = univector.phiR(delta_x, delta_y)
    phi_tuf = univector.phiTuf(theta, delta_x, delta_y)

    return univector.Nh(phi_tuf)

def repulsive(r_x: int, r_y: int, __, obs_pos: Tuple[int, int]) -> List[float]:

    obs_x, obs_y = obs_pos[0]
    delta_x, delta_y = math_utils.delta_axis(obs_x, obs_y, r_x, r_y)
    phi_r = univector.phiR(delta_x, delta_y)

    return univector.Nh(phi_r)



w, h = measures.getArenaSize()
img_w, img_h = measures.getImageSize()
step = measures.step

field = np.zeros((img_h, img_w, 3))
vectors = getVectors(w, h, step, composition, ball, obstacles)
vectorField = draw.drawVectorField(copy(field), vectors, w, h, step, ball, obstacles)

cv2.imshow('field', vectorField)
cv2.waitKey(0)

cv2.destroyAllWindows()