from math import cos, pi, sin, sqrt, atan2
import math_utils
import measures
import numpy as np
from typing import List, Tuple

# Parameter Optmization

de = 5.37 # cm
kr = 4.15 # cm
ko = 0.12
d_min = 3.48 # cm
delta = 4.57 # cm

def v_robot():
    v_robot = np.array([0, 0])

    return v_robot

def v_obstacle():
    v_obstacle = np.array([0, 0])

    return v_obstacle

def closestObstacle(x: int, y: int, obstacles: List[Tuple[int, int]]) -> Tuple[int, int]:
    '''
    Given an obstacles list, return the obstacle closest to the robot
    '''
    last_ro = 0
    count = 0
    for obstacle in obstacles:
        obs_x, obs_y = obstacle
        delta_x, delta_y = math_utils.delta_axis(x, y, obs_x, obs_y)
        if not count:
            last_ro = math_utils.norm(delta_x, delta_y)
        if (math_utils.norm(delta_x, delta_y) <= last_ro):
            closer_obs = obstacle
            last_ro = math_utils.norm(delta_x, delta_y)
        count += 1
        
    return closer_obs


def Nh(phi: float) -> List[float]:
    return np.array([cos(phi), sin(phi)])


def phiAuf(obs_x: int, obs_y: int, r_x: int, r_y: int, r_o_dist:float, v_obs: list = v_obstacle(), v_rob: list = v_robot(), ko: float = ko) -> float: # Avoid Obstacles
    '''
    Returns an avoidance coefficient, relative to the obstacle, considering the obstacle's position and velocity, 
    as well as the robot's position and velocity
    '''
    obstacle_position = np.array([obs_x, obs_y])

    s_vec = ko * (v_obs - v_rob)
    s_norm = math_utils.norm(s_vec[0], s_vec[1])
    obs_robot_dist = r_o_dist

    if obs_robot_dist >= s_norm:
        p_line_obs = obstacle_position + s_vec
    else:
        p_line_obs = obstacle_position + obs_robot_dist * s_vec / s_norm

    delta_x, delta_y = math_utils.delta_axis(p_line_obs[0], p_line_obs[1], r_x, r_y)
    phi_auf = phiR(delta_x, delta_y)
    
    return math_utils.wrapToPi(phi_auf)


def phiComposed(phi_tuf: float, phi_auf: float, R: float, obstacles: List[Tuple], delta: float = delta, d_min: float = d_min) -> float: # Composition
    '''
    Merges the avoidance and movement coefficients and returns a coefficient of movement, considering the obstacles and robot's position
    '''
    if obstacles is None:
        phi_composed = math_utils.wrapToPi(phi_tuf)
    else:
        gauss = math_utils.gaussian(R - d_min, delta)
        
        if R <= d_min:
            phi_composed = phi_auf
        else:
            # phi_composed = phi_auf * G(R - d_min, delta_const) + phi_tuf * (1 - G(R - d_min, delta_const))
            diff = math_utils.wrapToPi(phi_auf - phi_tuf)
            phi_composed = math_utils.wrapToPi(gauss * diff + phi_tuf)  

    return math_utils.wrapToPi(phi_composed)      


def phiH(ro: float, theta: float, cw: bool = False, radius: float = de, kr: float = kr) -> float: # Hyperbolic
    '''
    Returns a coefficient of a hyperbolic spiral that guides the robot to the ball
    '''
    '''
    The direction of rotation of the spiral has been inverted, cause by passing as in the article, 
    the clockwise direction becomes counterclockwise and vice versa
    '''

    if ro > radius:
        angle = (pi / 2) * (2 - ((radius + kr) / (ro + kr)))
    elif 0 <= ro <= radius:
        angle = (pi / 2) * sqrt(ro / radius)

    if cw:
        return math_utils.wrapToPi(theta + angle)
    else:
        return math_utils.wrapToPi(theta - angle)


def phiR(d_x: float, d_y: float) -> float: # Repulsive
    '''
    Returns an avoidance coefficient, relative to the obstacle, considering nothing but the obstacle's 
    position and the robot's position 
    '''
    return atan2(d_y, d_x)


def phiTuf(theta: float, d_x: float, d_y: float, radius: float = de) -> float: # Move to Goal
    '''
    Merges a clockwise and a counterclockwise hyperbolic spiral and returns a coefficient of 
    movement that guides the robot to the ball, following the smallest path 
    '''
    y_l = d_y + radius
    y_r = d_y - radius

    ro_l = math_utils.norm(d_x, d_y - radius)
    ro_r = math_utils.norm(d_x, d_y + radius)

    phi_ccw = phiH(ro_l, theta, cw=True)
    phi_cw = phiH(ro_r, theta, cw=False)

    nh_ccw = Nh(phi_ccw)
    nh_cw = Nh(phi_cw)
    # The absolute value of y_l and y_r was not specified in the article, but the obtained results 
    # with this trick are closer to the article images
    spiral_merge = (abs(y_l) * nh_ccw + abs(y_r) * nh_cw) / (2 * radius) 

    if -radius <= d_y < radius:
        phi_tuf = atan2(spiral_merge[1], spiral_merge[0])
    elif d_y < -radius:
        phi_tuf = phiH(ro_l, theta, cw=False)
    else:
        phi_tuf = phiH(ro_r, theta, cw=True)

    return math_utils.wrapToPi(phi_tuf)


def generateUnivectorField(r_x: int, r_y: int, ball_pos: Tuple[int, int], obs_pos: List[Tuple[int, int]], de: float = de, v_obs: list = v_obstacle(), v_rob: list = v_robot(), ko: float = ko, delta: float = delta, d_min: float = d_min) -> float:

    ball_x, ball_y = ball_pos
    d_ball_x, d_ball_y = math_utils.delta_axis(ball_x, ball_y, r_x, r_y)
    theta = phiR(d_ball_x, d_ball_y)
    phi_tuf = phiTuf(theta, d_ball_x, d_ball_y, de)

    obstacle = closestObstacle(r_x, r_y, obs_pos)
    obs_x, obs_y = obstacle

    robot_obs_x, robot_obs_y = math_utils.delta_axis(obs_x, obs_y, r_x, r_y)
    R = math_utils.norm(robot_obs_x, robot_obs_y)
    robot_obs_dist = math_utils.norm(robot_obs_x, robot_obs_y)
    
    phi_auf = phiAuf(obs_x, obs_y, r_x, r_y, robot_obs_dist, v_obs, v_rob, ko)
    phi_composed = phiComposed(phi_tuf, phi_auf, R, obstacle, delta, d_min)

    return Nh(phi_composed)

