import kinematics as kin
import numpy as np
import phx

phx.turn_on()
phx.rest_position()


# calculations
def go_to_pos(pickup_pos, theta0_4):
    joint_angles = kin.ik3(pickup_pos)
    theta4 = kin.calculate_theta_4(joint_angles, theta0_4)
    phx.set_wse(joint_angles)
    phx.set_wrist(theta4)
    phx.wait_for_completion()


def go_to_a():
    point_a = [-20, -10, 7]
    theta0_4 = 0
    go_to_pos(point_a, theta0_4)
    phx.wait_for_completion()


def go_to_b():
    point_b = [10, 0,  20]
    theta0_4 = 0
    go_to_pos(point_b, theta0_4)
    phx.wait_for_completion()












