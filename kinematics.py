import numpy as np
import phx
from rbx_toolkit import rbx_toolkit as rbx


# Link Lengths
a1 = 8.5
a2 = 15
a3 = 15
a4 = 9
link_lengths = np.array([a1, a2, a3, a4])


def htm3(theta):
    """Input: 1x3 angle array  Returns: 4x4 htm matrix"""
    # h0_1
    r0_1 = np.dot(rbx.rot_x(90), rbx.rot_y(theta[0]))
    d0_1 = rbx.transl(0, 0, a1)
    h0_1 = rbx.htm(r0_1, d0_1)
    # h1_2
    r1_2 = rbx.rot_z(theta[1])
    x1_2 = a2*np.cos(np.radians(theta[1]))
    y1_2 = a2*np.sin(np.radians(theta[1]))
    z1_2 = 0
    d1_2 = rbx.transl(x1_2, y1_2, z1_2)
    h1_2 = rbx.htm(r1_2, d1_2)
    # h2_3
    r2_3 = rbx.rot_z(theta[2])
    x2_3 = a3*np.cos(np.radians(theta[2]))
    y2_3 = a3*np.sin(np.radians(theta[2]))
    z2_3 = 0
    d2_3 = rbx.transl(x2_3, y2_3, z2_3)
    h2_3 = rbx.htm(r2_3, d2_3)
    # h0_3
    h0_2 = np.dot(h0_1, h1_2)
    h0_3 = np.dot(h0_2, h2_3)
    return h0_3


def htm4(theta):
    """ Input: 1x4 angle array  Returns: 4x4 htm matrix"""
    h0_3 = htm3(theta)
    # h3_4
    r3_4 = rbx.rot_z(theta[3])
    x3_4 = a4 * np.cos(np.radians(theta[3]))
    y3_4 = a4 * np.sin(np.radians(theta[3]))
    z3_4 = 0
    d3_4 = rbx.transl(x3_4, y3_4, z3_4)
    h3_4 = rbx.htm(r3_4, d3_4)
    h0_4 = np.dot(h0_3, h3_4)
    return h0_4


def fk3(theta):
    """Input: 1x3 angle array  Returns: 1x3 position array """
    h0_3 = htm3(theta)
    x0_3 = h0_3[0, 3]
    y0_3 = h0_3[1, 3]
    z0_3 = h0_3[2, 3]
    d0_3 = [x0_3, y0_3, z0_3]
    return d0_3


def fk4(theta):
    """Input: 1x4 angle array  Returns: 1x3 position array"""
    h0_4 = htm4(theta)
    x0_4 = h0_4[0, 3]
    y0_4 = h0_4[1, 3]
    z0_4 = h0_4[2, 3]
    d0_4 = [x0_4, y0_4, z0_4]
    return d0_4


def ik3(xyz_array):
    # Eqn 1
    theta_1 = np.arctan2(xyz_array[1], xyz_array[0])
    # Eqn 2
    r1 = np.hypot(xyz_array[0], xyz_array[1])
    # Eqn 3
    r2 = xyz_array[2] - link_lengths[0]
    # Eqn 4
    phi2 = np.arctan2(r2, r1)
    # Eqn 5
    r3 = np.hypot(r1, r2)
    # Eqn 6
    num6 = np.power(link_lengths[2], 2) - np.power(link_lengths[1], 2) - np.power(r3, 2)
    den6 = -2 * link_lengths[1] * r3
    phi1 = np.arccos(num6 / den6)
    # Eqn 7
    # theta_2 = phi2 - phi1  # elbow down
    theta_2 = phi2 + phi1
    # Eqn 8
    num8 = np.power(r3, 2) - np.power(link_lengths[1], 2) - np.power(link_lengths[2], 2)
    den8 = -2 * link_lengths[1] * link_lengths[2]
    phi3 = np.arccos(num8 / den8)
    # Eqn 9
    # theta_3 = pi - phi3 # elbow down
    theta_3 = -(np.pi - phi3)
    # Output Joint Angles
    theta_1 = np.rad2deg(theta_1)
    theta_2 = np.rad2deg(theta_2)
    theta_3 = np.rad2deg(theta_3)
    joint_angles = np.array([theta_1, theta_2, theta_3])
    return joint_angles


def calculate_theta_4(joint_angles, theta0_4):
    # R0_3
    theta_1 = joint_angles[0]
    theta_2 = joint_angles[1]
    theta_3 = joint_angles[2]
    # R0_4
    R0_4a = np.dot(rbx.rot_z(theta_1), rbx.rot_x(90))
    R0_4 = np.dot(R0_4a, rbx.rot_z(theta0_4))
    R0_1 = np.dot(rbx.rot_x(90), rbx.rot_y(theta_1))
    R1_2 = rbx.rot_z(theta_2)
    R2_3 = rbx.rot_z(theta_3)
    R0_2 = np.dot(R0_1, R1_2)
    R0_3 = np.dot(R0_2, R2_3)
    # R3_4
    R3_4 = np.dot(np.transpose(R0_3), R0_4)
    # theta_4
    theta_4 = np.degrees(np.arcsin(R3_4[1, 0]))
    return theta_4


def interpolate_line(p_start, p_end, inter_size):
    xyz_matrix = np.zeros([inter_size, 3])  # to store results of interpolated data
    a_vector = p_end - p_start
    t = np.linspace(0, 1, inter_size)
    for index in range(0, inter_size):
        xyz_matrix[index] = p_start + np.multiply(t[index], a_vector)
    return xyz_matrix


def create_joint_matrix(xyz_matrix):
    ik_matrix = np.zeros([xyz_matrix.shape[0], 4])  # to store results of ik3
    for row_num in range(0, xyz_matrix.shape[0]):
        ik_matrix[row_num] = calculate_theta_4(xyz_matrix[row_num], 0)
    return ik_matrix


def line_demo():
    p_start = np.array([-15, -15,  5])
    p_end = np.array([20, 0,  15])

    inter_size = 60
    r_matrix = interpolate_line(p_start, p_end, inter_size)
    print(r_matrix)

    ik_matrix = create_joint_matrix(r_matrix)
    ik_matrix = np.rint(ik_matrix)
    print(ik_matrix)
    phx.set_wsew(ik_matrix[0])
    phx.wait_for_completion()

    for positions in range(0, inter_size):
        phx.set_wsew(ik_matrix[positions])





