# Import packages first
import numpy as np
np.set_printoptions(suppress=True)


def rot_x(theta_deg):
    """ returns rotation matrix """
    theta_rad = np.radians(theta_deg)
    rotation_matrix = [[1, 0, 0],
                       [0, np.cos(theta_rad), -np.sin(theta_rad)],
                       [0, np.sin(theta_rad), np.cos(theta_rad)]]
    return np.matrix(rotation_matrix)


def rot_y(theta_deg):
    """ returns rotation matrix """
    theta_rad = np.radians(theta_deg)
    rotation_matrix = [[np.cos(theta_rad), 0, np.sin(theta_rad)],
                       [0, 1, 0],
                       [-np.sin(theta_rad), 0, np.cos(theta_rad)]]
    return np.matrix(rotation_matrix)


def rot_z(theta_deg):
    """ returns rotation matrix """
    theta_rad = np.radians(theta_deg)
    rotation_matrix = [[np.cos(theta_rad), -np.sin(theta_rad), 0],
                       [np.sin(theta_rad), np.cos(theta_rad), 0],
                       [0, 0, 1]]
    return np.matrix(rotation_matrix)


def transl(x, y, z):
    """ returns rotation matrix """
    displace_vector = [[x],
                       [y],
                       [z]]
    return np.matrix(displace_vector)


def htm(rot_matrix, d_vector):
    """ Returns htm matrix"""
    htm_matrix = np.append(rot_matrix, d_vector, axis=1)
    htm_matrix = np.append(htm_matrix, [[0, 0, 0, 1]], axis=0)
    return htm_matrix


def map_value(x_in, x_min, x_max, y_min, y_max):
    """ returns motor position value between 0-1023 """
    m = ((y_max - y_min) / (x_max - x_min))
    y_out = m * (x_in - x_min) + y_min
    return y_out
