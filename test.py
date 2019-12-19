import kinematics as kin
import numpy as np
import phx
import time

phx.connect()
phx.rest_position()

# wait until motion is complete
phx.wait_for_completion()

# calculations
pos_one = np.array([-20, -20, 15])
pos_two = np.array([-20, -20, 14])
desired_gripper_angle = -90

joint_angles_one = kin.ik4(pos_one, desired_gripper_angle)
wrist_angle_one = joint_angles_one[3]

joint_angles_two = kin.ik4(pos_two, desired_gripper_angle)
wrist_angle_two = joint_angles_two[3]

# go to position one
phx.set_wse(joint_angles_one[0:3])
# wait until motion is complete
phx.wait_for_completion()

phx.set_wrist(wrist_angle_one)
phx.wait_for_completion()

# go to position two
phx.set_wsew(joint_angles_two)
phx.wait_for_completion()



phx.set_gripper(200)
phx.wait_for_completion()

drop_off_pos = np.array([20, 10, 20])
joint_angles_c = kin.ik3(drop_off_pos)

phx.set_wse(joint_angles_c)
phx.wait_for_completion()

phx.open_gripper()



