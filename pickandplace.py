import kinematics as kin
import numpy as np
import phx
import time

phx.connect()
phx.rest_position()

# wait until motion is complete
phx.wait_for_completion()

# go to pick up location
p_start = np.array([-13, -20, 13])
p_end = np.array([0, -15, 30])
joint_angles = kin.ik3(p_start)

phx.set_wse(joint_angles)
# wait until motion is complete
phx.wait_for_completion()

phx.open_gripper()
# wait until motion is complete
phx.wait_for_completion()

# go to drop off location
joint_angles = kin.ik4(p_start, -90)
phx.set_wrist(joint_angles[3])

phx.close_gripper()
phx.wait_for_completion()

# shut down arm
phx.rest_position()
phx.wait_for_completion()
phx.go_to_sleep()















