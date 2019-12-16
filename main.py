import kinematics as kin
import numpy as np
import phx
import time

phx.connect()
phx.wake_up()

# go to pick up location
p_start = np.array([-13, -20, 13])
p_end = np.array([0, -15, 30])
joint_angles = kin.ik3(p_start)
phx.set_wse(joint_angles)
phx.open_gripper()
time.sleep(3)

# go to drop off location
joint_angles = kin.ik4(p_start, -90)
phx.set_wrist(joint_angles[3])
phx.close_gripper()
time.sleep(3)

# shut down arm
phx.go_to_sleep()








# phx.go_to_sleep()











