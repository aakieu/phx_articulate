import kinematics as kin
import numpy as np
import phx
import time

phx.connect()
phx.wake_up()

# wait until motion is complete
while phx.elbow1.is_moving():
    pass

# go to pick up location
p_start = np.array([-13, -20, 13])
p_end = np.array([0, -15, 30])
joint_angles = kin.ik3(p_start)

phx.set_wse(joint_angles)
# wait until motion is complete
while phx.waist.is_moving():
    pass

phx.open_gripper()
# wait until motion is complete
while phx.gripper.is_moving():
    pass

# go to drop off location
joint_angles = kin.ik4(p_start, -90)
phx.set_wrist(joint_angles[3])

phx.close_gripper()
while phx.gripper.is_moving():
    pass

# shut down arm
phx.go_to_sleep()















