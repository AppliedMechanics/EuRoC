# The current measured joint states of the system.

# The measured position in [m] / [rad].
float64[] position
# The measured torque in [N] / [Nm] - Only applies to LWR joints ('lwr_joint_x') and the gripper. The entries of the other joints contain zero values.
float64[] torque
# The measured external torque in [N] / [Nm] - Only applies to the joints of the LWR (lwr_joint_x). The entry of the gripper will be the same as torque. Other joints contain zero values.
float64[] external_torque