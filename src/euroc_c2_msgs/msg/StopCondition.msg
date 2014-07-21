# Describes one stop condition for the movement of the robot. If the condition evaluates to true the robot will stop its movement and return the stop reason.

# The type of condition. Can be: 'joint_ext_torque_threshold' or 'tool_force_threshold'.
# - joint_ext_torque_threshold: The torque applied to a specified joint from external sources exceeds a theshold
# - tool_force_threshold: The force applied to the tool exceeds a threshold
string condition_type
# The local direction along which to measure the force. Set to all zero for absolute force. For {tool, object}_force_threshold
geometry_msgs/Vector3 force_direction
# The name of the joint to monitor for joint_ext_torque_theshold
string joint_name
# The condition operator, either:
# - "<" which means value is bigger than threshold
# - ">" which means value is smaller than threshold
# - "|>" which means absolute value is bigger than threshold
# - "|<" which means absolute value is smaller than threshold
string condition_operator
# The threshold value either in [Nm] for rotational joints or in [N] for translational joints / forces.
float64 threshold 