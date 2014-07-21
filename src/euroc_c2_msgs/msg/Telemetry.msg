# The telemetry message contains the state of the axis, lwr, and the external camera.

#Standard ROS header. Contains the sequence number and simulation time.
std_msgs/Header header

# The names of the corresponding joints as mapping for all other vectors.
string[] joint_names
# The commanded joint telemetry (position, velocity, acceleration).
CommandedTelemetry commanded
# The measured joint telemetry (position, torque, external torque).
MeasuredTelemetry measured
