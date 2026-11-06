# FK of measured joints, not an independent Cartesian measurement.
std_msgs/Header header
geometry_msgs/Pose left
geometry_msgs/Pose right
float64[7] left_hand
float64[7] right_hand
# Unitree messages lack ROS headers. These are ROS receipt timestamps.
builtin_interfaces/Time body_stamp
builtin_interfaces/Time left_hand_stamp
builtin_interfaces/Time right_hand_stamp
bool valid
string reason
