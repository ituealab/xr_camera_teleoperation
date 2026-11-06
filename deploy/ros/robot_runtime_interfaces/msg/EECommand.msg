# torso_link-relative palm targets. Metres, quaternion xyzw; Dex3 motor radians.
# Stamp is creation time, valid_for is TTL; future/expired commands are rejected.
std_msgs/Header header
string command_id
builtin_interfaces/Duration valid_for
geometry_msgs/Pose left
geometry_msgs/Pose right
float64[7] left_hand
float64[7] right_hand
