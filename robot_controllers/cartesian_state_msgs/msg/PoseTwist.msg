# Cartesian state of an end effector.
# The pose is expressed in the frame given by header.frame_id (the controller's root link).
# The twist is expressed in header.frame_id as well, with child_frame_id naming the tip link.
std_msgs/Header header
string child_frame_id
geometry_msgs/Pose pose
geometry_msgs/Twist twist
