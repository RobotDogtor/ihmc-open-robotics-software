# This message is part of the IHMC whole-body controller API.
# It is a lightweight version of SO3TrajectoryMessage designed for streaming.

FrameInformation frame_information
# Flag that tells the controller whether the use of a custom control frame is requested.
bool use_custom_control_frame false
# Pose of custom control frame expressed in the end-effector frame.
# This is the frame attached to the rigid body that the taskspace trajectory is defined for.
geometry_msgs/Pose control_frame_pose

# Define the desired 3D orientation to be reached.
geometry_msgs/Quaternion orientation
# Define the desired 3D angular velocity to be reached.
geometry_msgs/Vector3 angular_velocity