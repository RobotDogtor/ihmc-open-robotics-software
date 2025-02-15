# This message is part of the IHMC whole-body inverse kinematics module.
# Similar to KinematicsToolboxConfigurationMessage, this contains auxiliary information that allows to further customized the behavior of the solver.
# The parameters exposed through this message are specific to application to humanoid robots.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# When set to true, the solver will maintain, if possible, the current x and y coordinates of the center
# of mass. By 'current', it means that the solver will use the robot configuration data
# broadcasted by the controller to obtain the center of mass position.
# Field default value True
bool hold_current_center_of_mass_xy_position

# When set to true, the solver will process the balance status from an active controller session and build a support polygon.
# Note that the auto support polygon feature is overidden by a user specified support polygon when provided.
# - when the walking controller is running, the CapturabilityBasedStatus message is used to identify the current support polygon.
# - when the multi-contact controller is running, the MultiContactBalanceStatus message is used to identify the current support polygon.
# Field default value True
bool enable_auto_support_polygon

# When set to true, the solver will hold the pose of the rigid-bodies with active contact points.
# - when the walking controller is running, the rigid-bodies in question are the feet.
# - when a multi-contact controller is running, any rigid-body of the robot can be in contact.
# Field default value True
bool hold_support_rigid_bodies


