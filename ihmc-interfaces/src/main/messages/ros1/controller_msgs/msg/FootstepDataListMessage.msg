# This message is part of the IHMC whole-body controller API.
# This message commands the controller to execute a list of footsteps.
# See FootstepDataMessage for more information about defining a footstep.

int8 EXECUTION_TIMING_CONTROL_DURATIONS=0

int8 EXECUTION_TIMING_CONTROL_ABSOLUTE_TIMINGS=1

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Defines the list of footstep to perform.
controller_msgs/FootstepDataMessage[] footstep_data_list

# When CONTROL_DURATIONS is chosen:
# The controller will try to achieve the swing_duration and the transfer_duration specified in the message.
# If a footstep touches down early, the next step will not be affected by this and the whole trajectory might finish earlier than expected.
# When CONTROL_ABSOLUTE_TIMINGS is chosen:
# The controller will compute the expected times for swing start and touchdown and attempt to start a footstep at that time.
# If a footstep touches down early, the following transfer will be extended to make up for this
# time difference and the footstep plan will finish at the expected time.
int8 execution_timing

# The swing_duration is the time a foot is not in ground contact during a step.
# Each step in a list of footsteps might have a different swing duration.
# The value specified here is a default value, used if a footstep in this list was created without a swing_duration.
# When set to zero or a negative value, the controller will its own default value.
# Field default value -1.0
float64 default_swing_duration

# The transfer_duration is the time spent with the feet in ground contact before a step.
# Each step in a list of footsteps might have a different transfer duration.
# The value specified here is a default value, used if a footstep in this list was created without a transfer-duration.
# When set to zero or a negative value, the controller will its own default value.
# Field default value -1.0
float64 default_transfer_duration

# Specifies the time used to return to a stable standing stance after the execution of the
# footstep list is finished. If the value is negative the default_transfer_duration will be used,
# which in turn if not provided indicate the controller to use its own internal default value.
# Field default value -1.0
float64 final_transfer_duration

# If false the controller adjust each footstep height to be at the support sole height.
# Field default value True
bool trust_height_of_footsteps

# Contains information on whether the robot can automatically adjust its footsteps to retain balance.
bool are_footsteps_adjustable

# If true the controller will adjust the x and y coordinates of the upcoming footsteps with the location error of previous steps.
bool offset_footsteps_with_execution_error

# If true the controller will adjust the z coordinate of the adjust upcoming footsteps with the location error of previous steps.
bool offset_footsteps_height_with_execution_error

# Properties for queueing footstep lists.
controller_msgs/QueueableMessage queueing_properties


