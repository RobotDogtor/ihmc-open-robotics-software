# This message is part of the IHMC height map module

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Discretization of the height map grid
float64 xy_resolution -1.0

# The height map covers a square of this width
float64 grid_size_xy -1.0

# X coordinate of the center of the height map
float64 grid_center_x 0.0

# Y coordinate of the center of the height map
float64 grid_center_y 0.0

# Height of the ground plane, which is assumed to be flat
float64 estimated_ground_height 0.0

# List of height map keys. See HeightMapTools for converting keys to coordinates
int32[<=30000] keys

# List of heights, which correspond to the list of keys
float32[<=30000] heights
