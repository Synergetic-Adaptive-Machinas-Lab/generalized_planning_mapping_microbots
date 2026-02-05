# Message for occupancy grid
std_msgs/Header header
int32 grid_rows        # Number of rows in grid
int32 grid_cols        # Number of columns in grid
int32[] data           # Flattened array of occupancy counts (row-major order)
string grid_type       # "coarse" or "fine"
