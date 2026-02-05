# Message for individual microbot position
float32 x              # X coordinate in pixels
float32 y              # Y coordinate in pixels
float32 normalized_x   # X coordinate normalized to [0, 1]
float32 normalized_y   # Y coordinate normalized to [0, 1]
int32 coarse_grid_row  # Row in coarse grid (e.g., 0-2 for 3x3)
int32 coarse_grid_col  # Column in coarse grid
int32 fine_grid_row    # Row in fine grid
int32 fine_grid_col    # Column in fine grid
int32 robot_id         # Unique ID for tracking
float32 area           # Contour area in pixels
