# Micro Robot Detection and Occupancy Grid System

A computer vision system for detecting micro robots on a white background using a monocular USB camera and generating a 3×3 occupancy grid for path planning.

## Overview

This system:
- Captures video from a USB camera positioned above an 8cm × 8cm white bin
- Converts frames to binary (black/white) to detect dark robots
- Divides the bin into a 3×3 grid
- Detects and counts all robots
- Creates an occupancy grid showing which cells contain robots

## Requirements

- Python 3.7 or higher
- USB camera (monocular)
- White bin/background (8cm × 8cm)
- Dark-colored micro robots

## Installation

1. Install the required packages:
```bash
pip install -r requirements.txt
```

Or install manually:
```bash
pip install opencv-python numpy
```

## Usage

### Basic Usage

Run the detection system:
```bash
python robot_detection.py
```

### Controls

- **'q'** - Quit the application
- **'+'** - Increase detection threshold (if robots not detected)
- **'-'** - Decrease detection threshold (if too much noise)
- **'s'** - Save current frame and occupancy grid

### Adjustable Parameters

You can modify these parameters in the `RobotDetector` class:

```python
detector = RobotDetector(
    camera_index=0,      # Change if camera is not at index 0
    grid_size=3          # Change for different grid sizes
)

# Detection parameters (in __init__ method)
self.binary_threshold = 100    # Lower = detect lighter objects
self.min_contour_area = 50     # Minimum robot size in pixels
self.max_contour_area = 5000   # Maximum robot size in pixels
```

## Output

### Real-time Display

1. **Main View Window**: Shows the camera feed with:
   - Green grid overlay (3×3)
   - Red contours around detected robots
   - Blue circles at robot centroids
   - Yellow rings highlighting robots
   - Magenta numbers showing robot count per cell
   - Status information (total robots, threshold)

2. **Binary Image Window**: Shows the processed black/white image used for detection

### Console Output

Every 30 frames, the system prints:
```
--- Frame 0 ---
Total Robots Detected: 5
Occupancy Grid (3x3):
[[1 2 0]
 [0 1 1]
 [0 0 0]]
Grid Layout:
   1 |  2 |  0
   0 |  1 |  1
   0 |  0 |  0
------------------------------
```

### Saved Files (when pressing 's')

- `robot_detection_<timestamp>.jpg` - Annotated frame
- `occupancy_grid_<timestamp>.txt` - Occupancy grid data

## Calibration

### If Robots Are Not Detected:

1. **Adjust Threshold**: Press **'-'** to lower the threshold
2. **Check Lighting**: Ensure even lighting without shadows
3. **Modify Parameters**: Edit `binary_threshold` in the code (lower for lighter robots)

### If Too Much Noise:

1. **Adjust Threshold**: Press **'+'** to increase the threshold
2. **Adjust Size Filters**: Modify `min_contour_area` and `max_contour_area`
3. **Clean Background**: Ensure the white background is clean

### Optimal Setup:

- Position camera directly above the bin (perpendicular)
- Ensure the entire 8cm × 8cm area fills the camera view
- Use uniform lighting without shadows
- Keep the white background clean
- Robots should be significantly darker than the background

## Understanding the Occupancy Grid

The occupancy grid is a 3×3 matrix where:
- Each cell represents one section of the 8cm × 8cm bin
- The value in each cell is the **count** of robots in that cell
- Grid coordinates:

```
Cell (0,0) | Cell (0,1) | Cell (0,2)
Cell (1,0) | Cell (1,1) | Cell (1,2)
Cell (2,0) | Cell (2,1) | Cell (2,2)
```

### Example:
```
Occupancy Grid:
[[1 2 0]
 [0 1 1]
 [0 0 0]]
```

This means:
- Top-left cell: 1 robot
- Top-middle cell: 2 robots
- Middle-middle cell: 1 robot
- Middle-right cell: 1 robot
- All other cells: 0 robots
- **Total: 5 robots**

## Path Planning Integration

The occupancy grid can be directly used for path planning algorithms:

```python
# Get occupancy grid
occupancy_grid = detector.create_occupancy_grid(robot_positions, frame.shape)

# Example: Check if a cell is occupied
if occupancy_grid[row, col] > 0:
    print(f"Cell ({row},{col}) is occupied by {occupancy_grid[row, col]} robot(s)")
else:
    print(f"Cell ({row},{col}) is free")

# Example: Find free cells
free_cells = np.argwhere(occupancy_grid == 0)
print(f"Free cells: {free_cells}")
```

## Troubleshooting

### Camera Not Found
- Change `camera_index` parameter (try 0, 1, 2, etc.)
- Check if camera is properly connected
- On Linux, you may need camera permissions

### Poor Detection
- Adjust `binary_threshold` (default: 100)
- Ensure good lighting conditions
- Check robot size parameters (`min_contour_area`, `max_contour_area`)

### Grid Misalignment
- Ensure camera is perpendicular to the bin
- Verify the entire bin fills the camera view
- Adjust camera position or zoom

## Advanced Configuration

### Changing Grid Size

To use a different grid (e.g., 4×4):
```python
detector = RobotDetector(camera_index=0, grid_size=4)
```

### Custom Image Processing

Modify the `preprocess_frame` method for different detection methods:
```python
def preprocess_frame(self, frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Try adaptive thresholding for uneven lighting
    binary = cv2.adaptiveThreshold(gray, 255, 
                                   cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                   cv2.THRESH_BINARY_INV, 11, 2)
    
    return binary, gray
```

## System Architecture

```
Camera Feed → Grayscale → Gaussian Blur → Binary Threshold
     ↓
Contour Detection → Filter by Size → Calculate Centroids
     ↓
Map to Grid Cells → Create Occupancy Matrix → Display
```

## License

This project is provided as-is for educational and research purposes.

## Support

For issues or questions, adjust the parameters as described in the Calibration section above.