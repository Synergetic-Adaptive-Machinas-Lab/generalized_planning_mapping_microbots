# Microbot Detector ROS2 Package

A comprehensive ROS2 package for detecting and tracking microbots using computer vision with USB cameras. Features multi-resolution occupancy grids, real-time tracking, and extensive customization options.

## ROS2 Version
This package is designed for **ROS2 Humble/Iron/Jazzy**. For ROS1, see the separate ROS1 package.

## Features

- **Real-time Detection**: Detects black microbots on white backgrounds using computer vision
- **Multi-Resolution Grids**: Provides both coarse and fine occupancy grids for spatial analysis
- **Robot Tracking**: Assigns persistent IDs to robots across frames
- **Customizable Parameters**: Extensive configuration via YAML parameters
- **Multiple ROS2 Topics**: Publishes count, positions, grids, and debug images
- **ROI Support**: Crop camera view to specific regions of interest
- **Adjustable Detection**: Real-time threshold adjustment with keyboard controls

## Published Topics

| Topic | Message Type | QoS | Description |
|-------|--------------|-----|-------------|
| `/microbot/count` | `std_msgs/Int32` | Reliable | Total number of detected microbots |
| `/microbot/positions` | `microbot_detector_ros2/MicrobotArray` | Reliable | Array of all microbot positions with grid assignments |
| `/microbot/occupancy/coarse` | `microbot_detector_ros2/OccupancyGrid` | Reliable | Coarse occupancy grid (default 3x3) |
| `/microbot/occupancy/fine` | `microbot_detector_ros2/OccupancyGrid` | Reliable | Fine occupancy grid (default 9x9) |
| `/microbot/debug/image` | `sensor_msgs/Image` | Best Effort | Annotated debug visualization |
| `/microbot/debug/binary` | `sensor_msgs/Image` | Best Effort | Binary thresholded image |
| `/microbot/camera/raw` | `sensor_msgs/Image` | Best Effort | Raw camera feed |

## Custom Messages

### MicrobotPosition.msg
```
float32 x              # X coordinate in pixels
float32 y              # Y coordinate in pixels
float32 normalized_x   # X coordinate normalized to [0, 1]
float32 normalized_y   # Y coordinate normalized to [0, 1]
int32 coarse_grid_row  # Row in coarse grid
int32 coarse_grid_col  # Column in coarse grid
int32 fine_grid_row    # Row in fine grid
int32 fine_grid_col    # Column in fine grid
int32 robot_id         # Unique tracking ID
float32 area           # Contour area in pixels
```

### MicrobotArray.msg
```
std_msgs/Header header
int32 total_count
MicrobotPosition[] microbots
```

### OccupancyGrid.msg
```
std_msgs/Header header
int32 grid_rows
int32 grid_cols
int32[] data           # Flattened array (row-major)
string grid_type       # "coarse" or "fine"
```

## Installation

### Prerequisites
- ROS2 (Humble, Iron, or Jazzy recommended)
- Python 3.8+
- OpenCV
- NumPy
- SciPy

### Install System Dependencies
```bash
# Update package list
sudo apt update

# Install ROS2 dependencies
sudo apt install -y \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-image-tools \
    ros-${ROS_DISTRO}-rqt-image-view

# Install Python dependencies
sudo apt install -y \
    python3-pip \
    python3-opencv \
    python3-numpy \
    python3-scipy

# Or install via pip
pip3 install opencv-python numpy scipy
```

### Build the Package
```bash
# Create ROS2 workspace (if you don't have one)
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Copy this package
cp -r microbot_detector_ros2 .

# Build with colcon
cd ~/ros2_ws
colcon build --packages-select microbot_detector_ros2

# Source the workspace
source install/setup.bash

# Add to bashrc for persistence
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

## Quick Start

### 1. Find Your Camera
```bash
ls /dev/video*
# Note which index your camera is (usually /dev/video0)
```

### 2. Edit Configuration
Edit `config/detector_params.yaml` and set your camera index:
```yaml
microbot_detector:
  ros__parameters:
    camera:
      index: 0  # Change to your camera index
```

### 3. Launch the Detector
```bash
# Source your workspace
source ~/ros2_ws/install/setup.bash

# Launch with default config
ros2 launch microbot_detector_ros2 microbot_detector.launch.py

# Or with custom config
ros2 launch microbot_detector_ros2 microbot_detector.launch.py \
    config_file:=/path/to/custom_config.yaml
```

### 4. View the Data

**Terminal 1: View robot count**
```bash
ros2 topic echo /microbot/count
```

**Terminal 2: View robot positions**
```bash
ros2 topic echo /microbot/positions
```

**Terminal 3: View occupancy grid**
```bash
ros2 topic echo /microbot/occupancy/coarse
```

**Terminal 4: View images**
```bash
# View annotated debug image
ros2 run rqt_image_view rqt_image_view /microbot/debug/image
```

## Configuration

All parameters are in `config/detector_params.yaml`. Key settings:

### Camera Settings
```yaml
camera:
  index: 0              # USB camera index
  frame_width: 640      # Camera resolution
  frame_height: 480
  
  # Region of Interest (set to -1 to disable)
  roi_x: -1
  roi_y: -1
  roi_width: -1
  roi_height: -1
```

### Detection Parameters
```yaml
detection:
  binary_threshold: 140      # Adjust based on lighting (0-255)
  min_contour_area: 5        # Minimum robot size (pixels²)
  max_contour_area: 5000     # Maximum robot size (pixels²)
```

### Grid Configuration
```yaml
grid:
  coarse_rows: 3        # Coarse grid dimensions
  coarse_cols: 3
  fine_rows: 9          # Fine grid dimensions  
  fine_cols: 9
```

## Calibration

Before first use, calibrate your camera settings:

```bash
cd ~/ros2_ws/src/microbot_detector_ros2/microbot_detector_ros2
python3 camera_calibration.py 0  # Replace 0 with your camera index
```

**Controls:**
- Press **+** to increase threshold
- Press **-** to decrease threshold
- Press **a/A** to adjust minimum area
- Press **z/Z** to adjust maximum area
- Press **s** to save settings
- Press **q** to quit

This creates `calibrated_params.yaml` which you can use with the launch file.

## ROS2 Commands

### List Topics
```bash
ros2 topic list
```

### View Topic Data
```bash
# View count
ros2 topic echo /microbot/count

# View positions (with limit)
ros2 topic echo /microbot/positions --once

# View coarse grid
ros2 topic echo /microbot/occupancy/coarse
```

### Check Topic Info
```bash
ros2 topic info /microbot/positions
ros2 topic hz /microbot/count
ros2 topic bw /microbot/debug/image
```

### View Parameters
```bash
ros2 param list /microbot_detector
ros2 param get /microbot_detector camera.index
```

### Set Parameters at Runtime
```bash
ros2 param set /microbot_detector detection.binary_threshold 160
ros2 param set /microbot_detector grid.fine_rows 12
```

### Record Data
```bash
# Record all topics
ros2 bag record -a

# Record specific topics
ros2 bag record /microbot/count /microbot/positions /microbot/occupancy/coarse
```

### Play Back Recorded Data
```bash
ros2 bag play <bag_file>
```

## Keyboard Controls (when OpenCV windows are active)

- **q**: Quit the program
- **+**: Increase binary threshold (+5)
- **-**: Decrease binary threshold (-5)

## Troubleshooting

### Camera Not Opening
```bash
# List available cameras
ls -l /dev/video*

# Check permissions
sudo usermod -a -G video $USER
# Then logout and login

# Try different camera indices in config
```

### No Robots Detected
1. Check binary image window - robots should appear as white blobs
2. Adjust `binary_threshold` (try values between 100-180)
3. Check lighting conditions
4. Verify `min_contour_area` and `max_contour_area` settings

### Build Errors
```bash
# Clean and rebuild
cd ~/ros2_ws
rm -rf build/ install/ log/
colcon build --packages-select microbot_detector_ros2

# Check dependencies
rosdep install --from-paths src --ignore-src -r -y
```

### Topics Not Publishing
```bash
# Check node is running
ros2 node list

# Check node info
ros2 node info /microbot_detector

# View node logs
ros2 run ros2 bag_play <bag_file>
```

## Example Usage

### Monitor Count in Python
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class CountMonitor(Node):
    def __init__(self):
        super().__init__('count_monitor')
        self.subscription = self.create_subscription(
            Int32,
            '/microbot/count',
            self.callback,
            10)
    
    def callback(self, msg):
        self.get_logger().info(f'Robot count: {msg.data}')

def main():
    rclpy.init()
    node = CountMonitor()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Track Positions
```python
import rclpy
from rclpy.node import Node
from microbot_detector_ros2.msg import MicrobotArray

class PositionTracker(Node):
    def __init__(self):
        super().__init__('position_tracker')
        self.subscription = self.create_subscription(
            MicrobotArray,
            '/microbot/positions',
            self.callback,
            10)
    
    def callback(self, msg):
        self.get_logger().info(f'Tracking {msg.total_count} robots')
        for robot in msg.microbots:
            self.get_logger().info(
                f'  Robot {robot.robot_id}: ({robot.x:.1f}, {robot.y:.1f})')

def main():
    rclpy.init()
    node = PositionTracker()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Performance Optimization

### For Maximum Speed
```yaml
camera:
  frame_width: 320
  frame_height: 240

detection:
  use_morphology: false

grid:
  fine_rows: 6
  fine_cols: 6

visualization:
  show_opencv_windows: false

performance:
  publish_rate: 60
  skip_frames: 0
```

### For Maximum Accuracy
```yaml
camera:
  frame_width: 1280
  frame_height: 720

detection:
  gaussian_blur_kernel: 9
  use_morphology: true
  morph_iterations: 3

grid:
  fine_rows: 20
  fine_cols: 20

performance:
  skip_frames: 0
```

## System Requirements

**Minimum:**
- Ubuntu 22.04 (for Humble)
- ROS2 Humble/Iron/Jazzy
- 2GB RAM
- 2-core CPU
- USB 2.0 camera

**Recommended:**
- Ubuntu 22.04/24.04
- ROS2 Jazzy
- 4GB+ RAM
- 4-core+ CPU
- USB 3.0 camera (720p+)

## License

MIT License - See LICENSE file for details

## Contributing

Contributions are welcome! Please submit issues and pull requests.

## Citation

If you use this package in your research:
```
@software{microbot_detector_ros2,
  title={Microbot Detector ROS2 Package},
  author={Your Name},
  year={2026},
  url={https://github.com/yourusername/microbot_detector_ros2}
}
```

## Support

For issues and questions:
- Check this README
- Review ROS2 documentation
- Open an issue on GitHub
