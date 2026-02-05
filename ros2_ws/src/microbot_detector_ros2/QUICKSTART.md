# Quick Start Guide - Microbot Detector ROS2

## Installation (5 minutes)

### 1. Copy Package to Workspace
```bash
# Create ROS2 workspace if you don't have one
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Copy the package
cp -r microbot_detector_ros2 .
```

### 2. Install Dependencies
```bash
# Install system dependencies
sudo apt update
sudo apt install -y \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-image-tools \
    python3-opencv python3-numpy python3-scipy
```

### 3. Build the Package
```bash
cd ~/ros2_ws
colcon build --packages-select microbot_detector_ros2
source install/setup.bash

# Add to bashrc
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

## Running the Detector (2 commands)

### Terminal 1: Launch Detector
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch microbot_detector_ros2 microbot_detector.launch.py
```

### Terminal 2: View Robot Count
```bash
ros2 topic echo /microbot/count
```

## View Images
```bash
# View debug image with detections
ros2 run rqt_image_view rqt_image_view /microbot/debug/image
```

## Calibration (Optional but Recommended)

Before first use, run calibration:
```bash
cd ~/ros2_ws/src/microbot_detector_ros2/microbot_detector_ros2
python3 camera_calibration.py 0  # Replace 0 with your camera index

# Press +/- to adjust threshold
# Press 's' to save settings
# Press 'q' to quit
```

## Common Commands

```bash
# List all topics
ros2 topic list

# View occupancy grid
ros2 topic echo /microbot/occupancy/coarse

# View robot positions
ros2 topic echo /microbot/positions

# Record data
ros2 bag record -a

# Check parameters
ros2 param list /microbot_detector

# Change parameter at runtime
ros2 param set /microbot_detector detection.binary_threshold 160
```

## Troubleshooting

### Camera Not Found
```bash
# Find your camera
ls /dev/video*

# Edit config file
nano ~/ros2_ws/src/microbot_detector_ros2/config/detector_params.yaml
# Change: camera.index to your camera number

# Rebuild
cd ~/ros2_ws
colcon build --packages-select microbot_detector_ros2
```

### No Robots Detected
- Run calibration tool: `python3 camera_calibration.py 0`
- Adjust threshold with +/- keys
- Ensure white background and good lighting

### Package Not Found
```bash
# Make sure you sourced the workspace
source ~/ros2_ws/install/setup.bash

# Rebuild if needed
cd ~/ros2_ws
colcon build --packages-select microbot_detector_ros2
```

## Next Steps

1. Read README.md for full documentation
2. Customize config/detector_params.yaml
3. Test with your microbots
4. Record data with `ros2 bag record`
5. Develop your own processing nodes

## Key Differences from ROS1

- Use `ros2 launch` instead of `roslaunch`
- Use `ros2 topic` instead of `rostopic`
- Use `ros2 param` instead of `rosparam`
- Use `colcon build` instead of `catkin_make`
- Source `install/setup.bash` instead of `devel/setup.bash`
- Parameter files use nested structure under `ros__parameters`

## Get Help

- Check README.md
- View ROS2 docs: docs.ros.org
- Check parameters: `ros2 param list /microbot_detector`
- View node info: `ros2 node info /microbot_detector`
