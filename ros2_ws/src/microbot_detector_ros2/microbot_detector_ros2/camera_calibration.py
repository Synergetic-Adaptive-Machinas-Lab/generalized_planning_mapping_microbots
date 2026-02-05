#!/usr/bin/env python3

import cv2
import numpy as np
import sys

def save_settings_ros2(threshold, min_area, max_area, width, height, camera_index):
    """Save calibrated settings to a ROS2 config file"""
    
    filename = "calibrated_params.yaml"
    
    yaml_content = f"""# Auto-generated calibrated parameters for ROS2

microbot_detector:
  ros__parameters:
    camera:
      index: {camera_index}
      frame_width: {width}
      frame_height: {height}
      fps: 30
      roi_x: -1
      roi_y: -1
      roi_width: -1
      roi_height: -1

    detection:
      binary_threshold: {threshold}
      min_contour_area: {min_area}
      max_contour_area: {max_area}
      gaussian_blur_kernel: 5
      use_morphology: true
      morph_kernel_size: 3
      morph_iterations: 1

    grid:
      coarse_rows: 3
      coarse_cols: 3
      fine_rows: 9
      fine_cols: 9

    topics:
      robot_count: "/microbot/count"
      robot_positions: "/microbot/positions"
      coarse_grid: "/microbot/occupancy/coarse"
      fine_grid: "/microbot/occupancy/fine"
      debug_image: "/microbot/debug/image"
      binary_image: "/microbot/debug/binary"
      raw_image: "/microbot/camera/raw"

    visualization:
      show_opencv_windows: true
      draw_grid_lines: true
      draw_contours: true
      draw_centroids: true
      draw_ids: true
      draw_occupancy_text: true
      grid_color: [0, 255, 0]
      contour_color: [0, 0, 255]
      centroid_color: [255, 0, 0]
      id_text_color: [255, 255, 0]

    tracking:
      enable_tracking: true
      max_distance: 50

    performance:
      publish_rate: 30
      skip_frames: 0

    logging:
      console_output: true
      console_rate: 1.0
"""
    
    with open(filename, 'w') as f:
        f.write(yaml_content)
    
    print("\n" + "="*70)
    print(f"✓ SETTINGS SAVED to: {filename}")
    print("="*70)
    print(f"Camera Index: {camera_index}")
    print(f"Threshold: {threshold}")
    print(f"Min Area: {min_area}")
    print(f"Max Area: {max_area}")
    print("="*70)

def main():
    camera_index = 4
    
    if len(sys.argv) > 1:
        try:
            camera_index = int(sys.argv[1])
        except ValueError:
            print(f"Invalid camera index: {sys.argv[1]}")
            sys.exit(1)
    
    print("="*70)
    print("Microbot Detector - Camera Calibration (ROS2)")
    print("="*70)
    print(f"\nOpening camera at index {camera_index}...")
    
    cap = cv2.VideoCapture(camera_index)
    
    if not cap.isOpened():
        print(f"\nERROR: Cannot open camera")
        sys.exit(1)
    
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    
    print(f"✓ Camera opened: {width}x{height}")
    
    print("\n" + "="*70)
    print("IMPORTANT: CLICK ON THE CAMERA VIEW WINDOW TO ACTIVATE IT!")
    print("="*70)
    print("CONTROLS (press keys while window is active):")
    print("  '+' or '='  : Increase threshold")
    print("  '-' or '_'  : Decrease threshold")
    print("  'a'         : Decrease min area")
    print("  'A'         : Increase min area")
    print("  'z'         : Decrease max area")
    print("  'Z'         : Increase max area")
    print("  's'         : SAVE settings")
    print("  'q' or ESC  : Quit")
    print("="*70)
    print("\nAdjust threshold until microbots appear WHITE in Binary window")
    print("="*70)
    
    # Detection parameters
    threshold = 80
    min_area = 5
    max_area = 5000
    
    # Create named windows
    cv2.namedWindow('Camera View', cv2.WINDOW_NORMAL)
    cv2.namedWindow('Binary Threshold', cv2.WINDOW_NORMAL)
    
    try:
        while True:
            ret, frame = cap.read()
            
            if not ret:
                print("Failed to grab frame")
                break
            
            # Process frame
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            blurred = cv2.GaussianBlur(gray, (5, 5), 0)
            _, binary = cv2.threshold(blurred, threshold, 255, cv2.THRESH_BINARY_INV)
            
            contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            valid_count = 0
            display = frame.copy()
            
            for contour in contours:
                area = cv2.contourArea(contour)
                
                if min_area < area < max_area:
                    valid_count += 1
                    cv2.drawContours(display, [contour], -1, (0, 255, 0), 2)
                    
                    M = cv2.moments(contour)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        cv2.circle(display, (cx, cy), 5, (255, 0, 0), -1)
            
            # Add large text overlay
            info_lines = [
                f"Threshold: {threshold}",
                f"Detected: {valid_count}",
                f"CLICK THIS WINDOW, THEN PRESS 's' TO SAVE"
            ]
            
            y_offset = 30
            for line in info_lines:
                cv2.putText(display, line, (10, y_offset),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                y_offset += 30
            
            cv2.imshow('Camera View', display)
            cv2.imshow('Binary Threshold', binary)
            
            # Wait for key with timeout
            key = cv2.waitKey(30) & 0xFF
            
            if key != 255:  # Key was pressed
                if key == ord('q') or key == 27:
                    print("\nQuitting...")
                    break
                elif key == ord('+') or key == ord('='):
                    threshold = min(255, threshold + 5)
                    print(f">>> Threshold: {threshold}")
                elif key == ord('-') or key == ord('_'):
                    threshold = max(0, threshold - 5)
                    print(f">>> Threshold: {threshold}")
                elif key == ord('a'):
                    min_area = max(1, min_area - 5)
                    print(f">>> Min Area: {min_area}")
                elif key == ord('A'):
                    min_area += 5
                    print(f">>> Min Area: {min_area}")
                elif key == ord('z'):
                    max_area = max(min_area + 10, max_area - 100)
                    print(f">>> Max Area: {max_area}")
                elif key == ord('Z'):
                    max_area += 100
                    print(f">>> Max Area: {max_area}")
                elif key == ord('s'):
                    save_settings_ros2(threshold, min_area, max_area, width, height, camera_index)
                    print("\n>>> Press 'q' to quit or continue adjusting...")
    
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        cap.release()
        cv2.destroyAllWindows()
        print("\nCalibration complete!")

if __name__ == '__main__':
    main()
