import cv2
import numpy as np
import time

class RobotDetector:
    def __init__(self, camera_index=0, grid_size=3):
        """
        Initialize the robot detector
        
        Args:
            camera_index: USB camera index (usually 0)
            grid_size: Number of rows/columns in grid (default 3x3)
        """
        self.camera_index = camera_index
        self.grid_size = grid_size
        self.cap = None
        
        # Parameters for detection (adjustable)
        self.binary_threshold = 100  # Threshold for black/white conversion
        self.min_contour_area = 50   # Minimum area to consider as a robot
        self.max_contour_area = 5000 # Maximum area to consider as a robot
        
    def initialize_camera(self):
        """Initialize the USB camera"""
        self.cap = cv2.VideoCapture(self.camera_index)
        if not self.cap.isOpened():
            raise Exception(f"Cannot open camera {self.camera_index}")
        
        # Set camera properties for better quality
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        print("Camera initialized successfully")
        return True
    
    def preprocess_frame(self, frame):
        """
        Convert frame to binary (black and white)
        
        Args:
            frame: Input BGR frame from camera
            
        Returns:
            binary: Binary image (black robots on white background)
            gray: Grayscale image
        """
        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Apply Gaussian blur to reduce noise
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        
        # Convert to binary using thresholding
        # Invert so robots (dark) become white in binary image
        _, binary = cv2.threshold(blurred, self.binary_threshold, 255, cv2.THRESH_BINARY_INV)
        
        return binary, gray
    
    def detect_robots(self, binary_image):
        """
        Detect robot positions from binary image
        
        Args:
            binary_image: Binary image with robots as white blobs
            
        Returns:
            robot_positions: List of (x, y) centroid positions
            contours: Detected contours
        """
        # Find contours
        contours, _ = cv2.findContours(binary_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        robot_positions = []
        valid_contours = []
        
        # Filter contours by area and get centroids
        for contour in contours:
            area = cv2.contourArea(contour)
            
            # Filter by size to avoid noise
            if self.min_contour_area < area < self.max_contour_area:
                # Calculate centroid
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    robot_positions.append((cx, cy))
                    valid_contours.append(contour)
        
        return robot_positions, valid_contours
    
    def create_occupancy_grid(self, robot_positions, frame_shape):
        """
        Create 3x3 occupancy grid based on robot positions
        
        Args:
            robot_positions: List of (x, y) positions
            frame_shape: Shape of the frame (height, width)
            
        Returns:
            occupancy_grid: 3x3 numpy array with robot counts per cell
        """
        height, width = frame_shape[:2]
        occupancy_grid = np.zeros((self.grid_size, self.grid_size), dtype=int)
        
        # Calculate cell dimensions
        cell_width = width / self.grid_size
        cell_height = height / self.grid_size
        
        # Map each robot to a grid cell
        for x, y in robot_positions:
            # Determine which grid cell the robot is in
            col = int(x / cell_width)
            row = int(y / cell_height)
            
            # Ensure within bounds
            col = min(col, self.grid_size - 1)
            row = min(row, self.grid_size - 1)
            
            # Increment occupancy count
            occupancy_grid[row, col] += 1
        
        return occupancy_grid
    
    def draw_grid_overlay(self, frame, occupancy_grid, robot_positions, contours):
        """
        Draw grid lines, robot markers, and occupancy info on frame
        
        Args:
            frame: Input frame
            occupancy_grid: 3x3 occupancy matrix
            robot_positions: List of robot positions
            contours: Detected contours
            
        Returns:
            annotated_frame: Frame with overlays
        """
        annotated = frame.copy()
        height, width = frame.shape[:2]
        
        cell_width = width / self.grid_size
        cell_height = height / self.grid_size
        
        # Draw grid lines
        for i in range(1, self.grid_size):
            # Vertical lines
            x = int(i * cell_width)
            cv2.line(annotated, (x, 0), (x, height), (0, 255, 0), 2)
            
            # Horizontal lines
            y = int(i * cell_height)
            cv2.line(annotated, (0, y), (width, y), (0, 255, 0), 2)
        
        # Draw contours
        cv2.drawContours(annotated, contours, -1, (0, 0, 255), 2)
        
        # Draw robot centroids
        for x, y in robot_positions:
            cv2.circle(annotated, (x, y), 5, (255, 0, 0), -1)
            cv2.circle(annotated, (x, y), 8, (0, 255, 255), 2)
        
        # Draw occupancy count in each cell
        for row in range(self.grid_size):
            for col in range(self.grid_size):
                count = occupancy_grid[row, col]
                
                # Calculate cell center for text
                text_x = int((col + 0.5) * cell_width)
                text_y = int((row + 0.5) * cell_height)
                
                # Draw count
                if count > 0:
                    cv2.putText(annotated, str(count), (text_x - 10, text_y + 10),
                               cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 255), 2)
        
        return annotated
    
    def run(self):
        """Main loop for robot detection"""
        if not self.initialize_camera():
            return
        
        print("\n=== Robot Detection System ===")
        print("Controls:")
        print("  'q' - Quit")
        print("  '+' - Increase threshold")
        print("  '-' - Decrease threshold")
        print("  's' - Save current frame")
        print("================================\n")
        
        frame_count = 0
        
        try:
            while True:
                ret, frame = self.cap.read()
                
                if not ret:
                    print("Failed to grab frame")
                    break
                
                # Process frame
                binary, gray = self.preprocess_frame(frame)
                robot_positions, contours = self.detect_robots(binary)
                occupancy_grid = self.create_occupancy_grid(robot_positions, frame.shape)
                
                # Create visualization
                annotated_frame = self.draw_grid_overlay(frame, occupancy_grid, 
                                                         robot_positions, contours)
                
                # Add info text
                total_robots = len(robot_positions)
                info_text = f"Robots: {total_robots} | Threshold: {self.binary_threshold}"
                cv2.putText(annotated_frame, info_text, (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                # Display frames
                cv2.imshow('Robot Detection - Main View', annotated_frame)
                cv2.imshow('Binary Image', binary)
                
                # Print occupancy grid to console every 30 frames
                if frame_count % 30 == 0:
                    print(f"\n--- Frame {frame_count} ---")
                    print(f"Total Robots Detected: {total_robots}")
                    print("Occupancy Grid (3x3):")
                    print(occupancy_grid)
                    print("Grid Layout:")
                    for row in range(self.grid_size):
                        row_str = " | ".join([f"{occupancy_grid[row, col]:2d}" 
                                             for col in range(self.grid_size)])
                        print(f"  {row_str}")
                    print("-" * 30)
                
                frame_count += 1
                
                # Handle keyboard input
                key = cv2.waitKey(1) & 0xFF
                
                if key == ord('q'):
                    print("\nExiting...")
                    break
                elif key == ord('+'):
                    self.binary_threshold = min(255, self.binary_threshold + 5)
                    print(f"Threshold increased to {self.binary_threshold}")
                elif key == ord('-'):
                    self.binary_threshold = max(0, self.binary_threshold - 5)
                    print(f"Threshold decreased to {self.binary_threshold}")
                elif key == ord('s'):
                    filename = f"robot_detection_{int(time.time())}.jpg"
                    cv2.imwrite(filename, annotated_frame)
                    print(f"Saved frame as {filename}")
                    
                    # Also save occupancy grid to text file
                    grid_filename = f"occupancy_grid_{int(time.time())}.txt"
                    with open(grid_filename, 'w') as f:
                        f.write(f"Total Robots: {total_robots}\n")
                        f.write("Occupancy Grid (3x3):\n")
                        f.write(str(occupancy_grid))
                    print(f"Saved occupancy grid as {grid_filename}")
        
        finally:
            self.cap.release()
            cv2.destroyAllWindows()
            print("Camera released and windows closed")


def main():
    """Main entry point"""
    # Create detector instance
    # Change camera_index if your USB camera is not at index 0
    detector = RobotDetector(camera_index=0, grid_size=3)
    
    # Run detection
    detector.run()


if __name__ == "__main__":
    main()