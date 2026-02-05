"""
Example script demonstrating how to use the occupancy grid
for path planning applications
"""

import numpy as np
from robot_detection import RobotDetector
import cv2


class PathPlanner:
    """Simple path planning using occupancy grid"""
    
    def __init__(self, occupancy_grid):
        """
        Initialize path planner with occupancy grid
        
        Args:
            occupancy_grid: 3x3 numpy array with robot counts
        """
        self.occupancy_grid = occupancy_grid
        self.grid_size = occupancy_grid.shape[0]
    
    def is_cell_free(self, row, col):
        """Check if a cell is free (no robots)"""
        if 0 <= row < self.grid_size and 0 <= col < self.grid_size:
            return self.occupancy_grid[row, col] == 0
        return False
    
    def get_free_cells(self):
        """Get list of all free cells"""
        free_cells = []
        for row in range(self.grid_size):
            for col in range(self.grid_size):
                if self.occupancy_grid[row, col] == 0:
                    free_cells.append((row, col))
        return free_cells
    
    def get_occupied_cells(self):
        """Get list of all occupied cells with robot counts"""
        occupied_cells = []
        for row in range(self.grid_size):
            for col in range(self.grid_size):
                if self.occupancy_grid[row, col] > 0:
                    occupied_cells.append((row, col, self.occupancy_grid[row, col]))
        return occupied_cells
    
    def get_neighbors(self, row, col, allow_diagonal=False):
        """
        Get neighboring cells (4-connected or 8-connected)
        
        Args:
            row, col: Current cell position
            allow_diagonal: If True, includes diagonal neighbors
        """
        neighbors = []
        
        # 4-connected (up, down, left, right)
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        
        # Add diagonals for 8-connected
        if allow_diagonal:
            directions += [(-1, -1), (-1, 1), (1, -1), (1, 1)]
        
        for dr, dc in directions:
            new_row, new_col = row + dr, col + dc
            if 0 <= new_row < self.grid_size and 0 <= new_col < self.grid_size:
                neighbors.append((new_row, new_col))
        
        return neighbors
    
    def find_simple_path(self, start, goal):
        """
        Find a simple path from start to goal avoiding occupied cells
        Uses breadth-first search
        
        Args:
            start: (row, col) tuple
            goal: (row, col) tuple
            
        Returns:
            path: List of (row, col) tuples or None if no path
        """
        if not self.is_cell_free(start[0], start[1]) or not self.is_cell_free(goal[0], goal[1]):
            return None
        
        from collections import deque
        
        queue = deque([(start, [start])])
        visited = {start}
        
        while queue:
            (current, path) = queue.popleft()
            
            if current == goal:
                return path
            
            for neighbor in self.get_neighbors(current[0], current[1], allow_diagonal=True):
                if neighbor not in visited and self.is_cell_free(neighbor[0], neighbor[1]):
                    visited.add(neighbor)
                    queue.append((neighbor, path + [neighbor]))
        
        return None  # No path found
    
    def visualize_grid(self):
        """Print a visual representation of the occupancy grid"""
        print("\nOccupancy Grid Visualization:")
        print("=" * (self.grid_size * 4 + 1))
        
        for row in range(self.grid_size):
            row_str = "|"
            for col in range(self.grid_size):
                count = self.occupancy_grid[row, col]
                if count == 0:
                    row_str += " . |"
                else:
                    row_str += f" {count} |"
            print(row_str)
            print("-" * (self.grid_size * 4 + 1))
        
        print("\nLegend: '.' = free, number = robot count")


def demo_path_planning():
    """Demonstrate path planning with example occupancy grid"""
    
    print("=== Path Planning Demo ===\n")
    
    # Example occupancy grid
    example_grid = np.array([
        [1, 0, 0],
        [0, 2, 0],
        [0, 0, 1]
    ])
    
    planner = PathPlanner(example_grid)
    
    # Visualize the grid
    planner.visualize_grid()
    
    # Get free cells
    free_cells = planner.get_free_cells()
    print(f"\nFree cells: {free_cells}")
    
    # Get occupied cells
    occupied_cells = planner.get_occupied_cells()
    print(f"Occupied cells (row, col, count): {occupied_cells}")
    
    # Find path from top-right to bottom-left
    start = (0, 2)  # Top-right
    goal = (2, 0)   # Bottom-left
    
    print(f"\nFinding path from {start} to {goal}...")
    path = planner.find_simple_path(start, goal)
    
    if path:
        print(f"Path found: {path}")
        print(f"Path length: {len(path)} cells")
    else:
        print("No path found!")
    
    # Try another path
    start2 = (0, 1)
    goal2 = (2, 2)
    
    print(f"\nFinding path from {start2} to {goal2}...")
    path2 = planner.find_simple_path(start2, goal2)
    
    if path2:
        print(f"Path found: {path2}")
        print(f"Path length: {len(path2)} cells")
    else:
        print("No path found!")


def live_detection_with_path_planning():
    """
    Run live detection with path planning capabilities
    This shows how to integrate the detector with path planning
    """
    print("\n=== Live Detection with Path Planning ===")
    print("This will start the camera and show path planning options\n")
    
    detector = RobotDetector(camera_index=0, grid_size=3)
    
    if not detector.initialize_camera():
        return
    
    print("Press 'p' to analyze path planning options")
    print("Press 'q' to quit\n")
    
    try:
        while True:
            ret, frame = detector.cap.read()
            
            if not ret:
                break
            
            # Process frame
            binary, gray = detector.preprocess_frame(frame)
            robot_positions, contours = detector.detect_robots(binary)
            occupancy_grid = detector.create_occupancy_grid(robot_positions, frame.shape)
            
            # Create visualization
            annotated_frame = detector.draw_grid_overlay(frame, occupancy_grid,
                                                         robot_positions, contours)
            
            # Display
            cv2.imshow('Robot Detection with Path Planning', annotated_frame)
            
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('q'):
                break
            elif key == ord('p'):
                # Analyze path planning
                print("\n" + "="*50)
                print("PATH PLANNING ANALYSIS")
                print("="*50)
                
                planner = PathPlanner(occupancy_grid)
                planner.visualize_grid()
                
                free_cells = planner.get_free_cells()
                occupied_cells = planner.get_occupied_cells()
                
                print(f"\nTotal robots detected: {len(robot_positions)}")
                print(f"Free cells ({len(free_cells)}): {free_cells}")
                print(f"Occupied cells ({len(occupied_cells)}): {occupied_cells}")
                
                # Example: Try to find path between first two free cells
                if len(free_cells) >= 2:
                    start = free_cells[0]
                    goal = free_cells[-1]
                    path = planner.find_simple_path(start, goal)
                    
                    if path:
                        print(f"\nExample path from {start} to {goal}:")
                        print(f"  Path: {path}")
                        print(f"  Length: {len(path)} cells")
                    else:
                        print(f"\nNo path available from {start} to {goal}")
                
                print("="*50 + "\n")
    
    finally:
        detector.cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    # Run the demo with example data
    demo_path_planning()
    
    # Uncomment to run with live camera
    # live_detection_with_path_planning()