#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from microbot_detector_ros2.msg import OccupancyGrid
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class GridVisualizer(Node):
    def __init__(self):
        super().__init__('grid_visualizer')
        
        # Subscribe to both grids
        self.create_subscription(OccupancyGrid, '/microbot/occupancy/coarse', 
                                self.coarse_callback, 10)
        self.create_subscription(OccupancyGrid, '/microbot/occupancy/fine', 
                                self.fine_callback, 10)
        
        self.coarse_grid = None
        self.fine_grid = None
        
        # Setup matplotlib
        self.fig, (self.ax1, self.ax2) = plt.subplots(1, 2, figsize=(12, 5))
        self.get_logger().info("Grid Visualizer started!")
        
    def coarse_callback(self, msg):
        self.coarse_grid = np.array(msg.data).reshape(msg.grid_rows, msg.grid_cols)
        self.update_plot()
        
    def fine_callback(self, msg):
        self.fine_grid = np.array(msg.data).reshape(msg.grid_rows, msg.grid_cols)
        self.update_plot()
    
    def update_plot(self):
        if self.coarse_grid is not None:
            self.ax1.clear()
            im1 = self.ax1.imshow(self.coarse_grid, cmap='hot', interpolation='nearest')
            self.ax1.set_title(f'Coarse Grid (3x3)\nMax: {self.coarse_grid.max()}')
            
            # Add numbers to cells
            for i in range(self.coarse_grid.shape[0]):
                for j in range(self.coarse_grid.shape[1]):
                    self.ax1.text(j, i, str(self.coarse_grid[i, j]), 
                                ha="center", va="center", color="white", fontsize=20)
        
        if self.fine_grid is not None:
            self.ax2.clear()
            im2 = self.ax2.imshow(self.fine_grid, cmap='hot', interpolation='nearest')
            self.ax2.set_title(f'Fine Grid (9x9)\nMax: {self.fine_grid.max()}')
        
        plt.pause(0.01)

def main():
    rclpy.init()
    node = GridVisualizer()
    
    plt.ion()
    plt.show()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
