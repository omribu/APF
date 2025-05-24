#!/usr/bin/env python3

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

class RobotPlotter:
    def __init__(self):
        pass
    
    def plot_configuration_space_3d(self, x_history, y_history, theta_history, time_history):
        """
        Plot robot configuration space in 3D (x, y, theta)
        """
        fig = plt.figure(figsize=(12, 10))
        
        ax1 = fig.add_subplot(221, projection='3d')
        
        # Plot trajectory with color gradient based on time
        scatter = ax1.scatter(x_history, y_history, theta_history, 
                             c=time_history, cmap='viridis', s=20, alpha=0.7)
        
        # plot the trajectory line
        ax1.plot(x_history, y_history, theta_history, 
                color='red', linewidth=1.5, alpha=0.8)
        
        # Mark start and end points
        ax1.scatter([x_history[0]], [y_history[0]], [theta_history[0]], 
                   color='green', s=100, marker='o', label='Start')
        ax1.scatter([x_history[-1]], [y_history[-1]], [theta_history[-1]], 
                   color='red', s=100, marker='s', label='End')
        
        ax1.set_xlabel('X Position (m)')
        ax1.set_ylabel('Y Position (m)')
        ax1.set_zlabel('Theta Orientation (rad)')
        ax1.set_title('Configuration Space (C-Space)\n3D Robot Trajectory')
        ax1.legend()
        
        plt.colorbar(scatter, ax=ax1, shrink=0.8, label='Time (s)')
        
        # 2D Configuration projections
        ax2 = fig.add_subplot(222)
        ax2.plot(x_history, y_history, 'b-', linewidth=2, alpha=0.7)
        ax2.scatter([x_history[0]], [y_history[0]], color='green', s=100, marker='o', label='Start')
        ax2.scatter([x_history[-1]], [y_history[-1]], color='red', s=100, marker='s', label='End')
        ax2.set_xlabel('X Position (m)')
        ax2.set_ylabel('Y Position (m)')
        ax2.set_title('Task Space (X-Y Plane)')
        ax2.grid(True, alpha=0.3)
        ax2.legend()
        ax2.axis('equal')
        
        # Time vs X position
        ax3 = fig.add_subplot(223)
        ax3.plot(time_history, x_history, 'r-', linewidth=2)
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('X Position (m)')
        ax3.set_title('X Position vs Time')
        ax3.grid(True, alpha=0.3)
        
        # Time vs Y position
        ax4 = fig.add_subplot(224)
        ax4.plot(time_history, y_history, 'g-', linewidth=2)
        ax4.set_xlabel('Time (s)')
        ax4.set_ylabel('Y Position (m)')
        ax4.set_title('Y Position vs Time')
        ax4.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.show()
    
    def plot_orientation_analysis(self, time_history, theta_history, x_history, y_history):
        """
        Detailed orientation analysis
        """
        fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(15, 10))
        
        # Orientation vs Time
        ax1.plot(time_history, np.degrees(theta_history), 'purple', linewidth=2)
        ax1.set_xlabel('Time (s)')
        ax1.set_ylabel('Orientation (degrees)')
        ax1.set_title('Robot Orientation vs Time')
        ax1.grid(True, alpha=0.3)
        
        # Angular velocity (approximate)
        if len(theta_history) > 1:
            dt = np.diff(time_history)
            dtheta = np.diff(theta_history)

            dtheta = np.array([self.normalize_angle(angle) for angle in dtheta])
            angular_velocity = dtheta / dt
            
            ax2.plot(time_history[1:], np.degrees(angular_velocity), 'orange', linewidth=2)
            ax2.set_xlabel('Time (s)')
            ax2.set_ylabel('Angular Velocity (deg/s)')
            ax2.set_title('Robot Angular Velocity vs Time')
            ax2.grid(True, alpha=0.3)
        
        # Position trajectory with orientation 
        ax3.plot(x_history, y_history, 'b-', linewidth=2, alpha=0.7, label='Trajectory')
        
        # Add orientation arrows
        step = max(1, len(x_history) // 15)  
        for i in range(0, len(x_history), step):
            x, y, theta = x_history[i], y_history[i], theta_history[i]
            dx = 0.1 * np.cos(theta)
            dy = 0.1 * np.sin(theta)
            ax3.arrow(x, y, dx, dy, head_width=0.03, head_length=0.02, 
                     fc='red', ec='red', alpha=0.8)
        
        ax3.scatter([x_history[0]], [y_history[0]], color='green', s=100, marker='o', label='Start')
        ax3.scatter([x_history[-1]], [y_history[-1]], color='red', s=100, marker='s', label='End')
        ax3.set_xlabel('X Position (m)')
        ax3.set_ylabel('Y Position (m)')
        ax3.set_title('Task Space with Orientation Vectors')
        ax3.grid(True, alpha=0.3)
        ax3.legend()
        ax3.axis('equal')
        
        # Polar plot of orientation
        ax4 = plt.subplot(224, projection='polar')
        ax4.plot(theta_history, np.ones_like(theta_history), 'mo', markersize=4, alpha=0.6)
        ax4.set_title('Orientation Distribution\n(Polar Plot)')
        ax4.set_ylim(0, 1.5)
        
        plt.tight_layout()
        plt.show()
    
    def plot_velocity_analysis(self, time_history, x_history, y_history):
        """
        Velocity and acceleration analysis
        """
        if len(time_history) < 2:
            print("Not enough data points for velocity analysis")
            return
            
        # Calculate velocities
        dt = np.diff(time_history)
        dx = np.diff(x_history)
        dy = np.diff(y_history)
        
        vx = dx / dt
        vy = dy / dt
        v_magnitude = np.sqrt(vx**2 + vy**2)
        
        fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(15, 10))
        
        # Linear velocity components
        ax1.plot(time_history[1:], vx, 'r-', linewidth=2, label='Vx')
        ax1.plot(time_history[1:], vy, 'g-', linewidth=2, label='Vy')
        ax1.plot(time_history[1:], v_magnitude, 'b-', linewidth=2, label='|V|')
        ax1.set_xlabel('Time (s)')
        ax1.set_ylabel('Velocity (m/s)')
        ax1.set_title('Linear Velocity Components')
        ax1.grid(True, alpha=0.3)
        ax1.legend()
        
        # Speed over trajectory
        ax2.scatter(x_history[1:], y_history[1:], c=v_magnitude, cmap='plasma', s=30)
        ax2.set_xlabel('X Position (m)')
        ax2.set_ylabel('Y Position (m)')
        ax2.set_title('Speed Along Trajectory')
        ax2.axis('equal')
        cb = plt.colorbar(ax2.collections[0], ax=ax2)
        cb.set_label('Speed (m/s)')
        
        # Acceleration 
        if len(time_history) > 2:
            dt2 = dt[1:]
            dvx = np.diff(vx)
            dvy = np.diff(vy)
            ax = dvx / dt2
            ay = dvy / dt2
            a_magnitude = np.sqrt(ax**2 + ay**2)
            
            ax3.plot(time_history[2:], ax, 'r-', linewidth=2, label='Ax')
            ax3.plot(time_history[2:], ay, 'g-', linewidth=2, label='Ay')
            ax3.plot(time_history[2:], a_magnitude, 'b-', linewidth=2, label='|A|')
            ax3.set_xlabel('Time (s)')
            ax3.set_ylabel('Acceleration (m/s²)')
            ax3.set_title('Linear Acceleration Components')
            ax3.grid(True, alpha=0.3)
            ax3.legend()
        
        # Distance
        distances = np.sqrt(dx**2 + dy**2)
        cumulative_distance = np.cumsum(distances)
        
        ax4.plot(time_history[1:], cumulative_distance, 'purple', linewidth=2)
        ax4.set_xlabel('Time (s)')
        ax4.set_ylabel('Distance Traveled (m)')
        ax4.set_title('Cumulative Distance Traveled')
        ax4.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.show()
    
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > np.pi:
            angle -= 2.0 * np.pi
        while angle < -np.pi:
            angle += 2.0 * np.pi
        return angle
    
    def plot_complete_analysis(self, time_history, x_history, y_history, theta_history, 
                              goal=None, obstacles_dist=None):
        """
        Complete robot analysis with all plots
        """
        print("Generating complete robot trajectory analysis...")
        
        # 1. Configuration Space 
        self.plot_configuration_space_3d(x_history, y_history, theta_history, time_history)
        
        # 2. Orientation 
        self.plot_orientation_analysis(time_history, theta_history, x_history, y_history)
        
        # 3. Velocity 
        self.plot_velocity_analysis(time_history, x_history, y_history)
        
        # 4. Goal and Distance 
        if goal is not None:
            self.plot_goal_analysis(time_history, x_history, y_history, goal, obstacles_dist)
    
    def plot_goal_analysis(self, time_history, x_history, y_history, goal, obstacles_dist=None):
        """
        Analysis related to goal reaching and obstacle avoidance
        """
        # Calculate distance to goal over time
        goal_distances = [np.sqrt((x - goal[0])**2 + (y - goal[1])**2) 
                         for x, y in zip(x_history, y_history)]
        
        fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(15, 10))
        
        # Distance to goal over time
        ax1.plot(time_history, goal_distances, 'blue', linewidth=2)
        ax1.set_xlabel('Time (s)')
        ax1.set_ylabel('Distance to Goal (m)')
        ax1.set_title('Distance to Goal vs Time')
        ax1.grid(True, alpha=0.3)
        
        # Trajectory with goal
        ax2.plot(x_history, y_history, 'b-', linewidth=2, label='Robot Path')
        ax2.plot(goal[0], goal[1], 'ro', markersize=15, label='Goal')
        ax2.scatter([x_history[0]], [y_history[0]], color='green', s=100, marker='o', label='Start')
        ax2.scatter([x_history[-1]], [y_history[-1]], color='red', s=100, marker='s', label='End')
        ax2.set_xlabel('X Position (m)')
        ax2.set_ylabel('Y Position (m)')
        ax2.set_title('Robot Trajectory and Goal')
        ax2.grid(True, alpha=0.3)
        ax2.legend()
        ax2.axis('equal')
        
        # Obstacle distances 
        if obstacles_dist is not None and len(obstacles_dist) > 0:
            time_obs = time_history[:len(obstacles_dist)]
            ax3.plot(time_obs, obstacles_dist, 'red', linewidth=2)
            ax3.axhline(y=0.3, color='orange', linestyle='--', label='Safety Threshold')
            ax3.set_xlabel('Time (s)')
            ax3.set_ylabel('Distance to Nearest Obstacle (m)')
            ax3.set_title('Distance to Obstacles vs Time')
            ax3.grid(True, alpha=0.3)
            ax3.legend()
        
        final_distance = goal_distances[-1]
        total_time = time_history[-1] - time_history[0]
        path_length = sum(np.sqrt((x_history[i+1] - x_history[i])**2 + 
                                 (y_history[i+1] - y_history[i])**2) 
                         for i in range(len(x_history)-1))
        direct_distance = np.sqrt((x_history[-1] - x_history[0])**2 + 
                                 (y_history[-1] - y_history[0])**2)
        efficiency = direct_distance / path_length if path_length > 0 else 0
        
        metrics_text = f"""Performance Metrics:
                           Final Distance to Goal: {final_distance:.3f} m
                           Total Time: {total_time:.2f} s
                           Path Length: {path_length:.3f} m
                           Direct Distance: {direct_distance:.3f} m
                           Path Efficiency: {efficiency:.3f}
                           Avg Speed: {path_length/total_time:.3f} m/s"""
        
        ax4.text(0.1, 0.5, metrics_text, transform=ax4.transAxes, fontsize=12,
                verticalalignment='center', bbox=dict(boxstyle='round', facecolor='lightblue'))
        ax4.set_xlim(0, 1)
        ax4.set_ylim(0, 1)
        ax4.axis('off')
        ax4.set_title('Performance Summary')
        
        plt.tight_layout()
        plt.show()

def plot_robot_data(controller_instance):
    """
    Function to call from your APF_controller class
    """
    plotter = RobotPlotter()
    
    if len(controller_instance.time_history) > 1:
        plotter.plot_complete_analysis(
            controller_instance.time_history,
            controller_instance.x_position_history,
            controller_instance.y_position_history,
            controller_instance.orientation_history,
            controller_instance.goal,
            controller_instance.dist if hasattr(controller_instance, 'dist') else None
        )
    else:
        print("Not enough data points for analysis")

