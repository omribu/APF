#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, TwistStamped
from sensor_msgs.msg import LaserScan
import numpy as np  
from nav_msgs.msg import Odometry
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from tf_transformations import euler_from_quaternion
from rcl_interfaces.msg import SetParametersResult
from visualization_msgs.msg import Marker
from rclpy.time import Time
from robot_plotter import RobotPlotter, plot_robot_data


class APF_controller(Node):
    def __init__(self):
        super().__init__('apf_navigation_controller')

        self.goal = None
        self.lidar_data = None
        self.dist = []
        self.dist_t = []

        self.time_history = []
        self.x_position_history = []
        self.y_position_history = []
        self.orientation_history = []
        self.start_time = None 

        # Declare Parmeters
        self.declare_parameter('max_linear_vel', 0.2)
        self.declare_parameter('goal', [4.0, -2.0])
        self.declare_parameter('kp', 0.1)
        self.declare_parameter('eta', 0.1)
        self.declare_parameter('repulsion_radius', 0.5)

        self.kp = self.get_parameter('kp').value
        self.eta = self.get_parameter('eta').value
        self.repulsion_radius = self.get_parameter('repulsion_radius').value
        self.max_linear_vel = self.get_parameter('max_linear_vel').value
        self.goal = np.array(self.get_parameter('goal').value)
        self.get_logger().info(f'goal: {self.goal}')


        # Subscribers
        self.lidar_subscriber_ = self.create_subscription(LaserScan, "lidar", self.lidar_callback, 10)
        self.robot_pose_subscriber_ = self.create_subscription(Odometry, "/apf_controller/odom", self.robot_pose_callback, 10)
        
        # Publishers
        self.publisher_ = self.create_publisher(TwistStamped, "/apf_controller/cmd_vel", 10)
        self.publisher = self.create_publisher(Marker, 'visualization_marker', 10)


        # Flag to check the robots position
        self.robot_pose_ready = False          #None
        self.robot_orientation = 0.0
        self.robot_position = np.array([0.0, 0.0])

        self.ctrl_loop_timer = self.create_timer(0.01, self.control_loop)


    def robot_pose_callback(self, msg):
        
        # Handling Time
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        if self.start_time is None:
            self.start_time = current_time

        normalized_time = current_time - self.start_time
        self.time_ = normalized_time        


        self.robot_position = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
        self.get_logger().info(f"x={msg.pose.pose.position.x} y={msg.pose.pose.position.y}")
        _, _, self.robot_orientation = euler_from_quaternion([msg.pose.pose.orientation.x,
                                                              msg.pose.pose.orientation.y,
                                                              msg.pose.pose.orientation.z,
                                                              msg.pose.pose.orientation.w ])
        
        # Data of configuration for ploting
        self.time_history.append(normalized_time)
        self.x_position_history.append(msg.pose.pose.position.x)
        self.y_position_history.append(msg.pose.pose.position.y)
        self.orientation_history.append(self.robot_orientation)
        self.robot_pose_ready = True

    
    def lidar_callback(self, msg):
        self.lidar_data = msg


    def attractive_potential(self):
        return self.kp * (self.goal - self.robot_position)
    
    def repulsive_potential(self):
        if self.lidar_data is None:
            return np.array([0.0, 0.0])
        
        force = np.array([0.0, 0.0])
        angle_min = self.lidar_data.angle_min
        angle_increment = self.lidar_data.angle_increment

        for i, distance in enumerate(self.lidar_data.ranges):
            if distance < self.repulsion_radius and distance > 0.1:  
                angle = angle_min + i * angle_increment + self.robot_orientation  # Transform to world frame
            
                # Calculate obstacle position in world frame
                obstacle_world_pos = self.robot_position + np.array([distance * np.cos(angle), 
                                                                   distance * np.sin(angle)])

                # Repulsive force direction (away from obstacle)
                direction = self.robot_position - obstacle_world_pos
                direction_norm = np.linalg.norm(direction)

                if direction_norm > 0:
                    force += self.eta * (1.0/distance - 1.0/self.repulsion_radius) * (1.0/(distance**2)) * (direction/direction_norm)
    
        return force
    
    def get_dist_from_obstacles(self):
        if self.lidar_data is None:
            return 0
        return min(self.lidar_data.ranges)
    

    def normalize_angle(self, angle):
        while angle > np.pi:
            angle -= 2.0 * np.pi
        while angle < -np.pi:
            angle += 2.0 * np.pi
        return angle


    def control_loop(self):

        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "goal_marker"
        marker.id = 0
        marker.type = Marker.ARROW # or Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = self.goal[0]
        marker.pose.position.y = self.goal[1]
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0  # No rotation

        marker.scale.x = 0.3
        marker.scale.y = 0.1
        marker.scale.z = 0.1

        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0


        attractive = self.attractive_potential()
        repulsive = self.repulsive_potential()
        resultant_force = attractive + repulsive


        self.get_logger().info(f'attractive force: {attractive}')
        self.get_logger().info(f'repulsive force: {repulsive}')
        self.get_logger().info(f'resultant force: {resultant_force}')

        # Calculate the velocity command
        angular_direction = math.atan2(resultant_force[1], resultant_force[0])

        # angular_error = angular_direction - self.robot_orientation
        angular_error = self.normalize_angle(angular_direction - self.robot_orientation)
        
        linear_error = math.sqrt(resultant_force[0]*resultant_force[0] + resultant_force[1]*resultant_force[1])
        target_dist = math.sqrt((self.goal[0] - self.robot_position[0])**2 + (self.goal[1] - self.robot_position[1])**2)
        

        dist = self.get_dist_from_obstacles()
        if dist == 0: 
            dist = 8.0 # Max lidar allowed values
        self.dist.append( dist )

        self.get_logger().info(f'angular_error: {angular_error}, linear_error: {linear_error}, target_dist: {target_dist}')

        velocity = TwistStamped()
        velocity.header.stamp = self.get_clock().now().to_msg()

        min_distance = self.get_dist_from_obstacles()
        if min_distance < 0.3:  # 30cm safety margin
            velocity.twist.linear.x = 0.0
            velocity.twist.angular.z = 0.5 if angular_error > 0 else -0.5  # Turn in place
            self.get_logger().warn(f"Emergency stop! Obstacle at {min_distance:.2f}m")



        elif( target_dist > 0.05): 

            if abs(angular_error) < 0.5:  # About 28 degrees
                velocity.twist.linear.x = min(linear_error * self.max_linear_vel, self.max_linear_vel)
            else:
                velocity.twist.linear.x = 0.1  # Still move slowly while turning

            velocity.twist.angular.z = angular_error
        else:
            velocity.twist.linear.x = 0.0
            velocity.twist.angular.z = 0.0
            
            print("Goal reached! Generating comprehensive analysis...")
            
            plotter = RobotPlotter()
            
            if len(self.time_history) > 1:
                # Generate all analysis plots
                plotter.plot_complete_analysis(
                    self.time_history,
                    self.x_position_history,
                    self.y_position_history,
                    self.orientation_history,
                    self.goal,
                    self.dist  # obstacle distances
                )
            else:
                print("Not enough data points for analysis")

        self.publisher_.publish(velocity)
        # Publish Goal marker
        self.publisher.publish(marker)


    def generate_plots(self):
        """Generate plots on demand"""
        if len(self.time_history) > 1:
            plotter = RobotPlotter()
            plotter.plot_complete_analysis(
                self.time_history,
                self.x_position_history,
                self.y_position_history,
                self.orientation_history,
                self.goal,
                self.dist
            )
        else:
            self.get_logger().info("Not enough data points for analysis")


def main(args=None):
    rclpy.init(args=args)
    controller = APF_controller()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()