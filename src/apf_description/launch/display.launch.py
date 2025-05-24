from launch import LaunchDescription
from launch_ros.actions import Node
import launch_ros.descriptions
from launch.substitutions import Command
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import OpaqueFunction
from launch_ros.parameter_descriptions import ParameterValue
import os

def generate_launch_description():

    apf_description_dir = get_package_share_directory("apf_description")

    model_arg = DeclareLaunchArgument(
        name = "model",
        default_value=os.path.join(apf_description_dir, "urdf", "apf.urdf.xacro"),
        description="Absolute path to robot URDF file"
    )

    robot_description =  ParameterValue(Command(["xacro ", LaunchConfiguration("model")]), value_type=str)
                                                      
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',        
        parameters=[{
            'robot_description': robot_description}]
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen')
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=["-d", os.path.join(get_package_share_directory("apf_description"), "rviz", "display.rviz")]
        )

    return LaunchDescription ( [
        model_arg,
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz_node
     ])
