from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.substitutions import FindPackageShare
import time
from launch.actions import DeclareLaunchArgument
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    apf_control_dir = get_package_share_directory('apf_control')
    apf_description_dir = get_package_share_directory("apf_description")   

    conf_file_path = 'config/apf.yaml'

    config_file = PathJoinSubstitution([
        get_package_share_directory('apf_control'),	
        conf_file_path
    ])


    # DEBUG

    # Debug: Print the config file path
    print(f"Config file path: {config_file}")
    
    # Check if file exists
    full_path = os.path.join(get_package_share_directory('apf_control'), "config", conf_file_path)
    print(f"Full config path: {full_path}")
    print(f"File exists: {os.path.exists(full_path)}")


    #   DISPLAY IN RVIZ

    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(apf_description_dir, "urdf", "apf.urdf.xacro"),
        description="Absolute path to robot URDF file")
    
    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]), value_type=str)
    
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=["-d", os.path.join(get_package_share_directory("apf_description"), "rviz", "display_apf.rviz")],
        )


    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager"
        ],
    )

    wheel_controller_spwaner =  Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "apf_controller",
            "--controller-manager",
            "/controller_manager"
        ],
    )

    apf_control = Node(
            package='apf_control',
            executable='apf_controller.py',
            name='apf_navigation_controller',
            output='screen',
            parameters=[
                config_file
            ]
        )
    
    
    
    
    return LaunchDescription([
        
        model_arg,
        robot_state_publisher,
        rviz_node,
        joint_state_broadcaster_spawner,
        wheel_controller_spwaner,
        apf_control, 
    ])