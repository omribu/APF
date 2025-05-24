from launch import LaunchDescription
from launch_ros.actions import Node
import launch_ros.descriptions
from launch.substitutions import Command
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.actions import OpaqueFunction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.descriptions import ParameterValue
from pathlib import Path
import os


def generate_launch_description():

    apfbot_description_dir = get_package_share_directory("apf_description")
    world_file_path = os.path.join(apfbot_description_dir, "models", "apf_world.sdf")

    use_control = LaunchConfiguration("use_control")
    use_control_arg = DeclareLaunchArgument(
        "use_control",
        default_value="False"
    )

    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(apfbot_description_dir, "urdf", "apf.urdf.xacro"),
        description="Absolute path to robot URDF file"

    )

    robot_description =  ParameterValue(Command(["xacro ", LaunchConfiguration("model")]), value_type=str)

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )

    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[
            str(Path(apfbot_description_dir).parent.resolve())
            ],
    )

    gazebo = IncludeLaunchDescription(PythonLaunchDescriptionSource([
        os.path.join(
            get_package_share_directory("ros_gz_sim"), "launch"), "/gz_sim.launch.py"]),
            launch_arguments=[
                ("gz_args", [" -v 4", " -r", " ", world_file_path])   # " empty.sdf"        #  world_file_path
            ]
        )

    # Spawn
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', '/robot_description',
                   '-name', 'apfbot',
                       #    '-x', '0',
                       #    '-y', '0',
                       #    '-z', '0.1',
                       #    '-r', '0',
                       #    '-p', '0',
                       #    '-Y', '0',
                  ])

    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=['lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                    '/lidar/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
                    'cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                    '/model/diff_drive/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry'],
        output='screen',
        condition= IfCondition(use_control)
        )    

    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            'lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/lidar/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked'],
        output='screen',
        condition= UnlessCondition(use_control)
        )



    return LaunchDescription ( [
        use_control_arg,
        model_arg,
        robot_state_publisher,
        gazebo_resource_path,
        gazebo,
        gz_spawn_entity,
        gz_ros2_bridge,
     ])