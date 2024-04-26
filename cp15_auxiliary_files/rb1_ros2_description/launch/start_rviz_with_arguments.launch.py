from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from launch_ros.actions import Node

from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo


def generate_launch_description():

    #args for rviz
    package_description_arg = DeclareLaunchArgument('package_description', default_value="my_package")
    rviz_config_file_name_arg = DeclareLaunchArgument('rviz_config_file_name', default_value='config1.rviz')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time',default_value='True')
    #params for rviz
    package_description = LaunchConfiguration('package_description')
    rviz_config_file_name = LaunchConfiguration('rviz_config_file_name')
    use_sim_time = LaunchConfiguration('use_sim_time')

    global_path_to_rviz_file = PathJoinSubstitution([
        FindPackageShare(package_description),
        'rviz_config',
        rviz_config_file_name
    ])

    message_path = LogInfo(
        msg=global_path_to_rviz_file)

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        name='rviz_node',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', global_path_to_rviz_file])

    # create and return the launch description object
    return LaunchDescription(
        [
            package_description_arg,
            rviz_config_file_name_arg,
            use_sim_time_arg,
            rviz_node,
            message_path
        ]
    )
