import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node



def generate_launch_description():

    package_name='lidar_plane_filtering'

    robot_state_publisher = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )
    
    imu_data_converter_node = Node(
        package=package_name,
        executable='imu_data_converter',
        name='imu_data_converter'
    )
    
    # imu_complementary_filter node
    imu_complementary_filter_node = Node(
        package='imu_complementary_filter',
        executable='complementary_filter_node',
        name='complementary_filter_node'
        # Add any additional arguments or parameters here
    ) 

    orientation_tf_node = Node(
        package=package_name,
        executable='orientation_tf',
        name='orientation_tf'
    )

    #lidar_tf_node = Node(
        #package=package_name,
        #executable='lidar_tf',
        #name='lidar_tf'
    #)
    
    plane_filter_node = Node(
        package=package_name,
        executable='plane_filter',
        name='plane_filter'
    )

    # RViz2 node
    rviz_config_file = os.path.join(
        get_package_share_directory(package_name), 'config', 'sensor_stack.rviz'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    # Launch them all!
    return LaunchDescription([
        robot_state_publisher,
        imu_data_converter_node,
        orientation_tf_node,
        #lidar_tf_node,
        imu_complementary_filter_node,
        rviz_node
    ])