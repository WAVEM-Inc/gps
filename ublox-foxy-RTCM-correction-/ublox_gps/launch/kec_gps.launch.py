import os
import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    ublox_gps_dir = get_package_share_directory('ublox_gps')
    ublox_gps_launch_file = os.path.join(velodyne_dir, 'launch', 'ublox_gps_node-launch.py')
    ublox_gps_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            ublox_gps_launch_file
        )
    )
    ntrip_client_dir = get_package_share_directory('ntrip_client')
    ntrip_client_launch_file = os.path.join(imu_dir, 'launch','ntrip_client_launch.py')
    ntrip_client_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            ntrip_client_launch_file
        )
    )
    ld = launch.LaunchDescription()

    ld.add_action(ublox_gps_launch)
    ld.add_action(ntrip_client_launch)
    return ld
