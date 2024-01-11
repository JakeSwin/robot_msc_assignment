import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

# TODO Add in simple potholes world bool argumemnt

def generate_launch_description():
  world_file_name = 'potholes.world'
  map_file_name = 'potholes_20mm.yaml'
  params_file_name = 'nav2_params.yaml'

  pkg_assignment_template = FindPackageShare(package='assignment_template').find('assignment_template')
  pkg_limo_gazebosim = FindPackageShare(package='limo_gazebosim').find('limo_gazebosim')
  pkg_limo_navigation = FindPackageShare(package='limo_navigation').find('limo_navigation')

  start_limo_gazebo_diff = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(pkg_limo_gazebosim, 'launch', 'limo_gazebo_diff.launch.py')),
    launch_arguments={'world': os.path.join(pkg_assignment_template, "worlds", world_file_name)}.items()
  )

  start_limo_navigation = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(pkg_limo_navigation, 'launch', 'limo_navigation.launch.py')),
    launch_arguments={'map': os.path.join(pkg_assignment_template, "maps", map_file_name), 'params_file': os.path.join(pkg_assignment_template, "params", params_file_name), 'use_sim_time': 'true'}.items()
  )

  ld = LaunchDescription()

  ld.add_action(start_limo_gazebo_diff)
  ld.add_action(start_limo_navigation)

  return ld