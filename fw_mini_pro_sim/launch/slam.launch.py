"""Fairy point cloud and 2-D SLAM with Gazebo ideal odometry assistance."""
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable, RegisterEventHandler, EmitEvent
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node


def generate_launch_description():
    share=Path(get_package_share_directory('fw_mini_pro_sim'))
    server=ExecuteProcess(cmd=['ign','gazebo','-r','-s','--headless-rendering','-v','2',str(share/'worlds/slam_ground.sdf')],output='screen')
    adapter=Node(package='fw_mini_pro_sim',executable='simulation_adapter',parameters=[{'use_sim_time':True,'publish_map_tf':False,'pose_frame':'odom'}],remappings=[('/gnss_pose','/sim/ground_truth_pose')],output='screen')
    bridge=Node(package='ros_gz_bridge',executable='parameter_bridge',parameters=[{'config_file':str(share/'config/bridge_slam.yaml')}],output='screen')
    slam=Node(package='slam_toolbox',executable='async_slam_toolbox_node',name='slam_toolbox',parameters=[str(share/'config/slam.yaml'),
        {'transform_publish_period':ParameterValue(PythonExpression(["0.0 if '",LaunchConfiguration('fusion'),"'.lower() in ('true','1') else 0.02"]),value_type=float),
         'minimum_travel_distance':ParameterValue(PythonExpression(["0.0 if '",LaunchConfiguration('fusion'),"'.lower() in ('true','1') else 0.1"]),value_type=float),
         'minimum_travel_heading':ParameterValue(PythonExpression(["0.0 if '",LaunchConfiguration('fusion'),"'.lower() in ('true','1') else 0.1"]),value_type=float)}],
        remappings=[('pose','/slam_toolbox/pose')],output='screen')
    return LaunchDescription([
        DeclareLaunchArgument('fusion',default_value='false',description='Reserved for fusion.launch.py'),
        DeclareLaunchArgument('rtk',default_value='false'),
        DeclareLaunchArgument('rtk_params',default_value=str(share/'config/rtk_sim.yaml')),
        Node(package='tf2_ros',executable='static_transform_publisher',name='rtk_origin_tf',
             condition=IfCondition(PythonExpression(["'",LaunchConfiguration('rtk'),"'.lower() in ('true','1') and '",LaunchConfiguration('fusion'),"'.lower() not in ('true','1')"])),
             arguments=['--frame-id','odom','--child-frame-id','rtk_map'],
             parameters=[{'use_sim_time':True}]),
        Node(package='fw_mini_pro_sim',executable='rtk_simulator',
             condition=IfCondition(LaunchConfiguration('rtk')),
             parameters=[LaunchConfiguration('rtk_params'), {'use_sim_time':True, 'map_frame':'rtk_map'}],output='screen'),
        DeclareLaunchArgument('gui',default_value='true'),
        DeclareLaunchArgument('rviz',default_value='true'),
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH',str(share/'models')+os.pathsep+os.environ.get('IGN_GAZEBO_RESOURCE_PATH','')),
        *[RegisterEventHandler(OnProcessExit(target_action=p,on_exit=[EmitEvent(event=Shutdown(reason='SLAM process exited'))])) for p in (server,adapter,bridge,slam)],
        server,bridge,adapter,
        ExecuteProcess(cmd=['ign','gazebo','-g','--gui-config',str(share/'config/gui.config')],condition=IfCondition(LaunchConfiguration('gui')),output='screen'),
        Node(package='tf2_ros',executable='static_transform_publisher',name='lidar_mount_tf',arguments=['--x','0','--y','0','--z','0.483976069','--roll','0','--pitch','0','--yaw','0','--frame-id','base_link','--child-frame-id','rslidar'],parameters=[{'use_sim_time':True}]),
        Node(package='tf2_ros',executable='static_transform_publisher',name='lidar_optical_tf',arguments=['--x','0','--y','0','--z','0.035','--roll','0','--pitch','0','--yaw','0','--frame-id','rslidar','--child-frame-id','fairy_optical'],parameters=[{'use_sim_time':True}]),
        Node(package='pointcloud_to_laserscan',executable='pointcloud_to_laserscan_node',name='cloud_to_scan',remappings=[('cloud_in','/rslidar_points'),('scan','/scan')],parameters=[{'use_sim_time':True,'target_frame':'fairy_optical','transform_tolerance':.1,'min_height':-.06,'max_height':.06,'angle_min':-3.141592653589793,'angle_max':3.141592653589793,'angle_increment':.004363323129985824,'scan_time':.1,'range_min':.1,'range_max':20.0,'use_inf':True}],output='screen'),
        slam,
        Node(package='rviz2',executable='rviz2',arguments=['-d',str(share/'config/slam.rviz')],parameters=[{'use_sim_time':True}],condition=IfCondition(LaunchConfiguration('rviz')),output='screen'),
    ])
