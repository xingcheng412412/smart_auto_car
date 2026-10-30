"""Launch isolated simulation; never includes the physical CAN driver."""
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, EmitEvent, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('fw_mini_pro_sim'))
    gui = LaunchConfiguration('gui')
    world = LaunchConfiguration('world')
    server = ExecuteProcess(cmd=['ign','gazebo','-r','-s','-v','2',world],output='screen')
    adapter = Node(package='fw_mini_pro_sim',executable='simulation_adapter',
                   parameters=[{'use_sim_time':True}],
                   remappings=[('/gnss_pose', PythonExpression(["'/sim/ground_truth_pose' if '", LaunchConfiguration('rtk'), "'.lower() in ('true', '1') else '/gnss_pose'"]))],output='screen')
    bridge = Node(package='ros_gz_bridge',executable='parameter_bridge',
                  parameters=[{'config_file':str(share/'config/bridge.yaml')}],output='screen')
    # All consumers stop together if the server, adapter or bridge exits.
    shutdown_handlers = [RegisterEventHandler(OnProcessExit(target_action=process,
        on_exit=[EmitEvent(event=Shutdown(reason='simulation process exited'))]))
        for process in (server,adapter,bridge)]
    return LaunchDescription([
        DeclareLaunchArgument('rtk',default_value='false',description='Enable simulated RTK outputs'),
        DeclareLaunchArgument('rtk_params',default_value=str(share/'config/rtk_sim.yaml')),
        Node(package='fw_mini_pro_sim',executable='rtk_simulator',
             condition=IfCondition(LaunchConfiguration('rtk')),
             parameters=[LaunchConfiguration('rtk_params'), {'use_sim_time':True}],output='screen'),
        DeclareLaunchArgument('gui',default_value='true',description='Open Gazebo GUI'),
        DeclareLaunchArgument('tracker',default_value='false',description='Start the existing tracker'),
        DeclareLaunchArgument('world',default_value=str(share/'worlds/test_ground.sdf')),
        DeclareLaunchArgument('path_file',default_value=str(share/'paths/straight.csv')),
        DeclareLaunchArgument('tracker_params',default_value=str(share/'config/tracker_sim.yaml')),
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH',str(share/'models')+os.pathsep+os.environ.get('IGN_GAZEBO_RESOURCE_PATH','')),
        *shutdown_handlers,
        server,
        ExecuteProcess(cmd=['ign','gazebo','-g','--gui-config',str(share/'config/gui.config')],condition=IfCondition(gui),output='screen'),
        bridge,adapter,
        Node(package='lawn_mower_tracker',executable='lawn_mower_tracker_node',
             name='lawn_mower_tracker',condition=IfCondition(LaunchConfiguration('tracker')),
             parameters=[LaunchConfiguration('tracker_params'),
                         {'use_sim_time':True,'path_file':LaunchConfiguration('path_file')}],output='screen'),
    ])
