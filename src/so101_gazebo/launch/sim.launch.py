"""Gazebo + SO-101 + ros2_control を起動する。

  ros2 launch so101_gazebo sim.launch.py            # GUI あり
  ros2 launch so101_gazebo sim.launch.py gui:=false # ヘッドレス（CPU の負荷を下げる）
"""
import os
import re

import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    gazebo_ros = get_package_share_directory('gazebo_ros')
    desc = get_package_share_directory('so101_description')
    gz = get_package_share_directory('so101_gazebo')

    gui = LaunchConfiguration('gui')
    # gazebo_ros2_control は URDF 全体を ROS の `--param` 引数として再パースするため、
    # コメント（`: ` や `#` を含むとパース失敗する）を取り除いてから渡す
    urdf = xacro.process_file(os.path.join(desc, 'urdf', 'so101.urdf.xacro')).toxml()
    robot_description = re.sub(r'<!--.*?-->', '', urdf, flags=re.S)
    # 長いと gazebo_ros2_control の引数パースが失敗することがあるので、インデントと改行も詰める
    robot_description = re.sub(r'>\s+<', '><', robot_description)
    # Gazebo Classic は package:// のメッシュを解決できない（描画されない）ので file:// に置き換える
    robot_description = robot_description.replace('package://so101_description/', f'file://{desc}/')

    # real_time_update_rate（sim の速さ）は CPU / GPU で変えるので、環境変数 SIM_REAL_TIME_UPDATE_RATE で上書きする
    # （compose.gpu.yaml で設定。未設定なら table.world の値＝CPU 用の 80）。値と理由は docs/sim速度設定.md
    world = os.path.join(gz, 'worlds', 'table.world')
    rate = os.environ.get('SIM_REAL_TIME_UPDATE_RATE')
    if rate:
        with open(world, encoding='utf-8') as f:
            sdf = re.sub(r'<real_time_update_rate>[^<]*</real_time_update_rate>',
                         f'<real_time_update_rate>{float(rate):g}</real_time_update_rate>', f.read())
        world = '/tmp/so101_table.world'
        with open(world, 'w', encoding='utf-8') as f:
            f.write(sdf)

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros, 'launch', 'gazebo.launch.py')),
        launch_arguments={
            'world': world,
            'gui': gui,
            'verbose': 'false',
        }.items())

    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}])

    # gzserver の起動が PulseAudio の接続待ちで 1 分ほどかかることがあるので、spawn の待ち時間を延ばす（既定 30 秒）
    spawn = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'so101', '-x', '0', '-y', '0', '-z', '0.0',
                   '-timeout', '120'],
        output='screen')

    def load(name):
        return ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', name], output='screen')

    scene_manager = Node(package='scene_manager', executable='scene_manager_node', output='screen',
                         parameters=[{'use_sim_time': True}])

    load_jsb = load('joint_state_broadcaster')
    load_arm = load('arm_controller')

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        gazebo,
        robot_state_publisher,
        spawn,
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[load_jsb])),
        RegisterEventHandler(OnProcessExit(target_action=load_jsb, on_exit=[load_arm])),
        RegisterEventHandler(OnProcessExit(target_action=load_arm, on_exit=[scene_manager])),
    ])
