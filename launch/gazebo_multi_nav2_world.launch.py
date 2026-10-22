#!/usr/bin/env python3

import os
import tempfile
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace, SetRemap
from launch.conditions import IfCondition

def generate_launch_description():
    ld = LaunchDescription()

    # 1. CONFIGURACIÓN DE LA FLOTA
    robots = [
        {'name': 'tb1', 'x_pose': '-1.5', 'y_pose': '2.0'},
        {'name': 'tb2', 'x_pose': '-1.5', 'y_pose': '0.5'} #x_pose': '1.5', 'y_pose': '3.5'}
       # {'name': 'tb3', 'x_pose': '-2.5', 'y_pose': '1.5'} # Nuevo integrante

    ]

    TURTLEBOT3_MODEL = 'burger'
    MY_PACKAGE = 'turtlebot3_multi_robot'
    OFFICIAL_GAZEBO_PKG = 'turtlebot3_gazebo'
    
    my_pkg_dir = get_package_share_directory(MY_PACKAGE)
    gazebo_pkg_dir = get_package_share_directory(OFFICIAL_GAZEBO_PKG)
    nav_launch_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    base_params_path = os.path.join(my_pkg_dir, 'params', 'nav2_params.yaml')

    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='true'))
    ld.add_action(DeclareLaunchArgument('enable_rviz', default_value='true'))

    # 2. MUNDO GAZEBO
    world = os.path.join(gazebo_pkg_dir, 'worlds', 'turtlebot3_house.world')
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world}.items()
    )
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzclient.launch.py'))
    )
    ld.add_action(gzserver)
    ld.add_action(gzclient)

    # 3. BUCLE DE ROBOTS
    spawn_delay = 0.0

    for i, robot in enumerate(robots):
        namespace = robot['name']
        # Definimos quién es el vecino para la coordinación
        target_neighbor = robots[(i + 1) % len(robots)]['name']
        
        # LEER URDF
        urdf_path = os.path.join(my_pkg_dir, 'urdf', 'turtlebot3_' + TURTLEBOT3_MODEL + '.urdf')
        with open(urdf_path, 'r') as infp:
            robot_desc = infp.read()

        # REEMPLAZOS URDF
        robot_desc = robot_desc.replace('${namespace}', '') 
        robot_desc = robot_desc.replace('<frame_name>base_scan</frame_name>', f'<frame_name>{namespace}/base_scan</frame_name>')
        robot_desc = robot_desc.replace('<odometry_frame>odom</odometry_frame>', f'<odometry_frame>{namespace}/odom</odometry_frame>')
        robot_desc = robot_desc.replace('<robot_base_frame>base_footprint</robot_base_frame>', f'<robot_base_frame>{namespace}/base_footprint</robot_base_frame>')
        robot_desc = robot_desc.replace('<frame_name>imu_link</frame_name>', f'<frame_name>{namespace}/imu_link</frame_name>')

        # Nodos RSP y Spawn
        rsp = Node(
            package='robot_state_publisher',
            namespace=namespace,
            executable='robot_state_publisher',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc, 'frame_prefix': namespace + '/'}]
        )

        spawn = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', f'{namespace}/robot_description', '-entity', namespace, '-robot_namespace', namespace, '-x', robot['x_pose'], '-y', robot['y_pose'], '-z', '0.01', '-unpause'],
        )

        with open(base_params_path, 'r') as f:
            params_txt = f.read()
        params_txt = params_txt.replace('__NS__', namespace)
        tmp = tempfile.NamedTemporaryFile(mode='w', delete=False)
        tmp.write(params_txt)
        tmp.close()

        rviz_template_path = os.path.join(my_pkg_dir, 'rviz', 'multi_agent.rviz')
        with open(rviz_template_path, 'r') as f_rviz:
            rviz_txt = f_rviz.read()
            
        rviz_txt = rviz_txt.replace('__NS__', namespace)
        
        # Ojo: añadimos el sufijo .rviz para que RViz lo reconozca correctamente
        tmp_rviz = tempfile.NamedTemporaryFile(mode='w', delete=False, suffix='.rviz') 
        tmp_rviz.write(rviz_txt)
        tmp_rviz.close()
        
        # SLAM
        slam_node = Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            namespace=namespace,
            parameters=[tmp.name, {'use_sim_time': use_sim_time}],
            remappings=[('/tf', '/tf'), ('/tf_static', '/tf_static'), ('/scan', 'scan'), ('/map', 'map')]
        )

        # NAV2
        nav2_group = GroupAction(
            actions=[
                PushRosNamespace(namespace),
                SetRemap(src='/tf', dst='/tf'),
                SetRemap(src='/tf_static', dst='/tf_static'),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(os.path.join(nav_launch_dir, 'navigation_launch.py')),
                    launch_arguments={'use_sim_time': use_sim_time, 'params_file': tmp.name, 'autostart': 'true', 'use_namespace': 'True'}.items()
                ),
            ]
        )

        # 4. NODOS DE AUTONOMÍA (Tus archivos nuevos)
        exploration_node = Node(
            package='fronteras',
            executable='exploracion_multi',
            namespace=namespace,
            parameters=[{
                'use_sim_time': use_sim_time,
                'target_robot': 'tb2' if namespace == 'tb1' else 'tb1'
            }],
            remappings=[('/tf', '/tf'), ('/tf_static', '/tf_static')]
        )
        coordination_node = Node(
            package='turtlebot3_multi_robot',
            executable='coordinacion_multi.py',
            namespace=namespace,
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'namespace': namespace,
                'target_robot': target_neighbor,
                'comm_range': 10.0 # <--- Radio de umbral critico Vicsek
            }]
        )

        # RVIZ
        rviz = Node(
            package='rviz2',
            executable='rviz2',
            name=f'rviz2_node_{namespace}', 
            namespace=namespace,
            arguments=['-d', tmp_rviz.name],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(LaunchConfiguration('enable_rviz'))
        )

        # Combo con retraso (SLAM + NAV2 + EXPLORACIÓN)
        delayed_actions = TimerAction(
            period=10.0 + spawn_delay,
            actions=[slam_node, nav2_group, exploration_node, coordination_node]
        )

        ld.add_action(rsp)
        ld.add_action(spawn)
        ld.add_action(delayed_actions)
        ld.add_action(rviz)
        
        spawn_delay += 5.0 

    return ld