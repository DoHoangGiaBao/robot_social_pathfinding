import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg = get_package_share_directory('my_robot_description')
    pkg_share = pkg  # same reference, no need to call twice

    # 1. Process the xacro into a URDF string
    xacro_file = os.path.join(pkg, 'robot_description', 'robot.urdf.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()
    rviz_config_file = os.path.join(pkg, 'rviz', 'my_robot.rviz')

    # -------------------------------------------------------------------------
    # 2. Start Gazebo — launched first so it begins publishing /clock ASAP
    # -------------------------------------------------------------------------
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', os.path.join(pkg, 'worlds', 'maze_world.sdf')],
        output='screen'
    )

    # -------------------------------------------------------------------------
    # 3. Clock bridge — started immediately after Gazebo so /clock reaches ROS
    #    before RSP tries to publish fixed-joint transforms.
    #    Keeping this separate from the main bridge ensures /clock is available
    #    even if the main bridge is delayed.
    # -------------------------------------------------------------------------
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    # -------------------------------------------------------------------------
    # 4. Robot State Publisher
    #    - use_sim_time: True  → RSP honours /clock for its own stamps
    #    - ignore_timestamp: True → RSP keeps re-broadcasting fixed joints at
    #      publish_frequency Hz instead of going silent after the first t=0
    #      publish. This is the key fix for chassis/sensors showing 0.0.
    # -------------------------------------------------------------------------
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
            'publish_frequency': 50.0,
            'ignore_timestamp': True,   # <-- FIX: keeps fixed joints alive
        }]
    )

    # -------------------------------------------------------------------------
    # 5. Spawn robot — delayed 3 s to give Gazebo time to fully load the world
    #    before we try to insert the robot model.
    # -------------------------------------------------------------------------
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', '/robot_description',
            '-name',  'my_robot',
            '-x', '-8.0',
            '-y', '-8.0',
            '-z', '0.1',
            '-Y', '0.0',
        ],
        output='screen'
    )

    # -------------------------------------------------------------------------
    # 6. Main sensor/control bridge (everything except /clock)
    #    /clock is handled by clock_bridge above to guarantee early availability.
    # -------------------------------------------------------------------------
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='main_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/model/my_robot/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry', # Ensure this matches GZ topic
            '/world/maze_world/model/my_robot/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
        ],
        remappings=[
            ('/model/my_robot/odometry', '/odom'),
            ('/world/maze_world/model/my_robot/joint_state', '/joint_states'),
        ],
        output='screen'
    )

    # -------------------------------------------------------------------------
    # 7. EKF — fuses /odom + /imu into odom → base_link TF
    # -------------------------------------------------------------------------
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(pkg_share, 'config', 'ekf.yaml'),
            {'use_sim_time': True},
        ]
    )

    # -------------------------------------------------------------------------
    # 8. RViz
    # -------------------------------------------------------------------------
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            os.path.join(pkg, 'config', 'mapper_params_online_async.yaml'),
            {'use_sim_time': True}
        ]
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_slam',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,               # ← automatically configure + activate
            'node_names': ['slam_toolbox'],  # ← must match the node's name
            'bond_timeout': 4.0,
        }]
    )

    delayed_nodes = TimerAction(
        period=5.0,
        actions=[spawn, bridge, ekf_node, rviz, slam_toolbox, lifecycle_manager]
    )

    return LaunchDescription([
        gz_sim,         # 1. Gazebo first
        clock_bridge,   # 2. /clock bridge immediately — RSP needs this
        rsp,            # 3. RSP immediately — now has clock, ignore_timestamp keeps it alive
        delayed_nodes,  # 4. Everything else after 3 s
    ])