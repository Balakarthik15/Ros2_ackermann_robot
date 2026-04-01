import os
import yaml
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription,ExecuteProcess,RegisterEventHandler,TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node 
from launch_ros.parameter_descriptions import ParameterValue
from launch.event_handlers import OnProcessExit




def generate_launch_description():

    
    package_name = "ackermann_gazebo"
    package_path = get_package_share_directory(package_name)


    robot_description_path = os.path.join(package_path, 'urdf', 'vehicle.urdf.xacro')
    gz_bridge_params_path = os.path.join(package_path, 'config', 'ros_gz_bridge.yaml')
    vehicle_params_path = os.path.join(package_path, 'config', 'robot_params.yaml')
 



    # ========================================
    # 1. CLEANUP STEP (Optional but recommended)
    # This kills lingering gz processes before starting
    # ========================================
    cleanup_gz = ExecuteProcess(
        cmd=['pkill', '-9', 'gz'],
        output='screen'
    )

    robot_description_raw = xacro.process_file(robot_description_path).toxml()
        # Process Xacro
    robot_description_content = ParameterValue(
        Command(['xacro ', robot_description_path]),
        value_type=str
    )

    # ========================================
    # LAUNCH ARGUMENTS
    # ========================================
    use_sim_time = LaunchConfiguration("use_sim_time")
    
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", 
        default_value="true", 
        description="Use simulation clock if true"
    )


    # 1. Declare Arguments
    world_arg = DeclareLaunchArgument('world', default_value='lab.sdf')
    x_arg = DeclareLaunchArgument('x', default_value='0.0')
    y_arg = DeclareLaunchArgument('y', default_value='0.0')
    z_arg = DeclareLaunchArgument('z', default_value='0.2')
    yaw_arg = DeclareLaunchArgument('Y', default_value='0.0')

    # 2. Retrieve Configurations
    world_file = os.path.join(package_path, 'worlds', 'lab.sdf')
    use_sim_time = LaunchConfiguration("use_sim_time")




    # 4. Gazebo Sim Launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r -v 4 ', world_file], # Combined list handles substitution correctly
            'on_exit_shutdown': 'true'
        }.items()
    )

    # 5. Nodes

   



     # Spawn Robot
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "robot_description",
            "-name", "bot_ackermann",
            "-x", LaunchConfiguration('x'),
            "-y", LaunchConfiguration('y'),
            "-z", LaunchConfiguration('z'),
            "-Y", LaunchConfiguration('Y'),
            '-allow_renaming', 'false'
        ],
        output="screen",
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description':  robot_description_content,
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )

    gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        arguments=['--ros-args', '-p', f'config_file:={gz_bridge_params_path}'],
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )

   


    launch_description = LaunchDescription([
        declare_use_sim_time,
        world_arg,
        x_arg, y_arg, z_arg,yaw_arg,
        gazebo_launch,
        spawn_robot,
        robot_state_publisher_node,
        gz_bridge_node,
       
    ])

    # 6. Build Launch Description
    return launch_description