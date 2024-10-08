import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
 
def generate_launch_description():
    headless = LaunchConfiguration('headless')
    world = LaunchConfiguration('world')
    slam = LaunchConfiguration('slam')
    rviz_config_file = LaunchConfiguration('rviz_config_file')

    declare_headless = DeclareLaunchArgument(
        'headless', default_value='False', description='Launch simulation in headless mode')
    declare_world = DeclareLaunchArgument(
        'world', default_value=os.path.join(
            get_package_share_directory('turtlebot3_gazebo'), 'worlds', 'turtlebot3_dqn_stage4.world'), description='World file to load')
    declare_slam = DeclareLaunchArgument(
        'slam', default_value='True', description='Enable SLAM')
    declare_rviz_config_file = DeclareLaunchArgument(
        'rviz_config_file', default_value=os.path.join(
    get_package_share_directory('single_agent_rrt'), 'rviz', 'rrt_exploration.rviz'), description='RVIZ config file')
    
    tb3_simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('nav2_bringup'), 'launch', 'tb3_simulation_launch.py')),
        launch_arguments={
            'headless': headless,
            'world': world,
            'slam': slam,
            'rviz_config_file': rviz_config_file,
            'params_file': os.path.join(get_package_share_directory('single_agent_rrt'), 'config', 'nav2_params.yaml'),
        }.items()
    )
    
    # rrt_exploration_node = Node(
    #     package="single_agent_rrt",
    #     executable="rrt_exploration",
    #     name='rrt_exploration',
    #     respawn=True,
    # )
    local_frontier_detector_node = Node(
        package="single_agent_rrt",
        executable="local_frontier_detector",
        name='local_frontier_detector',
        respawn=True,
    )
    global_frontier_detector_node = Node(
        package="single_agent_rrt",
        executable="global_frontier_detector",
        name='global_frontier_detector',
        respawn=True,
    )
    filter_node = Node(
        package="single_agent_rrt",
        executable="filter",
        name='filter',
        respawn=True,
    )
    
    return LaunchDescription([
        SetEnvironmentVariable(name='TURTLEBOT3_MODEL', value='burger'),
        SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value='/opt/ros/humble/share/turtlebot3_gazebo/models'),
        declare_headless,
        declare_world,
        declare_slam,
        declare_rviz_config_file,
        tb3_simulation_launch,
        # rrt_exploration_node,
        filter_node,
        local_frontier_detector_node,
        global_frontier_detector_node
    ])
