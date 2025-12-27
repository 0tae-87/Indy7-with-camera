import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, OpaqueFunction
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution 
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

openvla_package = FindPackageShare('indy_openvla') 

def get_config_path(package_share, sub_dirs, file_name):
    return PathJoinSubstitution([package_share] + sub_dirs + [file_name])


def launch_setup(context, *args, **kwargs):
    gazebo_package = FindPackageShare('indy_gazebo')
    indy_description_package = FindPackageShare('indy_description')

    name = LaunchConfiguration("name")
    indy_type = LaunchConfiguration("indy_type")
    indy_eye = LaunchConfiguration("indy_eye")
    prefix = LaunchConfiguration("prefix")
    launch_rviz = LaunchConfiguration("launch_rviz")

    indy_type_val = indy_type.perform(context)

    if (indy_type_val == 'indyrp2') or (indy_type_val == 'indyrp2_v2'):
        initial_joint_controllers = PathJoinSubstitution(
            [gazebo_package, "controller", "indy_controllers_7dof.yaml"]
        )
    else:
        initial_joint_controllers = PathJoinSubstitution(
            [openvla_package, "config", "indy_openvla_controllers.yaml"]
        )

    sub_dirs_type = ["urdf", "config", indy_type_val]
    sub_dirs_init = ["urdf", "config"]

    joint_limits_path = get_config_path(indy_description_package, sub_dirs_type, "joint_limits.yaml")
    kinematics_path = get_config_path(indy_description_package, sub_dirs_type, "kinematics.yaml")
    physical_path = get_config_path(indy_description_package, sub_dirs_type, "physical_parameters.yaml")
    visual_path = get_config_path(indy_description_package, sub_dirs_type, "visual_parameters.yaml")
    joint_axes_path = get_config_path(indy_description_package, sub_dirs_type, "joint_axes.yaml")
    initial_positions_path = get_config_path(indy_description_package, sub_dirs_init, "initial_positions.yaml")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([openvla_package, "urdf", "indy.urdf.xacro"]), 
            " ",
            "name:=", name,
            " ",
            "indy_type:=", indy_type,
            " ",
            "indy_eye:=", indy_eye,
            " ",
            "prefix:=", prefix,
            " ",
            "sim_gazebo:=true",
            " ",
            "simulation_controllers:=", initial_joint_controllers,
            " ",
            "transmission_hw_interface:=", "hardware_interface/PositionJointInterface",
            " ",
            "joint_limit_params:=", joint_limits_path,
            " ",
            "kinematics_params:=", kinematics_path,
            " ",
            "physical_params:=", physical_path,
            " ",
            "visual_params:=", visual_path,
            " ",
            "joint_axes_params:=", joint_axes_path,
            " ",
            "initial_positions:=", initial_positions_path,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    description_package_original = FindPackageShare('indy_description') 
    rviz_config_file = PathJoinSubstitution(
        [description_package_original, "rviz_config", "indy.rviz"]
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": True}, robot_description],
    )

    static_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='zed2i_depth_frame_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'zed2i_camera_frame', 'indy/link6/zed2i_depth']
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )
    
    joint_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments=[('gz_args', [' -r -v 4 ', PathJoinSubstitution([FindPackageShare('indy_gazebo'), 'worlds', 'camera_world.sdf'])])]
    )

    gazebo_spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-string', robot_description_content,
                   '-name', 'indy',
                   '-allow_renaming', 'false'],
    )
    
    bridge_params = os.path.join(get_package_share_directory('indy_gazebo'),'config','gz_bridge.yaml')
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ]
    )

    rviz_node = Node(
        condition=IfCondition(launch_rviz),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    delay_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gazebo_spawn_robot,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    delay_robot_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[joint_controller_spawner],
        )
    )

    delay_rviz2_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_controller_spawner,
            on_exit=[rviz_node],
        )
    )

    nodes_to_start = [
        gazebo,
        gazebo_spawn_robot,
        robot_state_publisher_node,
        ros_gz_bridge,
        static_tf_publisher,
        delay_joint_state_broadcaster_spawner,
        delay_robot_controller_spawner,
        delay_rviz2_spawner,
    ]
    return nodes_to_start

def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "name",
            default_value="indy"
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "indy_type",
            default_value="indy7_v2",
            description="Type of Indy robot.",
            choices=["indy7", "indy7_v2" , "indy12", "indy12_v2", "indyrp2", "indyrp2_v2", "icon7l", "icon3", "nuri3s", "nuri4s", "nuri7c", "nuri20c", "opti5"]
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "indy_eye",
            default_value="false",
            description="Work with Indy Eye",
        )
    )
 
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for multi-robot setup. \
            If changed than also joint names in the controllers configuration have to be updated."
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz", 
            default_value="true", 
            description="Launch RViz?"
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])