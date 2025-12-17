from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, FindExecutable
from launch_ros.substitutions import FindPackageShare

def launch_setup(context, *args, **kwargs):
    
    gazebo_package = FindPackageShare('indy_gazebo')
    moveit_config_package = FindPackageShare('indy_moveit')
    openvla_package = FindPackageShare('indy_openvla') # 🌟 추가: OpenVLA 패키지 경로

    # Initialize Arguments
    name = LaunchConfiguration("name")
    indy_type = LaunchConfiguration("indy_type")
    indy_eye = LaunchConfiguration("indy_eye")
    servo_mode = LaunchConfiguration("servo_mode")
    prefix = LaunchConfiguration("prefix")
    
    # 🌟 핵심 수정: 로봇 모델 정의 파일 경로 덮어쓰기 (Override) 🌟
    # Gazebo 런치 파일에 수정된 XACRO 파일 경로를 전달합니다.
    robot_description_macro_path = [
        openvla_package, '/urdf/indy_macro.xacro'
    ]

    # Launch Gazebo with camera-enabled world
    indy_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [gazebo_package, "/launch", "/indy_gazebo.launch.py"]
        ),
        launch_arguments={
            "name": name,
            "indy_type": indy_type,
            "indy_eye": indy_eye,
            "prefix": prefix,
            "launch_rviz": "false", 
            # 🌟 수정: robot_description_macro_path 인자를 전달하여 로봇 모델 경로 덮어쓰기
            "robot_description_macro_path": robot_description_macro_path, 
        }.items(),
    )

    # Launch MoveIt2 for motion planning
    indy_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [moveit_config_package, "/launch", "/moveit.launch.py"]
        ),
        launch_arguments={
            "name": name,
            "indy_type": indy_type,
            "indy_eye": indy_eye,
            "servo_mode": servo_mode,
            "prefix": prefix,
            "use_sim_time": "true",
            "launch_rviz_moveit": "true",
        }.items(),
    )

    nodes_to_launch = [
        indy_gazebo_launch,
        indy_moveit_launch,
    ]

    return nodes_to_launch


def generate_launch_description():
    """
    Generate launch description for OpenVLA + Indy7 setup.
    
    Available topics for OpenVLA:
    - /indy7/zed2i/image          : RGB image
    - /indy7/zed2i/depth/image    : Depth image  
    - /indy7/zed2i/points         : Point cloud
    - /joint_states               : Robot joint states
    """
    
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "name",
            default_value="indy",
            description="Robot namespace"
        )
    )
    # ... (나머지 launch arguments 정의는 기존과 동일) ...
    declared_arguments.append(
        DeclareLaunchArgument(
            "indy_type",
            default_value="indy7",
            description="Type of Indy robot.",
            choices=["indy7", "indy7_v2", "indy12", "indy12_v2", "indyrp2", 
                    "indyrp2_v2", "icon7l", "icon3", "nuri3s", "nuri4s", 
                    "nuri7c", "nuri20c", "opti5"]
        )
    )
 
    declared_arguments.append(
        DeclareLaunchArgument(
            "indy_eye",
            default_value="false",
            description="Work with Indy Eye camera mounted on robot",
        )
    )
 
    declared_arguments.append(
        DeclareLaunchArgument(
            "servo_mode",
            default_value="false",
            description="Enable servo mode for real-time control",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix for joint names (useful for multi-robot setup)"
        )
    )
    
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])