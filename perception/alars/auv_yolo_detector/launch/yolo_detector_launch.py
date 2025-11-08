from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # ---- frequently changed params as launch arguments ...
    mode_arg = DeclareLaunchArgument('mode', default_value='real')
    namespace_arg = DeclareLaunchArgument('namespace', default_value='Quadrotor')
    device_arg = DeclareLaunchArgument('namespace', default_value='cpu')
    inference_frequency_arg = DeclareLaunchArgument('inference_frequency', default_value='0.1')
    model_path_arg = DeclareLaunchArgument('model_path',
                                       default_value = '/home/fm/KTH_Courses/ResearchProject/RProj_GitRepoFork/colcon_ws/src/smarc2/perception/alars/auv_yolo_detector')

    # ... and as node params
    mode = LaunchConfiguration('mode')
    namespace = LaunchConfiguration('namespace')
    device = LaunchConfiguration('device')
    inference_frequency = LaunchConfiguration('inference_frequency')
    model_path = LaunchConfiguration('model_path')

    # ---- rarely changed params from yaml (yaml has every parameter but launch arguments will override)
    config_file = PathJoinSubstitution([
        FindPackageShare('auv_yolo_detector'),
        'config',
        'params.yaml'
    ])

    detector_node = Node(
        package='auv_yolo_detector',
        executable='auv_yolo_detector',
        namespace=namespace,
        output='screen',
        parameters=[
            config_file,
            {
                'mode': mode,
                'namespace': namespace,
                'device': device,
                'inference_frequency': inference_frequency,
                'model_path': model_path,
                
            }
        ]
    )

    return LaunchDescription([
        namespace_arg,
        mode_arg,
        device_arg,
        inference_frequency_arg,
        LogInfo(msg=["[Launch] mode argument = ", mode]),
        LogInfo(msg=["[Launch] namespace argument = ", namespace]),
        LogInfo(msg=["[Launch] device argument = ", device]),
        model_path_arg,
        detector_node
    ])
