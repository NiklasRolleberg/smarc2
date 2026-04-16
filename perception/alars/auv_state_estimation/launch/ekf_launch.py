from dji_msgs.msg import Topics
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value="M350"
    )
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false')

    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")

    params_file = PathJoinSubstitution([
        FindPackageShare("auv_state_estimation"),
        "config",
        "params.yaml"
    ])

    cam_params_file = PathJoinSubstitution([
        FindPackageShare("auv_state_estimation"),
        "config",
        "cam_params.yaml"
    ])
    
    ekf_node = Node(
        package="auv_state_estimation",
        executable="ekf_node",
        namespace=namespace,
        name="ekf_node",
        output="screen",
        parameters=[
            params_file,
            {
                "use_sim_time": use_sim_time,
                "topics.input_polygon": Topics.ESTIMATED_AUV_OBB_TOPIC,
                "camera_info": cam_params_file,
            }
        ],
        )

    return LaunchDescription([
        namespace_arg,
        use_sim_time_arg,
        ekf_node,
    ])

