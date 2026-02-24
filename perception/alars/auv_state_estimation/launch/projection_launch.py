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

    projection_node = Node(
        package="auv_state_estimation",
        executable="projection",
        namespace=namespace,
        output="screen",
        parameters=[
            params_file,
            {"namespace": namespace},
            {"use_sim_time": use_sim_time}
        ],
    )

    return LaunchDescription([
        namespace_arg,
        use_sim_time_arg,
        projection_node
    ])
