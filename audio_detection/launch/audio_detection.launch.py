from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable


def generate_launch_description():

    pkg_name = "audio_detection"
    namespace = "audio_detection"

    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED", "1")

    #
    # NODES
    #

    audio_detection_node_cmd = Node(
        package=pkg_name,
        executable="audio_detector_node",
        name="audio_detector_node",
        namespace=namespace,
    )

    manager_node_cmd = Node(
        package=pkg_name,
        executable="manager_node",
        name="manager_node",
        namespace=namespace,
    )

    ld = LaunchDescription()

    ld.add_action(stdout_linebuf_envvar)

    ld.add_action(audio_detection_node_cmd)
    ld.add_action(manager_node_cmd)

    return ld
