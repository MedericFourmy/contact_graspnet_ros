from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_context import LaunchContext
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(
    context: LaunchContext, *args, **kwargs
) -> list[LaunchDescriptionEntity]:
    
    contact_graspnet = Node(
        package="contact_graspnet_ros",
        executable="contact_graspnet_node",
        name="contact_graspnet_node",
        parameters=[
        ],
        # remappings=[
        #     ("/unlabeled/detections", "/happypose/detections"),
        # ],
    )


    rs_launch_args = {
        "camera_name": "camera",
        "camera_namespace": "",
        # "rgb_camera.color_profile": "640x360x30",
        # "depth_module.depth_profile": "640x360x30",
        "rgb_camera.color_profile": "640x480x30",
        "depth_module.depth_profile": "640x480x30",
        # "rgb_camera.color_profile": "1280x720x30",
        # "depth_module.depth_profile": "1280x720x30",
        "initial_reset": "true",
        "pointcloud.enable": "true",
        "align_depth.enable": "true",
        "serial_no": "'827112070860'",  # d435
    }

    rs_cam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("realsense2_camera"),
                        "launch",
                        "rs_launch.py",
                    ]
                )
            ]
        ),
        launch_arguments=rs_launch_args.items(),
    )

    return [
        contact_graspnet,
        rs_cam,
    ]

def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "dataset_name",
            default_value="tless",
            description="Name of the dataset to be used in the pipeline.",
        )
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
