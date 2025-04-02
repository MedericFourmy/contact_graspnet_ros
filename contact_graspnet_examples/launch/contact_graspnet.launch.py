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
from launch_ros.parameter_descriptions import ParameterFile


def launch_setup(
    context: LaunchContext, *args, **kwargs
) -> list[LaunchDescriptionEntity]:
    

    # Obtain argument specifying path from which to load happypose_ros parameters
    happypose_params_path = LaunchConfiguration("happypose_params_path")
    contact_graspnet_params_path = LaunchConfiguration("contact_graspnet_params_path")

    # Start ROS node of happypose (to get the segmentation masks)
    happypose_node = Node(
        package="happypose_ros",
        executable="happypose_node",
        name="happypose_node",
        parameters=[ParameterFile(param_file=happypose_params_path, allow_substs=True)],
        remappings=[
            ("/cam_1/color/image_raw",   "/camera/color/image_raw"),
            ("/cam_1/color/camera_info", "/camera/color/camera_info"),
            ("/cam_1/depth/image_raw",   "/camera/aligned_depth_to_color/image_raw"),
            ("/cam_1/depth/camera_info", "/camera/aligned_depth_to_color/camera_info"),
        ]
    )

    contact_graspnet = Node(
        package="contact_graspnet_ros",
        executable="contact_graspnet_node",
        name="contact_graspnet_node",
        parameters=[ParameterFile(param_file=contact_graspnet_params_path, allow_substs=True)],
            remappings=[
            # ("/camera/color/image_raw",   "/camera/color/image_raw"),
            # ("/camera/color/camera_info", "/camera/color/camera_info"),
            # ("/camera/aligned_depth_to_color/image_raw",   "/camera/aligned_depth_to_color/image_raw"),
            # ("/camera/aligned_depth_to_color/camera_info", "/camera/aligned_depth_to_color/camera_info"),
        ]
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
        # rs_cam,
        # happypose_node,
        contact_graspnet,
    ]

def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "dataset_name",
            default_value="tless",
            description="Name of BOP dataset, used to load specific weights and object models.",
        ),
        DeclareLaunchArgument(
            "happypose_params_path",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("contact_graspnet_examples"),
                    "config",
                    "cosypose_params.yaml",
                ]
            ),
            description="Path to a file containing happypose_ros node parameters.",
        ),
        DeclareLaunchArgument(
            "contact_graspnet_params_path",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("contact_graspnet_examples"),
                    "config",
                    "contact_graspnet_params.yaml",
                ]
            ),
            description="Path to a file containing contact_graspnet_ros node parameters.",
        ),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
