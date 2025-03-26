#!/usr/bin/env python3

import os
import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R

from contact_graspnet_pytorch.contact_grasp_estimator import GraspEstimator
from contact_graspnet_pytorch import config_utils

from contact_graspnet_pytorch.visualization_utils_o3d import visualize_grasps, show_image
from contact_graspnet_pytorch.checkpoints import CheckpointIO 

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from message_filters import ApproximateTimeSynchronizer, Subscriber
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_system_default
from rclpy.qos_overriding_options import QoSOverridingOptions
from geometry_msgs.msg import PoseStamped


class ContactGraspnetNode(Node):
    def __init__(self, node_name: str = "contact_graspnet_node", **kwargs):
        super().__init__(node_name, **kwargs)

        #######################
        # ContactGraspnet setup
        #######################

        # Build the model
        self.z_range = [0.2, 1.8]
        self.local_regions = True
        self.filter_grasps = True
        self.skip_border_objects = True
        self.forward_passes = 1
        # TODO: make checkpoint_dir a parameter 
        ckpt_dir = '/home/ros/sandbox_mf/contact_graspnet_pytorch/checkpoints/contact_graspnet'
        arg_configs = []
        global_config = config_utils.load_config(ckpt_dir, batch_size=self.forward_passes, arg_configs=arg_configs)
        self.grasp_estimator = GraspEstimator(global_config)

        # Load the weights
        model_checkpoint_dir = os.path.join(ckpt_dir, 'checkpoints')
        checkpoint_io = CheckpointIO(checkpoint_dir=model_checkpoint_dir, model=self.grasp_estimator.model)
        try:
            checkpoint_io.load('model.pt')
        except FileExistsError as e:
            self.get_logger().error('No model checkpoint found')
            raise e

        ################
        # ROS setup
        ################
        cam_name = 'camera'
        sync_topics = [
            Subscriber(
                self._node,
                Image,
                cam_name + "/color/image_raw",
                qos_profile=qos_profile_system_default,
                qos_overriding_options=QoSOverridingOptions.with_default_policies(),
            ),
            Subscriber(
                self._node,
                CameraInfo,
                cam_name + "/color/camera_info",
                qos_profile=qos_profile_system_default,
                qos_overriding_options=QoSOverridingOptions.with_default_policies(),
            ),
            Subscriber(
                self._node,
                Image,
                cam_name + "/aligned_depth_to_color/image_raw",
                qos_profile=qos_profile_system_default,
                qos_overriding_options=QoSOverridingOptions.with_default_policies(),
            ),
            Subscriber(
                self._node,
                CameraInfo,
                cam_name + "/aligned_depth_to_color/camera_info",
                qos_profile=qos_profile_system_default,
                qos_overriding_options=QoSOverridingOptions.with_default_policies(),
            ),
            Subscriber(
                self._node,
                Image,
                cam_name + "/happypose/seg_masks",
                qos_profile=qos_profile_system_default,
                qos_overriding_options=QoSOverridingOptions.with_default_policies(),
            ),      
        ]

        # Synchronizer for approximate time synchronization
        self.sync = ApproximateTimeSynchronizer(sync_topics, queue_size=10, slop=0.1)
        self.sync.registerCallback(self.callback)

        # OpenCV bridge
        self.bridge = CvBridge()

        # Grasp pose publisher
        self.pose_publisher = self.create_publisher(PoseStamped, 'contact_graspnet/closest_grasp_pose', 10)

        self.get_logger().info('ContactGraspnetNode has been started.')

    def callback(self, 
                 color_image: Image,
                 color_camera_info: CameraInfo,
                 depth_image: Image,
                 depth_camera_info: CameraInfo,
                 seg_masks: Image = None,
                 ) -> None:
        
        connections = self.sync.input_connections
        if not np.allclose(color_camera_info.k, depth_camera_info.k):
            self.get_logger().warn(
                f"Topics '{connections[1].getTopic()}' and "
                f" '{connections[3].getTopic()}' contain different intrinsics matrices!"
                " Both color and depth images have to have the same intrinsics for ICP to work!",
                throttle_duration_sec=5.0,
            )
            return        

        try:
            # Convert ROS Image messages to OpenCV images
            rgb = self.bridge.imgmsg_to_cv2(color_image, desired_encoding='bgr8')
            depth = self.bridge.imgmsg_to_cv2(depth_image, desired_encoding='passthrough')
            segmap = self.bridge.imgmsg_to_cv2(seg_masks, desired_encoding='mono8')

            cam_K = color_camera_info.k.reshape((3, 3))

            # Process the images
            pc_full, pc_segments, pc_colors = self.grasp_estimator.extract_point_clouds(
                depth_image, cam_K, segmap, rgb, 
                skip_border_objects=self.skip_border_objects, 
                z_range=self.z_range
            )

            pred_grasps_cam, scores, contact_pts, _ = self.grasp_estimator.predict_scene_grasps(
                pc_full, 
                pc_segments=pc_segments, 
                local_regions=self.local_regions, 
                filter_grasps=self.filter_grasps, 
                forward_passes=self.forward_passes
            )  

            id_nb_grasps = {k: len(v) for k, v in pred_grasps_cam.items()}
            print("# of grasps per object id:", id_nb_grasps)

            # TODO: decide on a strategy to select the best grasp
            # For now, we just select the first object and the best grasp
            object_id = list(pred_grasps_cam.keys())[0]

            # grasps and scores associated with object id
            grasps = pred_grasps_cam[object_id]
            scores = scores[object_id]

            # # get the best grasp
            # best_grasp_idx = np.argmax(scores)
            # best_grasp = grasps[best_grasp_idx]
            # print("Best grasp:", best_grasp)

            # get closest grasp to the camera
            min_dist_cam_id = grasps[:,2,3].argmin()
            T_cg = grasps[min_dist_cam_id]
            print("closest_grasp:", T_cg)

            # Publish the closest grasp as a PoseStamped message
            quat = R.from_matrix(T_cg[:3,:3]).as_quat(scalar_first=False)
            trans = T_cg[:3,3]

            # Create PoseStamped message
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = "camera_frame"  # Replace with your camera frame
            pose_msg.pose.position.x = trans[0]
            pose_msg.pose.position.y = trans[1]
            pose_msg.pose.position.z = trans[2]
            pose_msg.pose.orientation.x = quat[0]
            pose_msg.pose.orientation.y = quat[1]
            pose_msg.pose.orientation.z = quat[2]
            pose_msg.pose.orientation.w = quat[3]

            # Publish the message
            self.pose_publisher.publish(pose_msg)

        except Exception as e:
            self.get_logger().error(f'Error processing images: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ContactGraspnetNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()