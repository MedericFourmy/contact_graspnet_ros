#!/usr/bin/env python3

import os
import numpy as np
import cv2
from pathlib import Path
from transforms3d.quaternions import mat2quat
import pairing

import contact_graspnet_pytorch
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
from geometry_msgs.msg import Pose
from agimus_msgs.msg import Grasps, SceneGrasps
from agimus_msgs.srv import GetSceneGrasps


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
        # TODO: pbly works only if contact_graspnet_pytorch what installed with pip install -e .
        ckpt_dir = Path(contact_graspnet_pytorch.__file__).parent.parent / "checkpoints/contact_graspnet"
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
                self,
                Image,
                cam_name + "/color/image_raw",
                qos_profile=qos_profile_system_default,
                qos_overriding_options=QoSOverridingOptions.with_default_policies(),
            ),
            Subscriber(
                self,
                CameraInfo,
                cam_name + "/color/camera_info",
                qos_profile=qos_profile_system_default,
                qos_overriding_options=QoSOverridingOptions.with_default_policies(),
            ),
            Subscriber(
                self,
                Image,
                cam_name + "/aligned_depth_to_color/image_raw",
                qos_profile=qos_profile_system_default,
                qos_overriding_options=QoSOverridingOptions.with_default_policies(),
            ),
            Subscriber(
                self,
                CameraInfo,
                cam_name + "/aligned_depth_to_color/camera_info",
                qos_profile=qos_profile_system_default,
                qos_overriding_options=QoSOverridingOptions.with_default_policies(),
            ),
            Subscriber(
                self,
                Image,
                cam_name + "/happypose/seg_masks",
                qos_profile=qos_profile_system_default,
                qos_overriding_options=QoSOverridingOptions.with_default_policies(),
            ),      
        ]

        # Synchronizer for approximate time synchronization
        self.sync = ApproximateTimeSynchronizer(sync_topics, queue_size=10, slop=0.1)
        self.sync.registerCallback(self.imgs_callback)

        # OpenCV bridge
        self.bridge = CvBridge()

        # Grasp pose publisher
        self.grasps_service = self.create_service(GetSceneGrasps, 'contact_graspnet/get_scene_grasps')
        self.grasps_client = self.create_client(GetSceneGrasps, 'contact_graspnet/get_scene_grasps')

        self.get_logger().info('ContactGraspnetNode has been started.')

    def imgs_callback(self, 
                    color_image: Image,
                    color_camera_info: CameraInfo,
                    depth_image: Image,
                    depth_camera_info: CameraInfo,
                    seg_masks: Image = None,
                 ) -> None:
        
        """
        Only store synchronized image, depth, info and segmentation masks.
        """
        
        connections = self.sync.input_connections
        if not np.allclose(color_camera_info.k, depth_camera_info.k):
            self.get_logger().warn(
                f"Topics '{connections[1].getTopic()}' and "
                f" '{connections[3].getTopic()}' contain different intrinsics matrices!"
                " Both color and depth images have to have the same intrinsics for ICP to work!",
                throttle_duration_sec=5.0,
            )
            return        

        # Convert ROS Image messages to OpenCV images
        self.rgb = self.bridge.imgmsg_to_cv2(color_image, desired_encoding='bgr8')
        self.depth = self.bridge.imgmsg_to_cv2(depth_image, desired_encoding='passthrough')
        self.segmap = self.bridge.imgmsg_to_cv2(seg_masks, desired_encoding='mono16')
        self.cam_K = color_camera_info.k.reshape((3, 3))

        # # Call our own service every time
        # req = GetSceneGrasps.Request()
        # self.future = self.grasps_client.call_async(req)
        # rclpy.spin_until_future_complete(self, self.future)
        # return self.future.result()

    def scene_grasps_srv_callback(self, request: GetSceneGrasps.Request, response: GetSceneGrasps.Response):
        """Return all grasps."""
        scene_grasps_msg = self.contact_graspnet_inference(self.rgb, self.depth, self.segmap, self.cam_K)
        response.scene_grasps = scene_grasps_msg

        return response

    def contact_graspnet_inference(self, rgb, depth, segmap, cam_K):
        if any(arg is None for arg in [rgb, depth, segmap, cam_K]):
            self.get_logger().warn("Not image received yet, return empty grasps")
            return SceneGrasps()

        try:
            # Process the images
            pc_full, pc_segments, pc_colors = self.grasp_estimator.extract_point_clouds(
                depth, cam_K, segmap, rgb, 
                skip_border_objects=self.skip_border_objects, 
                z_range=self.z_range
            )

            all_grasps, all_scores, contact_pts, _ = self.grasp_estimator.predict_scene_grasps(
                pc_full, 
                pc_segments=pc_segments, 
                local_regions=self.local_regions, 
                filter_grasps=self.filter_grasps, 
                forward_passes=self.forward_passes
            )  

        except Exception as e:
            self.get_logger().error(f'Error processing images: {e}')
            return SceneGrasps()
        
        id_nb_grasps = {pairing.depair(seg_id)[1]: len(v) for seg_id, v in all_grasps.items()}
        print("# of grasps per object id:", id_nb_grasps)

        scene_grasps = SceneGrasps()

        for seg_id in all_grasps:
            label_id, instance_id = pairing.depair(seg_id)

            # grasps and scores associated with object instance
            grasps = all_grasps[seg_id]
            scores = all_scores[seg_id]

            # loop over the grasps for this particular objects
            grasps_msg = Grasps()

            for i in range(len(grasps)):
                T_cg = grasps[i]
                score = scores[i]

                # Publish the closest grasp as a PoseStamped message
                qw, qx, qy, qz = mat2quat(T_cg[:3,:3])
                trans = T_cg[:3,3]

                pose_msg = Pose()
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.header.frame_id = "camera_frame"  # Replace with your camera frame
                pose_msg.pose.position.x = trans[0]
                pose_msg.pose.position.y = trans[1]
                pose_msg.pose.position.z = trans[2]
                pose_msg.pose.orientation.x = qx
                pose_msg.pose.orientation.y = qy
                pose_msg.pose.orientation.z = qz
                pose_msg.pose.orientation.w = qw

                grasps_msg.grasps.append(pose_msg)
                grasps_msg.scores.append(score)

                # Publish the message
                self.pose_publisher.publish(pose_msg)

            scene_grasps.object_grasps.append(grasps_msg)
            scene_grasps.object_type.append(str(label_id))

        return scene_grasps


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
