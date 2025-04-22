#!/usr/bin/env python3

# std lib
import os
import numpy as np
import numpy.typing as npt
import time
from pathlib import Path

# Third party
from transforms3d.quaternions import mat2quat
import pairing

# contact graspnet
import contact_graspnet_pytorch
from contact_graspnet_pytorch.contact_grasp_estimator import GraspEstimator
from contact_graspnet_pytorch import config_utils
from contact_graspnet_pytorch.checkpoints import CheckpointIO 

# ROS
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from message_filters import ApproximateTimeSynchronizer, Subscriber
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_system_default
from rclpy.qos_overriding_options import QoSOverridingOptions
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker, MarkerArray

# contact_graspnet_ros
from contact_graspnet_msgs.msg import Grasps, SceneGrasps
from contact_graspnet_msgs.srv import GetSceneGrasps
# Automatically generated file
from contact_graspnet_ros.contact_graspnet_ros_parameters import contact_graspnet_ros  # noqa: E402

"""
ros2 service call /contact_graspnet/get_scene_grasps contact_graspnet_msgs/srv/GetSceneGrasps
"""

# If we run our of colors, could switch to https://github.com/alan-turing-institute/distinctipy
LABELED_COLORS = [
    ([0.89019608, 0.46666667, 0.76078431, 1.0], "pink"),
    ([0.12156863, 0.46666667, 0.70588235, 1.0], "blue"),
    ([0.54901961, 0.3372549, 0.29411765, 1.0], "brown"),
    ([0.68235294, 0.78039216, 0.90980392, 1.0], "light blue"),
    ([0.99215686, 0.70588235, 0.1254902, 1.0], "yellow"),
    ([0.99607843, 0.96078431, 0.19607843, 1.0], "light yellow"),
    ([0.89019608, 0.46666667, 0.76078431, 1.0], "pink"),
    ([0.12156863, 0.46666667, 0.70588235, 1.0], "blue"),
    ([0.54901961, 0.3372549, 0.29411765, 1.0], "brown"),
    ([0.68235294, 0.78039216, 0.90980392, 1.0], "light blue"),
    ([0.99215686, 0.70588235, 0.1254902, 1.0], "yellow"),
    ([0.99607843, 0.96078431, 0.19607843, 1.0], "light yellow"),
]
BEST_GRASP_COLOR = [0.0, 1., 0.0, 1.0]


class ContactGraspnetNode(Node):
    def __init__(self, node_name: str = "contact_graspnet_node", **kwargs):
        super().__init__(node_name, **kwargs)

        #######################
        # ContactGraspnet setup
        #######################

        try:
            self._param_listener = contact_graspnet_ros.ParamListener(self)
            self._params = self._param_listener.get_params()
        except Exception as e:
            self.get_logger().error(str(e))
            raise e

        # Build the model
        # self._params.local_regions = True
        # self._params.filter_grasps = True
        # self._params.skip_border_objects = True
        # self._params.forward_passes = 1
        self.get_logger().info(f"{self._params.local_regions}")
        self.get_logger().info(f"{self._params.filter_grasps}")
        self.get_logger().info(f"{self._params.skip_border_objects}")
        self.get_logger().info(f"{self._params.forward_passes}")
        self.get_logger().info(f"{self._params.viz_grasps}")

        # TODO: pbly works only if contact_graspnet_pytorch what installed with pip install -e .
        ckpt_dir = Path(contact_graspnet_pytorch.__file__).parent.parent / "checkpoints/contact_graspnet"
        global_config = config_utils.load_config(ckpt_dir)
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
        topics_msg_type = [
            ("/camera/color/image_raw", Image),
            ("/camera/color/camera_info", CameraInfo),
            ("/camera/aligned_depth_to_color/image_raw", Image),
            ("/camera/aligned_depth_to_color/camera_info", CameraInfo),
            ("/seg_masks", Image),
        ]
        sync_subs = [
            Subscriber( 
                self, msg_type, topic,
                qos_profile=qos_profile_system_default,
                qos_overriding_options=QoSOverridingOptions.with_default_policies(),
            )
            for topic, msg_type in topics_msg_type
        ]

        # Synchronizer for approximate time synchronization
        self.sync = ApproximateTimeSynchronizer(sync_subs, queue_size=50, slop=0.1)
        self.sync.registerCallback(self.callback_imgs)

        # OpenCV bridge
        self.bridge = CvBridge()

        # Grasp pose publisher
        self.grasps_service = self.create_service(GetSceneGrasps, 'contact_graspnet/get_scene_grasps', self.callback_scene_grasps)

        if self._params.viz_grasps:
            # Grasp markers publisher
            self._markers_pub = self.create_publisher(MarkerArray, "contact_graspnet_ros/markers", 10)

        # contact graspnet inputs stored as attributes, initialized by callback_imgs
        self.rgb = None
        self.depth = None
        self.seg_masks = None 
        self.cam_K = None

        self.get_logger().info('ContactGraspnetNode has been started.')

    def callback_imgs(self, 
                      color_image: Image,
                      color_camera_info: CameraInfo,
                      depth_image: Image,
                      depth_camera_info: CameraInfo,
                      seg_masks: Image = None,
                 ) -> None:
        
        """
        Only store synchronized image, depth, info and segmentation masks.
        """
        
        self.get_logger().warn("callback_imgs.")
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
        self.seg_masks = self.bridge.imgmsg_to_cv2(seg_masks, desired_encoding='mono16')
        self.cam_K = color_camera_info.k.reshape((3, 3))
        depth = self.bridge.imgmsg_to_cv2(depth_image, desired_encoding='passthrough')
        # rescale and convert to floats 
        self.depth = self._params.depth_scale * depth.astype(np.float32)

        unique, counts = np.unique(self.seg_masks, return_counts=True)
        d = {u: c for u, c in zip(unique, counts)}
        self.get_logger().warn(f"{d}")

    def callback_scene_grasps(self, request: GetSceneGrasps.Request, response: GetSceneGrasps.Response):
        """Return all grasps."""
        t1 = time.time()
        self.get_logger().warn("SERVICE called: callback_scene_grasps.")
        if any(arg is None for arg in [self.rgb, self.depth, self.seg_masks, self.cam_K]):
            self.get_logger().warn("No image received yet, return empty grasps")
            return GetSceneGrasps.Response()
        
        scene_grasps_msg = self.contact_graspnet_inference(self.rgb, self.depth, self.seg_masks, self.cam_K)
        response.scene_grasps = scene_grasps_msg
        self.get_logger().warn(f"SERVICE inference over in {time.time() - t1} (s): callback_scene_grasps.")

        if self._params.viz_grasps:
            self.publish_grasp_markers(scene_grasps_msg)

        return response
    
    def publish_grasp_markers(self, scene_grasps_msg: SceneGrasps):
        grasp_markers = MarkerArray()
        for i, (grasp, object_type) in enumerate(zip(scene_grasps_msg.object_grasps, scene_grasps_msg.object_types)):
            if len(grasp.grasps) > 0:
                if i < len(LABELED_COLORS):
                    object_color, string_color = LABELED_COLORS[i]
                else:
                    object_color, string_color = LABELED_COLORS[len(LABELED_COLORS)-1]
                    self.get_logger().warning(f"LABELED_COLORS is too small: {i}")

                best_score_id = np.argmax(grasp.scores)
                best_score = grasp.scores[best_score_id]
                self.get_logger().info(f"Object {object_type} has {len(grasp.grasps)} grasps,of color {string_color} and best score {best_score}")
            for j, grasp_pose in enumerate(grasp.grasps):
                color = BEST_GRASP_COLOR if j == best_score_id else object_color
                marker = Marker()
                marker.header.frame_id = "camera_color_optical_frame"
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "grasps"
                marker.id = i * 100 + j
                marker.mesh_resource="package://contact_graspnet_ros/meshes/panda_gripper.obj"
                marker.type = Marker.MESH_RESOURCE
                marker.action = Marker.ADD
                marker.pose = grasp_pose
                marker.scale.x = 1.0
                marker.scale.y = 0.05  # flatten the gripper to make visualization more readable
                marker.scale.z = 1.0
                marker.color.r = color[0]
                marker.color.g = color[1]
                marker.color.b = color[2]
                marker.color.a = 0.8 if grasp.scores[j] == best_score else 0.05
                grasp_markers.markers.append(marker)
        
        self._markers_pub.publish(grasp_markers)

    def contact_graspnet_inference(
            self, 
            rgb: npt.NDArray, 
            depth: npt.NDArray, 
            seg_masks: npt.NDArray, 
            cam_K: npt.NDArray) -> SceneGrasps:
        try:
            # Process the images
            pc_full, pc_segments, pc_colors = self.grasp_estimator.extract_point_clouds(
                depth, cam_K, seg_masks, rgb, 
                skip_border_objects=self._params.skip_border_objects, 
                z_range=self._params.z_range
            )

            all_grasps, all_scores, contact_pts, _ = self.grasp_estimator.predict_scene_grasps(
                pc_full, 
                pc_segments=pc_segments, 
                local_regions=self._params.local_regions, 
                filter_grasps=self._params.filter_grasps, 
                forward_passes=self._params.forward_passes
            )  

        except Exception as e:
            self.get_logger().error(f'Error processing images: {e}')
            return SceneGrasps()
        
        id_nb_grasps = {pairing.depair(seg_id)[1]: len(v) for seg_id, v in all_grasps.items()}
        self.get_logger().warning(f"# of grasps per object id: {id_nb_grasps}")

        scene_grasps = SceneGrasps()

        for seg_id in all_grasps:
            label_id, instance_id = pairing.depair(seg_id)

            # grasps and scores associated with object instance
            grasps = all_grasps[seg_id].astype(np.float64)
            scores = all_scores[seg_id].astype(np.float64)

            # loop over the grasps for this particular objects
            grasps_msg = Grasps()

            for i in range(len(grasps)):
                T_cg = grasps[i]
                score = scores[i]

                # Publish the closest grasp as a PoseStamped message
                qw, qx, qy, qz = mat2quat(T_cg[:3,:3])
                trans = T_cg[:3,3]

                pose_msg = Pose()
                pose_msg.position.x = trans[0]
                pose_msg.position.y = trans[1]
                pose_msg.position.z = trans[2]
                pose_msg.orientation.x = qx
                pose_msg.orientation.y = qy
                pose_msg.orientation.z = qz
                pose_msg.orientation.w = qw

                grasps_msg.grasps.append(pose_msg)
                grasps_msg.scores.append(score)

            scene_grasps.object_grasps.append(grasps_msg)
            scene_grasps.object_types.append(str(label_id))

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
