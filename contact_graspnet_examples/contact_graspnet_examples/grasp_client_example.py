"""Example about how to query the contact_graspnet_node service."""

import rclpy
from rclpy.node import Node

from contact_graspnet_msgs.srv import GetSceneGrasps


def main(args=None):
    rclpy.init(args=args)

    node = Node('grasp_client_example')

    grasps_client = node.create_client(
        GetSceneGrasps, "contact_graspnet/get_scene_grasps"
    )

    grasps_client.wait_for_service()
    node.get_logger().info("Graspnet service is available, calling...")
    
    request = GetSceneGrasps.Request()  # Empty request
    future = grasps_client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    node.get_logger().info("Graspnet service response received!")
    resp= future.result()
    scene_grasps = resp.scene_grasps

    node.get_logger().info(f"{len(scene_grasps.object_types)} objects detected.")

    # array of object types (may have repeated values) 
    object_types = scene_grasps.object_types
    # array of object grasps 
    object_grasps = scene_grasps.object_grasps

    assert len(object_types) == len(object_grasps)

    node.get_logger().info(f"Print number of received grasps per object")
    for object_type, grasps in zip(object_types, object_grasps):
        node.get_logger().info(f"{object_type}: {len(grasps.grasps)}")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()






