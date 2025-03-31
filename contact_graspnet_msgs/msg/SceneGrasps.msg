# SceneGrasps.msg
# Contains grasp for several object instances.
# Non unique object types are stored alongside a sequence of Grasps.
# These object types may correspond to class id given by a detector.

std_msgs/Header header

string[] object_types
contact_graspnet_msgs/Grasps[] object_grasps
