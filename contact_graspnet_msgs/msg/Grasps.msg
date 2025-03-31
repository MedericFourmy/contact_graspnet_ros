# ObjectGrasps.msg
# This message contains grasp proposals associated with one object.
# A grasp is defined as a pose associated to a score (grasps and scores must have the same length).

geometry_msgs/Pose[] grasps
float64[] scores
