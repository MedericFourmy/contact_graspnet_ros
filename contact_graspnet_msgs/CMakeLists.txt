cmake_minimum_required(VERSION 3.22.1)
project(contact_graspnet_msgs)

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Define message files
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Grasps.msg" "msg/SceneGrasps.msg" "srv/GetSceneGrasps.srv"
  DEPENDENCIES geometry_msgs std_msgs
)

ament_package()
