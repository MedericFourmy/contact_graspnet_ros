ROS node interface to pytorch implementation of ContactGraspnet

## Install
```bash
vcs import <path>/<to>/<dependencies> < src/contact_graspnet_ros/contact_graspnet_ros/deps.repos
cd <path>/<to>/<dependencies>/contact_graspnet_pytorch
pip install -e .
```
https://github.com/MedericFourmy/contact_graspnet_pytorch.git
# Build
`colcon build --simlink-install`
`source install/setup.bash`

## Run 
```
cd ~/ros2_ws/src/ros2_ws
source install/setup.bash
ros2 launch contact_graspnet_examples contact_graspnet.launch.py
```

## Check results
```
ros2 service call /contact_graspnet/get_scene_grasps contact_graspnet_msgs/srv/GetSceneGrasps
```


/happypose/seg_masks
/camera/aligned_depth_to_color/camera_info
/camera/aligned_depth_to_color/image_raw
/camera/color/camera_info
/camera/color/image_raw
/camera/depth/color/points
/tf
/joint_states

# ros2 bag record
ros2 bag record /happypose/seg_masks /camera/aligned_depth_to_color/camera_info /camera/aligned_depth_to_color/image_raw /camera/color/camera_info /camera/color/image_raw /camera/depth/color/points /tf /joint_states