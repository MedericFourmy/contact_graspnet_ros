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
ros2 launch contact_graspnet_examples contact_graspnet.launch.py
```