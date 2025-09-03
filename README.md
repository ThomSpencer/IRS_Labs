# Needed installs
sudo apt install ros-humble-rqt-tf-tree

# Start/Setup ros2
source /opt/ros/humble/setup.bash 

# Make Package
ros2 pkg create --build-type ament_python --license Apache-2.0 --node-name\
{NAME_OF_PACKAGE} {NAME_OF_NODE}
## Config Files for RVIS
mkdir rviz config launch

# Colcon Build
colcon build --packages-select {NAME_OF_PACKAGE}
source install/local_setup.bash 
ros2 run {NAME_OF_PACKAGE} {NAME_OF_NODE}

# Run Mapping node
ros2 launch hand_solo_virtual_nav mapping_launch.py

# Transformation tree
ros2 run rqt_tf_tree rqt_tf_tree