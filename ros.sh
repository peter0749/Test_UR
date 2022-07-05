# put the following source into .bashrc
# source /opt/ros/foxy/setup.bash
# source ~/workspace/ros_ur_driver/install/setup.bash
# source ~/workspace/install/setup.bash
# source ~/ws_moveit2/install/setup.bash
ros2 launch ur_bringup ur_control.launch.py ur_type:=ur3 robot_ip:=yyy.yyy.yyy.yyy use_fake_hardware:=true launch_rviz:=false &
ros2 launch ur_bringup ur_moveit.launch.py ur_type:=ur3 robot_ip:="xxx.xxx" use_fake_hardware:=true launch_rviz:=true &
sleep 2
gnome-terminal --title Moveit2_control -- bash -c "ros2 launch moveit2_control moveit2_control.launch.py; exec bash"
gnome-terminal --title Publisher -- bash -c "ros2 run py_pub talker; exec bash"
# pkill -u oliver -f "ros"