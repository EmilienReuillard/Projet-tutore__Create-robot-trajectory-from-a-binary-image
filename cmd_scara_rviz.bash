echo "Execution des commandes pour rviz ... "
echo " "

echo "execution de :  ros2 control load_controller joint_state_broadcaster --set-state active"
ros2 control load_controller joint_state_broadcaster --set-state active
echo " "

echo "execution de :  ros2 control load_controller scara_position_controller --set-state active"
ros2 control load_controller scara_position_controller --set-state active
echo " "

echo "execution de :  ros2 control load_controller scara_trajectory_controller --set-state active"
ros2 control load_controller scara_trajectory_controller --set-state active
echo "  "

echo " execution de : ros2 control set_controller_state scara_position_controller inactive"
ros2 control set_controller_state scara_position_controller inactive
echo "  "

echo " execution de : ros2 control set_controller_state scara_trajectory_controller active "
ros2 control set_controller_state scara_trajectory_controller active
echo " "

echo "Visualisation que tout est bon (ros2 control list_controllers) : "
ros2 control list_controllers