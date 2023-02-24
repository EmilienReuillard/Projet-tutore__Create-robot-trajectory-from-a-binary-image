echo "Execution des commandes pour rviz configuration scara dxl... "
echo " "

ros2 control load_controller scara_position_controller --set-state active 

ros2 control load_controller scara_trajectory_controller --set-state active

ros2 control set_controller_state scara_trajectory_controller active

ros2 control set_controller_state scara_position_controller inactive

ros2 control set_controller_state scara_joint_velocity_controller inactive

ros2 control set_controller_state scara_trajectory_controller active