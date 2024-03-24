#colcon build --symlink-install
cmds=(
  "source ~/.bashrc"
  "ros2 launch nav_bringup bringup_sim.launch.py world:=RMUL mode:=nav localization:=amcl"
)

#  "ros2 run nav_control nav_to_pose_sim"

for cmd in "${cmds[@]}"
do
	echo Current CMD : "$cmd"
	gnome-terminal -- bash -c "cd $(pwd);source install/setup.bash;$cmd;exec bash;"
	sleep 0.2
done


#icp amcl slam_toolbox

# ros2 launch rm_bringup bringup_sim.launch.py world:=RMUL mode:=mapping

# ros2 launch rm_bringup bringup_sim.launch.py world:=RMUL mode:=mapping

# ros2 launch rm_bringup bringup_sim.launch.py world:=RMUL mode:=nav localization:=slam_toolbox
