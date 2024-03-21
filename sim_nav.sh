#colcon build --symlink-install
cmds=(  "ros2 launch nav_bringup bringup_sim.launch.py world:=RMUL mode:=nav localization:=amcl"
)


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
