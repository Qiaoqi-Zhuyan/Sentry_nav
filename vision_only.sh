#colcon build --symlink-install
cmds=(  "ros2 launch nav_bringup bringup_vision.launch.py"
	"ros2 launch foxglove_bridge foxglove_bridge_launch.xml"
	"foxglove-studio"
)


for cmd in "${cmds[@]}"
do
	echo Current CMD : "$cmd"
	gnome-terminal -- bash -c "cd $(pwd);source install/setup.bash;$cmd;exec bash;"
	sleep 0.2
done
