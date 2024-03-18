cmds=( "ros2 launch foxglove_bridge foxglove_bridge_launch.xml"
       "foxglove-studio"
)


for cmd in "${cmds[@]}"
do
	echo Current CMD : "$cmd"
	gnome-terminal -- bash -c "cd $(pwd);source install/setup.bash;$cmd;exec bash;"
	sleep 0.2
done

#ros2 launch nav_bringup bringup_real.launch.py mode:=mapping

#ros2 launch rm_bringup bringup_real.launch.py world:= mode:=nav localization:=amcl

#  "ros2 launch nav_bringup bringup_real.launch.py rviz:=true world:=scan3 mode:=nav localization:=amcl "

