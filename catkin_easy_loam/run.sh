echo "Aloam starting"
manual_source="source devel/setup.bash"
dataset_path="/home/g/gang/github/easy_aloam/catkin_easy_loam/nsh_indoor_outdoor.bag"

gnome-terminal -t "run" -x bash -c "${manual_source};roscore;exec bash;"
gnome-terminal -t "run" -x bash -c "${manual_source};rosrun easy_loam sensor_read_node;exec bash;"
gnome-terminal --window --geometry=+800+100 -t "demo" -x bash -c "rosbag play ${dataset_path}";

