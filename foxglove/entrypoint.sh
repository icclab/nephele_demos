mkdir -p /foxglove
touch /foxglove/default-layout.json
index_html=$(cat index.html)
replace_pattern='/*FOXGLOVE_STUDIO_DEFAULT_LAYOUT_PLACEHOLDER*/'
replace_value=$(cat /foxglove/default-layout.json)
echo "${index_html/"$replace_pattern"/$replace_value}" > index.html
echo "source /opt/ros/humble/setup.bash"
echo "source  /home/ros/colcon_ws/install/setup.bash"
echo "export ROS_PACKAGE_PATH=/home/ros/colcon_ws/install:/opt/ros/humble/share"
# Continue
exec "$@"
