echo "-------------------------------------------"
echo "Building ROS nodes"
echo "-------------------------------------------"

source /home/cc/ORB_SLAM3_DENSE_LOOP1/Examples/ROS/ORB_SLAM3/build/devel/setup.bash

roslaunch ORB_SLAM3 run_rgbd_mv.launch
