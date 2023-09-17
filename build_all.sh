echo "-------------------------------------------"
echo "Building ROS nodes"
echo "-------------------------------------------"

cd Examples/ROS/ORB_SLAM3
mkdir build
cd build
cmake .. -DROS_BUILD_TYPE=Release
make -j

echo "-------------------------------------------"
echo "Building MV-EB435i nodes"
echo "-------------------------------------------"

cd ../../mv_ws
catkin_make

echo "-------------------------------------------"
echo "Building Environment nodes"
echo "-------------------------------------------"

cd ../env_ws
catkin_make
