echo "-------------------------------------------"
echo "Building ROS nodes"
echo "-------------------------------------------"

cd Examples/ROS/ORB_SLAM3
mkdir build
cd build
cmake .. -DROS_BUILD_TYPE=Release
make -j