#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "../include/base_nodelet/common.hpp"
#include <sensor_msgs/Image.h>
#include <thread>

class nodeletclass1 : public nodelet::Nodelet { // 继承父类nodelet::Nodelet
public:
  nodeletclass1();
public:
  virtual void onInit();   // 这个虚函数，在启动本Nodelet节点时，自动调用
  static int YUYVToBGR24_Native(unsigned char* pYUV, int width, int height);
  static void pubRGB(MV3D_RGBD_FRAME_DATA* stFrameData, image_transport::Publisher* rgb_pub);
  static void pubDepth(MV3D_RGBD_FRAME_DATA* stFrameData, image_transport::Publisher* depth_pub);
static unsigned char* pBGR24;
};