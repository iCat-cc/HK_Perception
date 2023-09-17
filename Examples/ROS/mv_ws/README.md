ROS1集成及示例代码开发

## 一、本文开发环境

1. Ubuntu 18.04
2. ROS Melodic（注意：ROS版本须与Ubuntu版本对应，否则安装会出现问题）
3. RoboWare Studio（非必需）
4. 悉灵相机MV-EB435i
5. SDK_Mv3dRgbd_V1.0.0_Linux_220622_837455

## 二、ROS下集成悉灵相机并获取图像数据

### （一）基于话题通信方式

在话题（Topic）通信机制里，主要有三个角色：**发布者**，**订阅者**，**Master**；根据 Master 的参与主要分为两个阶段：**连接前的准备**，**连接和通信**。


1. 第一阶段：建立前的准备

在建立连接阶段，主要是解决发布者 与 订阅者 进行匹配进而连接的问题；在这个阶段主要分为五步：

（1）发布者（Talker）启动，通过RPC向 ROS Master 注册发布者的信息，包括：发布者节点信息，话题名，话题缓存大小等；Master 会将这些信息加入注册列表中；
（2）订阅者（Listener）启动，通过 RPC 向 ROC Master 注册订阅者信息，包括：订阅者节点信息，话题名等；Master 会将这些信息加入注册列表；
（3）Master 进行节点匹配：Master 会根据订阅者提供的信息，在注册列表中查找匹配的发布者；如果没有发布者（Talker），则等待发布者（Talker）的加入；如果找到匹配的发布者（Talker），则会主动把发布者（Talker）（有可能是很多个 Talker）的地址通过 RPC 传送给订阅者（Listener）节点；
（4）Listener 接收到 Master 的发出的 Talker 的地址信息，尝试通过 RPC 向 Talker 发出连接请求（信息包括：话题名，消息类型以及通讯协议（TCP/UDP））；
（5）Talker 收到 Listener 发出的连接请求后，通过 RPC 向 Listener 确认连接请求（包含的信息为自身 TCP 地址信息）；
至此，Talker 和 Listener 做好了连接前的准备工作。在这个过程中，Master 起到了 牵线搭桥 的作用；

2. 第二阶段：连接和通信

在发布消息阶段，主要解决的是发布者(Talker) 如何发布消息进入传递给 订阅者(Listener) 的过程。在这个过程中，完全是 Talker 和 Listener 两者之间的信息单向流动，Master 并未参与其中；主要分为两步：

（1）Listener 接收到 Talker 的确认消息后，使用 TCP 尝试与 Talker 建立网络连接；
（2）成功连接之后，Talker 开始向 Listener 发布话题消息数据；
至此，完成了 Talker 向 Listener 发布消息的过程；Master 在这个阶段并不参与两者之间的数据传递；
需要注意的是，有可能多个 Talker 连接一个 Listener，也有可能是一个 Talker 连接上多个 Listener；

### （二）基于nodelet通信方式

本文主要介绍基于Nodelet通信方式的实现，因为Nodelet能够提供一种在单机器单进程运行多个算法而不会在进程中传递消息时产生复制成本的方法，这正好符合图像传输时数据量大，实时性要求高的特点。基本原理就是让多个算法程序在一个进程中用 shared_ptr 实现零拷贝通信（zero copy transport），以降低因为传输大数据而损耗的时间。Nodelet可以将多个node捆绑在一起管理，使得同一个manager里面的topic的数据传输更快。

**实现过程**

1. 概述

   Nodelet节点与传统的ros节点不同，其源文件中不存放main函数，而是定义一个类，在编译时，不是编译成可执行文件，而是编译成库文件。

2. 项目package文件树

3. 创建package

   在catkin_ws工作空间（预先创建并初始化）下，创建package

   ```shell
   catkin_create_pkg base_nodelet
   ```

4. 分别创建include、lib、src、plugins、launch文件夹

   ```shell
   cd ./catkin_ws/
   mkdir include lib src plugins launch
   ```

   把SDK中的头文件和库文件分别放入include和lib文件夹

5. 转到src文件夹下，编写源代码

   ```
   cd src
   ```

   创建文件nodeletclass1.h

   ```c++
   #include <nodelet/nodelet.h>

   class nodeletclass1 :public nodelet::Nodelet // 继承父类nodelet::Nodelet
   {
   public:
     nodeletclass1();
   public:
      // 这个虚函数，在启动本Nodelet节点时，自动调用
     virtual void onInit();
     // 用于YUV422转BGR
     static int YUYVToBGR24_Native(unsigned char* pYUV, int width, int height);

   static unsigned char* pBGR24;
   };
   ```

   创建文件nodeletclass1.cpp

   ```c++
   #include "nodeletclass1.h"
   #include <pluginlib/class_list_macros.h>
   #include <ros/ros.h>
    
   nodeletclass1::nodeletclass1()
   {
    
   }
    
   //重载虚函数，启动时自动调用
   void nodeletclass1::onInit()
   {
       /*
       连接并启动相机，略
       */
   	
       // 设置图片节点
       image_transport::ImageTransport it(nh);

       // 设置图片的发布者，第一个参数是话题的名称，第二个是缓冲区的大小（即消息队列的长度，在发布图像消息时消息队列的长度只能是1）
       image_transport::Publisher rgb_pub = it.advertise("rgb/image_raw", 1);
       image_transport::Publisher depth_pub = it.advertise("depth/image_raw", 1);
       image_transport::Publisher lir_pub = it.advertise("lir/image_raw", 1);
       image_transport::Publisher rir_pub = it.advertise("rir/image_raw", 1);

       // 设置主题的发布频率
       ros::Rate loop_rate(30);

       // 图片节点进行主题的发布
       while(ros::ok())
       {
         	int nRet = MV3D_RGBD_FetchFrame(handle, &stFrameData, 5000);
           if (MV3D_RGBD_OK == nRet)
           {
               for (int i = 0; i < stFrameData.nImageCount; i++)
               {
                   // 打印图像信息
                   LOGD("MV3D_RGBD_FetchFrame success: framenum (%d) height(%d) width(%d)  len (%d)!", stFrameData.stImageData[i].nFrameNum,
                        stFrameData.stImageData[i].nHeight, stFrameData.stImageData[i].nWidth, stFrameData.stImageData[i].nDataLen);

                   // 发布RGB图
                   if (ImageType_YUV422 == stFrameData.stImageData[i].enImageType)
                   {
                       // 将yuv422格式转换为BGR格式
                       YUYVToBGR24_Native(stFrameData.stImageData[i].pData, stFrameData.stImageData[i].nWidth, stFrameData.stImageData[i].nHeight);
                       LOGD("YUYVToBGR24_Native success.");
                       // 将图像转成OpenCV中的mat格式
                       cv::Mat image(stFrameData.stImageData[i].nHeight, stFrameData.stImageData[i].nWidth, CV_8UC3, pBGR24);
                       // 利用cv_bridge库将mat格式转换为ROS中的sensor_msgs::Image格式
                       sensor_msgs::ImagePtr imageMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
                       // 发布图像数据
                       rgb_pub.publish(*imageMsg);
                   }

                   // 发布深度图
                   if (ImageType_Depth == stFrameData.stImageData[i].enImageType)
                   {
                       cv::Mat image(stFrameData.stImageData[i].nHeight, stFrameData.stImageData[i].nWidth, CV_8UC2, stFrameData.stImageData[i].pData);
                       sensor_msgs::ImagePtr imageMsg = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::TYPE_16UC1, image).toImageMsg();
                       depth_pub.publish(*imageMsg);
                   }

                   // 发布左IR图
                   if (i == 2)
                   {
                       cv::Mat image(stFrameData.stImageData[i].nHeight, stFrameData.stImageData[i].nWidth, CV_8UC1, stFrameData.stImageData[i].pData);
                       sensor_msgs::ImagePtr imageMsg = cv_bridge::CvImage(std_msgs::Header(), "mono8", image).toImageMsg();
                       lir_pub.publish(*imageMsg);
                   }

                   // 发布右IR图
                   if (i == 3)
                   {
                       cv::Mat image(stFrameData.stImageData[i].nHeight, stFrameData.stImageData[i].nWidth, CV_8UC1, stFrameData.stImageData[i].pData);
                       sensor_msgs::ImagePtr imageMsg = cv_bridge::CvImage(std_msgs::Header(), "mono8", image).toImageMsg();
                       rir_pub.publish(*imageMsg);
                   }
               }
           }
           ros::spinOnce();
           // 按照设定的频率来将程序挂起
           loop_rate.sleep();
       }
   	/*
       连接并启动相机，略
       */
   }
   ```


   //nodelet的本质是把节点作为插件来调用，因此需要PLUGINLIB的宏定义，通常放在cpp文件结尾
   //第一个参数是类名，第二个参数是父类
   PLUGINLIB_EXPORT_CLASS(nodeletclass1, nodelet::Nodelet);

6.  进入plugins文件夹，创建插件的引用xml文件

   ```
   cd plugins
   ```

   创建nodelet_plugins.xml

   ```xml
   <!--这里的path="",修改成path="lib/lib{项目名}",
       项目名就是CMakeLists.txt里面定义的project(base_nodelet)
       我这里就是path="lib/libbase_nodelet"
   -->
    
   <library path="lib/libbase_nodelet" >
    
     <!-- name: launch文件里面 load 后面接着的插件名
          type: c++文件定义的类名
          如 name="aaa/nodeletclass1",那么，launch文件对应启动如下：
          <node pkg="nodelet" type="nodelet" name="nodeletclass1"
          args="load aaa/nodeletclass1 nodelet_manager" output="screen">
     -->
     <class name="aaa/nodeletclass1" type="nodeletclass1" base_class_type="nodelet::Nodelet">
     <description>
     This is my nodelet.
     </description>
     </class>
   </library>
   ```

7. 回到项目根目录

   ```shell
   cd base_nodelet
   ```

   修改package.xml文件

   添加对其他package的依赖：

   ```xml
   <build_depend>nodelet</build_depend>
   <build_depend>roscpp</build_depend>
   <exec_depend>nodelet</exec_depend>
   <exec_depend>roscpp</exec_depend>
   ```

   在<export></export>标签中间加入如下内容：

   ```xml
   <nodelet plugin="${prefix}/plugins/nodelet_plugins.xml" />
   ```

8. 编写CMakeLists.txt

   由于Nodelet不是编译成可执行文件，而是库文件。

   所以加入如下语句：

   ```
   add_library(${PROJECT_NAME} src/nodeletclass1.cpp)
   ```

   同时加入如下语句：

   ```
   add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
   target_link_libraries(${PROJECT_NAME}
     ${catkin_LIBRARIES}
     /opt/ros/melodic/lib/libimage_transport.so # 
     ${PROJECT_SOURCE_DIR}/lib/libMv3dRgbd.so
   )
   ```

9. 编写launch文件

   进入launch目录

   ```shell
   cd launch
   ```

   创建nodeletclass1.launch

   ```xml
   <launch>
     <node pkg="nodelet" type="nodelet" name="nodelet_manager"  args="manager" output="screen"/>
    
     <node pkg="nodelet" type="nodelet" name="nodeletclass1" args="load aaa/nodeletclass1 nodelet_manager" output="screen">
     </node>

     <!-- display in Rviz -->
     <node pkg="rviz" type="rviz" name="rviz" args="-d $(find base_nodelet)/config/HikCamera.rviz"/>
   </launch>
   ```

   launch文件启动了两个节点

   一个是nodelet manager，并重命名为 nodelet_manager

   另外一个则是编写完成的Nodelet节点插件

10. 编译与测试

  添加环境环境变量（注意必须添加到root用户下的环境变量，因为USB相关的读写操作需要root权限，sudo也不行）

  ```
  export LD_LIBRARY_PATH=/UserDemoPath:$LD_LIBRARY_PATH
  ```

  转入工作空间根目录

  ```
  catkin_make
  ```

  编译完成，启动终端后**切换到root用户**，执行：

  ```shell
  source devel/setup.bash
  ```

  启动launch文件

  ```
  roslaunch base_nodelet nodeletclass1.launch
  ```