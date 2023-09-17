/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/PointCloud2.h>
#include<pcl/point_cloud.h>  
#include <pcl_conversions/pcl_conversions.h>
//#include<pcl/io/pcd_io.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM):mpSLAM(pSLAM){}

    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);

    ORB_SLAM3::System* mpSLAM;

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM3 RGBD path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::RGBD,true);

    ImageGrabber igb(&SLAM);

    //ros::NodeHandle nh;

        //加上"~"号后可以传入参数
    ros::NodeHandle nh("~");
 
    std::string rgb_topic = nh.param<std::string>("rgb", "/camera/rgb/image_color");
    std::string depth_topic = nh.param<std::string>("depth", "/camera/depth/image");
 
    cout << "rgb: " << rgb_topic << endl;
    cout << "depth: " << depth_topic << endl;
 
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, rgb_topic, 100);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, depth_topic, 100);


    // message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/rgb/image_raw", 100);
    // message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/depth_registered/image_raw", 100);
    // 以下为tum rgbd 数据集的topic
    //message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/rgb/image_color", 100);
    //message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/depth/image", 100);
    
    //message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/rgb/image_raw", 100);
    //message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/depth/image_raw", 100);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2));



 //TODO OCTOMAP添加

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr global_map(new pcl::PointCloud<pcl::PointXYZRGBA>);

    global_map = SLAM.getGlobalMap();//mpPointCloudMapping1->getGlobalMap();

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr global_map_copy(new pcl::PointCloud<pcl::PointXYZRGB>);
    //数据格式转换
    cout<<"-----------------------------------------------------------"<<endl;
    cout <<"ros is running "<<endl;
    //ros::NodeHandle nh1;
    while (ros::ok())
    {

        pcl::copyPointCloud(*global_map, *global_map_copy);

        ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("/orbslam3_dense/output", 10);

        //pcl::PointCloud<pcl::PointXYZ> cloud; 
        sensor_msgs::PointCloud2 output;
        //pcl::io::loadPCDFile (path, cloud);
        
        //cout << "----^^^------" << endl;
        //cout << *global_map_copy << endl;
        Eigen::Affine3f transform = Eigen::Affine3f::Identity();
        transform.rotate(Eigen::AngleAxisf(M_PI/2, Eigen::Vector3f(0,1,0)));
        transform.rotate(Eigen::AngleAxisf(-M_PI/2, Eigen::Vector3f(0,0,1)));
        //transform.rotate(Eigen::AngleAxisf(M_PI, Eigen::Vector3f(1,0,0)));
        pcl::transformPointCloud(*global_map_copy, *global_map_copy, transform);
    	
        pcl::toROSMsg(*global_map_copy,output);// 转换成ROS下的数据类型 最终通过topic发布

        output.header.stamp=ros::Time::now();
        output.header.frame_id  ="odom";
        //output.header.frame_id  ="map";


        ros::Rate loop_rate(10);

        pcl_pub.publish(output);
        ros::spinOnce();
        loop_rate.sleep();
    }
    //TODO 结束


    
    //ros::spin();
    SLAM.save("/home/cc/ORB_SLAM3_DENSE_LOOP1/地图.pcd");


    // Stop all threads
    SLAM.Shutdown();
        // Save camera trajectory
    //SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    //SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("/home/cc/ORB_SLAM3_DENSE_LOOP1/关键帧轨迹.txt");
        // Save camera trajectory**
    SLAM.SaveTrajectoryTUM("/home/cc/ORB_SLAM3_DENSE_LOOP1/相机轨迹.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());
}


