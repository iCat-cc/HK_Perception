#include "nodeletclass1.h"


unsigned char* nodeletclass1::pBGR24 = NULL;
 
nodeletclass1::nodeletclass1() {

}
 
// 重载虚函数，启动时自动调用
void nodeletclass1::onInit() {
    // 输出信息
    NODELET_DEBUG("Init nodelet...");
    ROS_INFO("Nodelet is OK for test");

    // 设置节点句柄
    ros::NodeHandle nh;
    // 设置图片节点
    LOGD("Here1.");
    image_transport::ImageTransport it(nh);
    LOGD("Here2.");

    LOGD("Initialize camere");
    ASSERT_OK( MV3D_RGBD_Initialize() );

    MV3D_RGBD_VERSION_INFO stVersion;
	ASSERT_OK( MV3D_RGBD_GetSDKVersion(&stVersion) );
	LOGD("dll version: %d.%d.%d", stVersion.nMajor, stVersion.nMinor, stVersion.nRevision);

	unsigned int nDevNum = 0;
    ASSERT_OK(MV3D_RGBD_GetDeviceNumber(DeviceType_USB, &nDevNum));
	LOGD("MV3D_RGBD_GetDeviceNumber success! nDevNum:%d.", nDevNum);
	ASSERT(nDevNum);

    // find device
	LOG("---------------------------------------------------------------\r\n");
	std::vector<MV3D_RGBD_DEVICE_INFO> devs(nDevNum);
    ASSERT_OK(MV3D_RGBD_GetDeviceList(DeviceType_USB, &devs[0], nDevNum, &nDevNum));
    for (unsigned int i = 0; i < nDevNum; i++) {  
		LOG("Index[%d]. SerialNum[%s] IP[%s] name[%s].\r\n", i, devs[i].chSerialNumber, devs[i].SpecialInfo.stNetInfo.chCurrentIp, devs[i].chModelName);
    }
    LOG("---------------------------------------------------------------");

    // open camera
    unsigned int nIndex = 0;
	void* handle = NULL;
    ASSERT_OK(MV3D_RGBD_OpenDevice(&handle, &devs[nIndex]));
    LOGD("OpenDevice success.");

        // find device info
	LOG("---------------------------------------------------------------\r\n");
	MV3D_RGBD_CALIB_INFO devs_info;
    ASSERT_OK(MV3D_RGBD_GetCalibInfo(handle,CoordinateType_RGB, &devs_info));
        LOG("stDistortion[");
        for(unsigned int m = 0; m < 12; m++){
            LOG("%f", devs_info.stDistortion.fData[m]);
        }
        LOG("]   \r\n  stIntrinsic[ ");
        for(unsigned int n = 0; n < 9; n++){
            LOG("%f", devs_info.stIntrinsic.fData[n]);
        }
        LOG("]   \r\n");
    LOG("---------------------------------------------------------------");

    // Start work
    ASSERT_OK(MV3D_RGBD_Start(handle));
	LOGD("Start work success.");

    // 设置主题的发布频率
    ros::Rate loop_rate(30);
    MV3D_RGBD_FRAME_DATA stFrameData = {0};
    //sensor_msgs::Image imageMsg;
    
    // 设置图片的发布者，第一个参数是话题的名称，第二个是缓冲区的大小（即消息队列的长度，在发布图像消息时消息队列的长度只能是1）
    image_transport::Publisher rgb_pub = it.advertise("rgb/image_raw", 1);
    image_transport::Publisher depth_pub = it.advertise("depth/image_raw", 1);

    // 图片节点进行主题的发布
    while(ros::ok()) {
        int nRet = MV3D_RGBD_FetchFrame(handle, &stFrameData, 5000);
        if (MV3D_RGBD_OK == nRet) {
            std::list<std::thread> lstThread;
            lstThread.push_back(std::thread(pubRGB, &stFrameData, &rgb_pub));
            lstThread.push_back(std::thread(pubDepth, &stFrameData, &depth_pub));
            for (auto& th : lstThread) {
                th.join();
            }
        }
        ros::spinOnce();
        // 按照设定的频率来将程序挂起
        loop_rate.sleep();
    }

    ASSERT_OK(MV3D_RGBD_Stop(handle));
    ASSERT_OK(MV3D_RGBD_CloseDevice(&handle));
    ASSERT_OK(MV3D_RGBD_Release());

    LOGD("onInit() done!");
}

// B分量
inline int YUV2B(unsigned char y, unsigned char u) {
    return (int)(y + 1.732446 * (u - 128));
}

// G分量
inline int YUV2G(unsigned char y, unsigned char u, unsigned char v) {
    return (int)(y - 0.698001 * (u - 128) - 0.703125 * (v - 128));
}

// R分量
inline int YUV2R(unsigned char y, unsigned char v) {
    return (int)(y + 1.370705  * (v - 128));
}

// YUV422转BGR8
int nodeletclass1::YUYVToBGR24_Native(unsigned char* pYUV, int width, int height) {
    if (pBGR24 == NULL) {
        pBGR24 = (uint8_t *)malloc(sizeof(uint8_t) * width * height * 3);
    }
	if (width < 1 || height < 1 || pYUV == NULL || pBGR24 == NULL)
		return 0;
	const long len = width * height;
	unsigned char* yData = pYUV;
	unsigned char* vData = pYUV;
	unsigned char* uData = pYUV;
	int y, x, k;

	int bgr[3];
	int yIdx, uIdx, vIdx, idx;
	for (y = 0; y < height; y++) {
		for (x = 0; x < width; x++) {
			yIdx = 2 * ((y*width) + x);
			uIdx = 4 * (((y*width) + x) >> 1) + 1;
			vIdx = 4 * (((y*width) + x) >> 1) + 3;

			bgr[0] = YUV2B(yData[yIdx], uData[uIdx]);
			bgr[1] = YUV2G(yData[yIdx], uData[uIdx], vData[vIdx]);                                  
			bgr[2] = YUV2R(yData[yIdx], vData[vIdx]);

			for (k = 0; k < 3; k++) {
				idx = (y * width + x) * 3 + k;
				if (bgr[k] >= 0 && bgr[k] <= 255)
					pBGR24[idx] = bgr[k];
				else
					pBGR24[idx] = (bgr[k] < 0) ? 0 : 255;
			}
		}
	}
	return 1;
}

void nodeletclass1::pubRGB(MV3D_RGBD_FRAME_DATA* stFrameData, image_transport::Publisher* rgb_pub) {
    for (int i = 0; i < stFrameData->nImageCount; i++) {
        if (ImageType_YUV422 == stFrameData->stImageData[i].enImageType) {
            LOGD("MV3D_RGBD_FetchFrame success: framenum (%d) height(%d) width(%d)  len (%d)!", stFrameData->stImageData[i].nFrameNum,
                stFrameData->stImageData[i].nHeight, stFrameData->stImageData[i].nWidth, stFrameData->stImageData[i].nDataLen);
            YUYVToBGR24_Native(stFrameData->stImageData[i].pData, stFrameData->stImageData[i].nWidth, stFrameData->stImageData[i].nHeight);
            LOGD("YUYVToBGR24_Native success.");
            cv::Mat image(stFrameData->stImageData[i].nHeight, stFrameData->stImageData[i].nWidth, CV_8UC3, pBGR24);
            sensor_msgs::ImagePtr imageMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
            rgb_pub->publish(*imageMsg);
        }
    }
}

void nodeletclass1::pubDepth(MV3D_RGBD_FRAME_DATA* stFrameData, image_transport::Publisher* depth_pub) {
    for (int i = 0; i < stFrameData->nImageCount; i++) {
        if (ImageType_Depth == stFrameData->stImageData[i].enImageType) {
            LOGD("MV3D_RGBD_FetchFrame success: framenum (%d) height(%d) width(%d)  len (%d)!", stFrameData->stImageData[i].nFrameNum,
                stFrameData->stImageData[i].nHeight, stFrameData->stImageData[i].nWidth, stFrameData->stImageData[i].nDataLen);
            cv::Mat depth_image(stFrameData->stImageData[i].nHeight, stFrameData->stImageData[i].nWidth, CV_16UC1, stFrameData->stImageData[i].pData);
            cv::Mat temp;
            cv::convertScaleAbs(depth_image, temp, 0.05);
            cv::Mat image;
            cv::applyColorMap(temp, image, cv::COLORMAP_JET);
            sensor_msgs::ImagePtr imageMsg = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::TYPE_8UC3, image).toImageMsg();
            depth_pub->publish(*imageMsg);
        }
    }
}
 
//nodelet的本质是把节点作为插件来调用，因此需要PLUGINLIB的宏定义、
//第一个参数是类名，第二个参数是父类
PLUGINLIB_EXPORT_CLASS(nodeletclass1, nodelet::Nodelet);