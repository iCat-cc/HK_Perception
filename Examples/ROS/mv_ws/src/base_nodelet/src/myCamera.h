#ifndef MYCAMERA_H
#define MYCAMERA_H

#include "../include/base_nodelet/common.hpp"

class MyCamera {
public:
    MyCamera();
    ~MyCamera();
    void init(unsigned int nIndex = 0);
    void fetchFrame();

private:
    MV3D_RGBD_VERSION_INFO stVersion;
    unsigned int nDevNum;
    //std::vector<MV3D_RGBD_DEVICE_INFO> devs;
    // unsigned int nIndex;
	void* handle;

public:
    MV3D_RGBD_FRAME_DATA stFrameData;
};

#endif