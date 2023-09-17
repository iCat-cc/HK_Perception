#include "myCamera.h"

MyCamera::MyCamera() : nDevNum(0), handle(NULL) {

}

MyCamera::~MyCamera() {
    ASSERT_OK(MV3D_RGBD_Stop(handle));
    ASSERT_OK(MV3D_RGBD_CloseDevice(&handle));
	ASSERT_OK(MV3D_RGBD_Release());
}

void MyCamera::init(unsigned int nIndex) {
    LOGD("Initialize camere");
    ASSERT_OK( MV3D_RGBD_Initialize() );

    ASSERT_OK( MV3D_RGBD_GetSDKVersion(&stVersion) );
	LOGD("dll version: %d.%d.%d", stVersion.nMajor, stVersion.nMinor, stVersion.nRevision);

    ASSERT_OK(MV3D_RGBD_GetDeviceNumber(DeviceType_USB, &nDevNum));
	LOGD("MV3D_RGBD_GetDeviceNumber success! nDevNum:%d.", nDevNum);
	ASSERT(nDevNum);

    // find device
	LOG("---------------------------------------------------------------\r\n");
	std::vector<MV3D_RGBD_DEVICE_INFO> devs(nDevNum);
    ASSERT_OK(MV3D_RGBD_GetDeviceList(DeviceType_USB, &devs[0], nDevNum, &nDevNum));
    for (unsigned int i = 0; i < nDevNum; i++)
    {  
		LOG("Index[%d]. SerialNum[%s] IP[%s] name[%s].\r\n", i, devs[i].chSerialNumber, devs[i].SpecialInfo.stNetInfo.chCurrentIp, devs[i].chModelName);
    }
    LOG("---------------------------------------------------------------");

    // open camera
    ASSERT_OK(MV3D_RGBD_OpenDevice(&handle, &devs[nIndex]));
    LOGD("OpenDevice success.");

    // Start work
    ASSERT_OK(MV3D_RGBD_Start(handle));
	LOGD("Start work success.");
}

void MyCamera::fetchFrame() {
    //ASSERT_OK(MV3D_RGBD_FetchFrame(handle, &stFrameData, 5000));
}
MyCamera::~MyCamera() {
    ASSERT_OK(MV3D_RGBD_Stop(handle));
    ASSERT_OK(MV3D_RGBD_CloseDevice(&handle));
	ASSERT_OK(MV3D_RGBD_Release());
}

void MyCamera::init(unsigned int nIndex) {
    LOGD("Initialize camere");
    ASSERT_OK( MV3D_RGBD_Initialize() );

    ASSERT_OK( MV3D_RGBD_GetSDKVersion(&stVersion) );
	LOGD("dll version: %d.%d.%d", stVersion.nMajor, stVersion.nMinor, stVersion.nRevision);

    ASSERT_OK(MV3D_RGBD_GetDeviceNumber(DeviceType_USB, &nDevNum));
	LOGD("MV3D_RGBD_GetDeviceNumber success! nDevNum:%d.", nDevNum);
	ASSERT(nDevNum);

    // find device
	LOG("---------------------------------------------------------------\r\n");
	std::vector<MV3D_RGBD_DEVICE_INFO> devs(nDevNum);
    ASSERT_OK(MV3D_RGBD_GetDeviceList(DeviceType_USB, &devs[0], nDevNum, &nDevNum));
    for (unsigned int i = 0; i < nDevNum; i++)
    {  
		LOG("Index[%d]. SerialNum[%s] IP[%s] name[%s].\r\n", i, devs[i].chSerialNumber, devs[i].SpecialInfo.stNetInfo.chCurrentIp, devs[i].chModelName);
    }
    LOG("---------------------------------------------------------------");

    // open camera
    ASSERT_OK(MV3D_RGBD_OpenDevice(&handle, &devs[nIndex]));
    LOGD("OpenDevice success.");

    // Start work
    ASSERT_OK(MV3D_RGBD_Start(handle));
	LOGD("Start work success.");
}

void MyCamera::fetchFrame() {
    ASSERT_OK(MV3D_RGBD_FetchFrame(handle, &stFrameData, 5000));
}