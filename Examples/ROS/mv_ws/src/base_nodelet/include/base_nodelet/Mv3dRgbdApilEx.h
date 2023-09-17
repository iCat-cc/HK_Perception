/***************************************************************************************************
*
* 版权信息：版权所有 (c) 2022, 杭州海康机器人技术有限公司, 保留所有权利
*
* 文件名称：Mv3dRgbdApilEx.h
* 摘    要: RGBD 相机SDK，拓展接口定义
*
* 当前版本：1.1.0
* 作    者: 机器视觉SDK团队
* 日    期：2022.4.26
* 备    注：新建
****************************************************************************************************/

#ifndef _MV3D_RGBD_API_EX_H_
#define _MV3D_RGBD_API_EX_H_

#include "Mv3dRgbdApi.h"
#include "Mv3dRgbdDefine.h"


#ifdef __cplusplus
extern "C" {
#endif 

/************************************************************************
*  @fn         MV3D_RGBD_API MV3D_RGBD_STATUS MV3D_RGBD_CamIsAccessible(IN MV3D_RGBD_DEVICE_INFO* pstDevInfo, IN unsigned int nAccessMode)
*  @brief      判断相机的访问权限
*  @param      MV_STEREOCAM_NET_INFO* pstDevInfo                [IN]        相机信息
*  @param      unsigned int * nAccessMode                       [IN]        相机的访问权限      //Eg: MV3D_RGBD_ACCESS_Exclusive
*  @return    可达，返回true；不可达，返回false 

*  @fn         MV3D_RGBD_API MV3D_RGBD_STATUS MV3D_RGBD_CamIsAccessible(IN MV3D_RGBD_DEVICE_INFO* pstDevInfo, IN unsigned int nAccessMode)
*  @brief      Get camera's Access privileges
*  @param      MV3D_RGBD_DEVICE_INFO* pstDevInfo                [IN]        camera info 
*  @param       unsigned int * nAccessMode                      [IN]        camera's Access privileges    //Eg: MV3D_RGBD_ACCESS_Exclusive
*  @return     Success, return true. Failure, false
************************************************************************/    
MV3D_RGBD_API MV3D_RGBD_STATUS  MV3D_RGBD_CamIsAccessible(MV3D_RGBD_DEVICE_INFO* pstDevInfo,unsigned int nAccessMode);

/************************************************************************
*  @fn     MV3D_RGBD_GetCameraXML
*  @brief  获取相机xml
*  @param  handle                      [IN]            设备句柄
*  @param  pData                       [IN OUT]        接收xml缓存
*  @param  nDataSize                   [IN]............缓存长度
*  @param  pnDataLen                   [OUT]...........数据实际长度
*  @return 成功,返回MV3D_RGBD_OK,失败,返回错误码

*  @fn     MV3D_RGBD_GetCameraXML
*  @brief  Get Camera XML
*  @param  handle                      [IN]            Device handle
*  @param  pData                       [IN OUT]        recieve xml buf point
*  @param  nDataSize                   [IN]............buffer size
*  @param  pnDataLen                   [OUT]...........get data len
*  @return Success, return MV3D_RGBD_OK. Failure, return error code
************************************************************************/
MV3D_RGBD_API MV3D_RGBD_STATUS MV3D_RGBD_GetCameraXML(void * handle,unsigned char* pData,unsigned int nDataSize,unsigned int* pnDataLen);

/***********************************************************************
 *  @fn     MV3D_RGBD_XML_GetNodeAccessMode
 *  @brief  获得当前节点的访问模式
 *  @param  handle                      [IN]            设备句柄
 *  @param  pstrName                    [IN]            节点名称
 *  @param  pAccessMode                 [OUT]           节点的访问模式
 *  @return 成功，返回MV3D_RGBD_OK；错误，返回错误码

 *  @fn     MV3D_RGBD_XML_GetNodeAccessMode
 *  @brief  Get Access mode of cur node
 *  @param  handle                      [IN]            Device handle
 *  @param  pstrName                    [IN]            Name of node
 *  @param  pAccessMode                 [OUT]           Access mode of the node
 *  @return Success, return MV3D_RGBD_OK. Failure, return error code
 ***********************************************************************/
MV3D_RGBD_API MV3D_RGBD_STATUS MV3D_RGBD_XML_GetNodeAccessMode(void* handle,const char * pstrName,MV3D_RGBD_XML_AccessMode *pAccessMode);

/***********************************************************************
 *  @fn     MV3D_RGBD_SetImageNodeNum
 *  @brief  设置SDK内部图像缓存节点个数，大于等于1，在抓图前调用
 *  @param  handle                      [IN]            设备句柄
 *  @param  nNum                        [IN]            缓存节点个数
 *  @return 成功，返回MV3D_RGBD_OK；错误，返回错误码
 
 *  @fn     MV3D_RGBD_SetImageNodeNum
 *  @brief  Set the number of the internal image cache nodes in SDK, Greater than or equal to 1, to be called before the capture
 *  @param  handle                      [IN]            Device handle
 *  @param  nNum                        [IN]            Image Node Number
 *  @return Success, return MV3D_RGBD_OK. Failure, return error code
 ***********************************************************************/
MV3D_RGBD_API MV3D_RGBD_STATUS MV3D_RGBD_SetImageNodeNum(void* handle,unsigned int nNodeCount);

/************************************************************************
 *  @fn     MV3D_RGBD_InvalidateNodes
 *  @brief  清除GenICam节点缓存
 *  @param  handle                      [IN]            设备句柄
 *  @return 成功，返回MV3D_RGBD_OK；错误，返回错误码
 
 *  @fn     MV3D_RGBD_InvalidateNodes
 *  @brief  Invalidate GenICam Nodes
 *  @param  handle                      [IN]            Device handle
 *  @return Success, return MV3D_RGBD_OK. Failure, return error code
 ************************************************************************/
MV3D_RGBD_API MV3D_RGBD_STATUS MV3D_RGBD_InvalidateNodes(void* handle);

/************************************************************************
*  @fn     MV3D_RGBD_ReadAddressMem
*  @brief  读内存
*  @param  handle                      [IN]            设备句柄
*  @param  pBuffer                     [IN][OUT]       作为返回值使用，保存读到的内存值（内存值是按照大端模式存储的）
*  @param  nRegAddress                 [IN]            待读取的内存地址，该地址可以从设备的Camera.xml文件中获取，形如xxx_RegAddr的xml节点值
                                                       （设备的Camera.xml文件会在设备打开之后自动生成在应用程序的当前目录中）
*  @param  nLength                     [IN]            待读取的内存长度
*  @return 成功,返回MV3D_RGBD_OK,失败,返回错误码

*  @fn     MV3D_RGBD_ReadAddressMem
*  @brief  Read Memory
*  @param  handle                      [IN]            Device Handle
*  @param  pBuffer                     [IN][OUT]       Used as a return value, save the read-in memory value ( Memory value is stored in accordance with the big end model)
*  @param  nRegAddress                 [IN]            Memory address to be read, which can be obtained from the Camera.xml file of the device, the form xml node value of xxx_RegAddr
                                                       (Camera.xml file of device is automatically generated in the application's current directory after the device is opened)
*  @param  nLength                     [IN]            Length of the memory to be read
*  @return Success, return MV3D_RGBD_OK. Failure, return error code 
************************************************************************/
MV3D_RGBD_API MV3D_RGBD_STATUS MV3D_RGBD_ReadAddressMem(void * handle,void *pBuffer,int64_t nRegAddress,int64_t nLength);

/************************************************************************
*  @fn     MV3D_RGBD_WriteAddressMem
*  @brief  写内存
*  @param  handle                      [IN]            设备句柄
*  @param  pBuffer                     [IN]            待写入的内存值（注意内存值要按照大端模式存储）
*  @param  nRegAddress                 [IN]            待写入的内存地址，该地址可以从设备的Camera.xml文件中获取，形如xxx_RegAddr的xml节点值
                                                       （设备的Camera.xml文件会在设备打开之后自动生成在应用程序的当前目录中）
*  @param  nLength                     [IN]            待写入的内存长度
*  @return 成功,返回MV3D_RGBD_OK,失败,返回错误码

*  @fn     MV3D_RGBD_WriteAddressMem
*  @brief  Write Memory
*  @param  handle                      [IN]            Device Handle
*  @param  pBuffer                     [IN]            Memory value to be written ( Note the memory value to be stored in accordance with the big end model)
*  @param  nRegAddress                 [IN]            Memory address to be written, which can be obtained from the Camera.xml file of the device, the form xml node value of xxx_RegAddr
                                                       (Camera.xml file of device is automatically generated in the application's current directory after the device is opened)
*  @param  nLength                     [IN]            Length of the memory to be written
*  @return Success, return MV3D_RGBD_OK. Failure, return error code 
************************************************************************/
MV3D_RGBD_API MV3D_RGBD_STATUS MV3D_RGBD_WriteAddressMem(void * handle,void *pBuffer,int64_t nRegAddress,int64_t nLength);

/************************************************************************
*  @fn     MV3D_RGBD_FileAccessRead()
*  @brief  从相机读取文件
*  @param  handle                [IN]           句柄地址
*  @param  pstFileAccess         [IN]           文件存取结构体
*  @return 成功，返回MV3D_RGBD_OK；错误，返回错误码 

*  @fn     MV3D_RGBD_FileAccessRead()
*  @brief  Read the file from the camera
*  @param  handle                [IN]           Handle
*  @param  pstFileAccess         [IN]           File access structure
*  @return Success, return MV3D_RGBD_OK. Failure, return error code
************************************************************************/
MV3D_RGBD_API MV3D_RGBD_STATUS MV3D_RGBD_FileAccessRead(void* handle,MV3D_RGBD_FILE_ACCESS * pstFileAccess);

/************************************************************************
*  @fn     MV3D_RGBD_FileAccessWrite()
*  @brief  将文件写入相机
*  @param  handle                [IN]           句柄地址
*  @param  pstFileAccess         [IN]           文件存取结构体
*  @return 成功，返回MV3D_RGBD_OK；错误，返回错误码 

*  @fn     MV3D_RGBD_FileAccessWrite()
*  @brief  Write the file to camera
*  @param  handle                [IN]           Handle
*  @param  pstFileAccess         [IN]           File access structure
*  @return Success, return MV3D_RGBD_OK. Failure, return error code
************************************************************************/
MV3D_RGBD_API MV3D_RGBD_STATUS MV3D_RGBD_FileAccessWrite(void* handle,MV3D_RGBD_FILE_ACCESS * pstFileAccess);

/************************************************************************
 *  @fn     MV3D_RGBD_SetGvcpTimeout
 *  @brief  设置GVCP命令超时时间
 *  @param  handle                      [IN]            设备句柄
 *  @param  nMillisec                   [IN]            超时时间，以毫秒位单位，范围：0-10000
 *  @return 成功，返回MV3D_RGBD_OK；错误，返回错误码 

 *  @fn     MV_GIGE_SetGvcpTimeout
 *  @brief  Set GVCP cammand timeout
 *  @param  handle                      [IN]            Device handle
 *  @param  nMillisec                   [IN]            Timeout, ms as unit, range: 0-10000
 *  @return Success, return MV3D_RGBD_OK. Failure, return error code
 ************************************************************************/
MV3D_RGBD_API MV3D_RGBD_STATUS MV3D_RGBD_SetGvcpTimeout(void* handle, unsigned int nMillisec);

/************************************************************************
*  @fn     MV3D_RGBD_GetGvcpTimeout
*  @brief  获取GVCP命令超时时间
*  @param  handle                      [IN]            设备句柄
*  @param  pMillisec                   [IN]            超时时间指针，以毫秒位单位
*  @return 成功，返回MV3D_RGBD_OK；错误，返回错误码 

*  @fn     MV_GIGE_GetGvcpTimeout
*  @brief  Get GVCP cammand timeout
*  @param  handle                      [IN]            Device handle
*  @param  pMillisec                   [IN]            Timeout, ms as unit
*  @return Success, return MV3D_RGBD_OK. Failure, return error code
************************************************************************/
MV3D_RGBD_API MV3D_RGBD_STATUS MV3D_RGBD_GetGvcpTimeout(void* handle, unsigned int* pMillisec);




#ifdef __cplusplus
}
#endif 

#endif //_MV3D_RGBD_API_EX_H_