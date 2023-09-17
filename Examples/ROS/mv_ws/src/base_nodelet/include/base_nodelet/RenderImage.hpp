#ifndef SAMPLE_COMMON_RENDERIMAGE_HPP_
#define SAMPLE_COMMON_RENDERIMAGE_HPP_

#include "RenderWindow.hpp"

using namespace RenderImage;

static int parseFrame(MV3D_RGBD_FRAME_DATA* pstFrameData, RIFrameInfo* pDepth
                             ,RIFrameInfo* pRgb)
{
    for (unsigned int i = 0; i < pstFrameData->nImageCount; i++)
    {
        LOGD("parseFrame : framenum (%d) height(%d) width(%d)  len (%d)!", pstFrameData->stImageData[i].nFrameNum,
            pstFrameData->stImageData[i].nHeight, pstFrameData->stImageData[i].nWidth, pstFrameData->stImageData[i].nDataLen);

        if (ImageType_Depth == pstFrameData->stImageData[i].enImageType)
        {
            pDepth->enPixelType = RIPixelType_Coord3D_C16;
            pDepth->nFrameNum   = pstFrameData->stImageData[i].nFrameNum;
            pDepth->nHeight     = pstFrameData->stImageData[i].nHeight;
            pDepth->nWidth      = pstFrameData->stImageData[i].nWidth;
            pDepth->nFrameLength= pstFrameData->stImageData[i].nDataLen;
            pDepth->pData       = pstFrameData->stImageData[i].pData;

        }

        if (ImageType_RGB8_Planar == pstFrameData->stImageData[i].enImageType)
        {
            pRgb->enPixelType   = RIPixelType_RGB8_Planar;
            pRgb->nFrameNum     = pstFrameData->stImageData[i].nFrameNum;
            pRgb->nHeight       = pstFrameData->stImageData[i].nHeight;
            pRgb->nWidth        = pstFrameData->stImageData[i].nWidth;
            pRgb->nFrameLength  = pstFrameData->stImageData[i].nDataLen;
            pRgb->pData         = pstFrameData->stImageData[i].pData;
        }
    }

    return 0;
}


#endif