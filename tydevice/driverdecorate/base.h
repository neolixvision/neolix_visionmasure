#ifndef NEOLIX_BASE_H
#define NEOLIX_BASE_H
#include <opencv2/opencv.hpp>
#include "../tuyang/TY_API.h"
namespace neolix{
#define RECT_LEFT_UP_X_POINT 145
#define RECT_LEFT_UP_Y_POINT 235
#define RECT_BOX_WIDTH       335
#define RECT_BOX_HEIGHT      230
//========相机内参=========
#define  NEICAN_FU  529.86016845703125f
#define  NEICAN_FV  529.86016845703125f
#define  NEICAN_U0  327.42376708984375f
#define  NEICAN_V0  259.32891845703125f
//==========测量平台到摄像头九个区域的距离=====
//#define  AREA1 1621.363636f
//#define  AREA2 1617.519481f
//#define  AREA3 1608.272727f
//#define  AREA4 1621.545455f
//#define  AREA5 1617.987013f
//#define  AREA6 1608.623377f
//#define  AREA7 1621.428571f
//#define  AREA8 1617.987013f
//#define  AREA9 1609.0f
#define  AREA1 1603.0f
#define  AREA2 1601.0f
#define  AREA3 1596.0f
#define  AREA4 1620.0f
#define  AREA5 1616.0f
#define  AREA6 1611.0f
#define  AREA7 1635.0f
#define  AREA8 1634.0f
#define  AREA9 1623.0f

//==========测量平台到摄像头九个区域的中心=====

#define  CENTER1 53,38
#define  CENTER2 56,115
#define  CENTER3 56,192
#define  CENTER4 168,38
#define  CENTER5 168,115
#define  CENTER6 168,192
#define  CENTER7 279,38
#define  CENTER8 279,115
#define  CENTER9 279,192

//=========系统校准参数=========
#define MEADSURING_TABLE_STDDEV 0.1f//


	typedef enum neolix_status
	{
		NEOLIX_SUCCESS = 0,
		NEOLIX_FALSE,

	}NEOLIX_STATUS_LIST;

	typedef int32_t NEOLIX_STATUS;
#include <stdio.h>
  //  #define LOGD0(_str, ...) do{printf(_str , ## __VA_ARGS__); printf("\n");fflush(stdout);} while(0)
	#define LOGINFO(_str, ...) do{printf(_str, ## __VA_ARGS__); printf("\n");fflush(stdout);} while (0)
	#define LOGERROR(_str, ...) do{printf(_str, ## __VA_ARGS__); printf("\n");fflush(stdout);} while (0)
	#define LOGINFO_TO_FILE(_logFileStream,logLine) do \
		{\
		(_logFileStream)<<("LOG  INFO: ")<<logLine<<std::endl;\
	} while (0)

	#define LOGERROR_TO_FILE(_logFileStream,logLine) do \
		{\
		(_logFileStream)<<("LOG  ERROR: ")<<logLine<<std::endl;\
	} while (0)


	#define CHECK_OK(x) do \
	{\
		int status = x;\
		if(status != NEOLIX_SUCCESS) \
			LOGINFO("check fail, error code is: %d",status)\
	} while (0)



	static inline void parseToRGB(const TY_FRAME_DATA& frame,cv::Mat* pColor,int i)
	{
		if (frame.image[i].pixelFormat == TY_PIXEL_FORMAT_YVYU){
			cv::Mat yuv(frame.image[i].height, frame.image[i].width
				, CV_8UC2, frame.image[i].buffer);
			cv::cvtColor(yuv, *pColor, cv::COLOR_YUV2BGR_YVYU);
		}
		else if (frame.image[i].pixelFormat == TY_PIXEL_FORMAT_YUYV){
			cv::Mat yuv(frame.image[i].height, frame.image[i].width
				, CV_8UC2, frame.image[i].buffer);
			cv::cvtColor(yuv, *pColor, cv::COLOR_YUV2BGR_YUYV);
		} else if(frame.image[i].pixelFormat == TY_PIXEL_FORMAT_RGB){
			cv::Mat rgb(frame.image[i].height, frame.image[i].width
				, CV_8UC3, frame.image[i].buffer);
			cv::cvtColor(rgb, *pColor, cv::COLOR_RGB2BGR);
		} else if(frame.image[i].pixelFormat == TY_PIXEL_FORMAT_MONO){
			cv::Mat gray(frame.image[i].height, frame.image[i].width
				, CV_8U, frame.image[i].buffer);
			cv::cvtColor(gray, *pColor, cv::COLOR_GRAY2BGR);
		}
	}

	static inline int parseFrame(const TY_FRAME_DATA& frame, cv::Mat* pDepth
		, cv::Mat* pLeftIR, cv::Mat* pRightIR
		, cv::Mat* pLeftRGB,cv::Mat* pRightRGB
		,cv::Mat* pPoints)
	{
		for (int i = 0; i < frame.validCount; i++)
		{
			// get depth image
			if(pDepth && frame.image[i].componentID == TY_COMPONENT_DEPTH_CAM){
				*pDepth = cv::Mat(frame.image[i].height, frame.image[i].width
					, CV_16U, frame.image[i].buffer);
			}
			// get left ir image
			else if(pLeftIR && frame.image[i].componentID == TY_COMPONENT_IR_CAM_LEFT){
				*pLeftIR = cv::Mat(frame.image[i].height, frame.image[i].width
					, CV_8U, frame.image[i].buffer);
			}
			// get right ir image
			else if(pRightIR && frame.image[i].componentID == TY_COMPONENT_IR_CAM_RIGHT){
				*pRightIR = cv::Mat(frame.image[i].height, frame.image[i].width
					, CV_8U, frame.image[i].buffer);
			}
			// get point3D
			else if(pPoints && frame.image[i].componentID == TY_COMPONENT_POINT3D_CAM){
				*pPoints = cv::Mat(frame.image[i].height, frame.image[i].width
					, CV_32FC3, frame.image[i].buffer);
			}
			//get left RGB
			else if (pLeftRGB && (frame.image[i].componentID == TY_COMPONENT_RGB_CAM || frame.image[i].componentID == TY_COMPONENT_RGB_CAM_LEFT))
			{
				parseToRGB(frame,pLeftRGB,i);
			}
			else if (pRightRGB && frame.image[i].componentID == TY_COMPONENT_RGB_CAM_RIGHT)
			{
				parseToRGB(frame,pRightRGB,i);
			}
		}
		return 0;
	}






}
#endif

