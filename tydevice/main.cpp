#include <iostream>
#include "driverdecorate/camdecorate.h"
#include <opencv2/opencv.hpp>
#include"driverdecorate/getfeatures.hpp"
#include "imagepro\Utils.h"
#include"imagepro\CalDepth.h"

using namespace neolix;
int main()
{
//lelelelele
	//==============getgeatures================
	//getfeatures();
	//=========================================
	Capturer a;
	deviceDataBase *frame = a.getFrame();

	int32_t componetIDs = TY_COMPONENT_DEPTH_CAM;
	a.open(componetIDs);

	a.setlaserPower(99);
	bool exit_main = false;

	cv::Mat depth,colorRoi,colorDepth,depthRoi;
	cv::Rect rects = cv::Rect(RECT_LEFT_UP_X_POINT,RECT_LEFT_UP_Y_POINT,RECT_BOX_WIDTH,RECT_BOX_HEIGHT);
	std::vector<short> data;
	cv::vector<cv::Point> contours;
	cv::vector<cv::Point2f> point;
	double confdence;
	float PixLength,PixWidth,Length,Width;
	while (!exit_main)
	{
		a>>(frame);

		cv::Mat colorDepth =frame->render->Compute(frame->depth);
		imshow("text",colorDepth);

			//cv::Mat left_rgb = frame->leftRGB;
			depth = frame->depth;
			//======================获得轮廓信息================================
			
			colorRoi = depth(rects).clone();
			colorDepth = frame->render->Compute(colorRoi);
			point.clear();
			NEOLIX_STATUS_LIST status =PopRang(colorDepth , contours , PixLength , PixWidth , point);
			//====================================//
			vector<vector<Point>> vvp;
			vvp.push_back(contours);
			drawContours(colorDepth,vvp,0,Scalar(0,0,0),4);
			imshow("test2",colorDepth);
			//====================================//
			if (NEOLIX_FALSE ==status)
			{
				std::cout << "can't find contours" <<std::endl;
				continue;
			}
			//==================================================================

			//===================获取了轮廓->计算高度=========================
				
			depthRoi = depth(rects);
			unsigned short BoxDistance = calculateDepthFromDepthImagInRangeCountour(depthRoi,contours,confdence);
			unsigned short PadDistance = calculateDepthFromDepthImagOutRangeCountour(depthRoi,contours,confdence);

		//========================================================================
		
		//获取真实的长宽q
		
		//neolix::Getxy(PixLength,PixWidth,distance,Length,Width);
		Getxyz2(point,BoxDistance,Length,Width);
		std::cout<<Length<<"mm*"<<Width<<"mm*"<<(1610-BoxDistance)<<"mm"<<std::endl;

		cv::setMouseCallback("depth",onMouse,&colorDepth);
<<<<<<< HEAD
		int key = cv::waitKey(30);
		switch (key & 0xff)
		{
		case 'q':
			exit_main = true;
			break;
		default:
			break;
		}
	}	
=======
		cvWait(exit_main);
	}
>>>>>>> 355bdc58ecb3840d6bc77789b7987ed7b47abd91
	return 0;
}
