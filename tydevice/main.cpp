#include <iostream>
#include "driverdecorate/camdecorate.h"
#include <opencv2/opencv.hpp>
#include"driverdecorate/getfeatures.hpp"
#include "imagepro\Utils.h"
#include"imagepro\CalDepth.h"

using namespace neolix;
#ifdef WIN32
#include<stdio.h>
#include<windows.h>
void changeDosColor()
{
	HANDLE handle = GetStdHandle(STD_OUTPUT_HANDLE);
	COORD pos;
	pos.X = 35;
	pos.Y = 1;
	SetConsoleCursorPosition(handle,pos);
	SetConsoleTextAttribute(handle,FOREGROUND_INTENSITY
		| FOREGROUND_GREEN);
}
#endif
void help()
{
	std::cout<<"重要提示：\n\n\n\t\t体积测量系统初始测量时，需要对系统进行校验，根据\n\t\t系统启动提示安装步骤操作并保证测量平台没有任何物\n\t\t体.\n";
}

void cvWait(bool &exit_main)
{
    int key = cv::waitKey(10);
		switch (key & 0xff)
		{
		case 'q':
			exit_main = true;
			break;
		default:
			break;
		}
}
int main(int argc, char** argv)
{
//lelele123
#ifdef WIN32
	//changeDosColor();
#endif

	help();
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

			depth = frame->depth;
	//		//======================获得轮廓信息================================
	//
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
				cvWait(exit_main);
				continue;
			}
	//		//==================================================================

	//		//===================获取了轮廓->计算高度=========================
	//
			depthRoi = depth(rects);
			unsigned short BoxDistance = calculateDepthFromDepthImagInRangeCountour(depthRoi,contours,confdence);
			unsigned short PadDistance = calculateDepthFromDepthImagOutRangeCountour(depthRoi,contours,confdence);

	//	//========================================================================
	//
	//	//获取真实的长宽q
	//
	//	//neolix::Getxy(PixLength,PixWidth,distance,Length,Width);
		Getxyz2(point,BoxDistance,Length,Width);
		std::cout<<Length<<"cm*"<<Width<<"cm*"<<BoxDistance/10.0<<"cm"<<std::endl;
		cv::setMouseCallback("depth",onMouse,&colorDepth);
		cvWait(exit_main);
	}
	return 0;
}
