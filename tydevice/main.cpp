#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>


#include "driverdecorate/camdecorate.h"
//#include"driverdecorate/getfeatures.hpp"
#include "imagepro\Utils.h"
#include"imagepro\CalDepth.h"
#include"ts/ts.h"



using namespace neolix;

//#define DEBUG
//#define TEST_CAL_PAD_DIS
#define RUNMAIN
#ifdef WIN32
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

int main(int argc, char** argv)
{
//lelele123
#ifdef WIN32
	changeDosColor();
#endif

	help();
	//==============getgeatures================
	//getfeatures();
	//=========================================
	#ifdef TEST_CAL_PAD_DIS

	#endif // TEST_CAL_PAD_DIS
    test_cal_pad_dis();
	#ifdef DEBUG

	//===========加载xml文件以及初始化========

	std::vector<cv::Rect> objects;
	cv::CascadeClassifier cascade;
	if( !cascade.load("cascade.xml"))
    {
        std::cout<<"can not  load xml file"<<std::endl;
        exit(-1);
    }

	//===========打开摄像设备=========
	Capturer capture;
	neolix::deviceDataBase *frame = capture.getFrame();
	capture.open(TY_COMPONENT_DEPTH_CAM | TY_COMPONENT_RGB_CAM);

    bool exit_main = false;
    cv::Mat RGBImage;
    cv::Mat gray;
    cv::namedWindow("RGBVideo");
    while(! exit_main)
    {
        capture>>frame;
        RGBImage = frame->leftRGB;
        cv::imshow("RGBVideo",RGBImage);
		cvWait(exit_main);
        cv::cvtColor(RGBImage, gray,CV_RGB2GRAY);
        cascade.detectMultiScale(gray,objects,1.1,4,0|cv::CASCADE_SCALE_IMAGE, cv::Size(100, 100));
        std::cout<<"检查到 "<<objects.size()<<"个目标"<<std::endl;
        for(size_t i = 0; i < objects.size(); i++)
        {
            cv::rectangle(RGBImage, objects[i], cv::Scalar(255,0,255));
        }
        cv::imshow("RGBVideo",RGBImage);
        cvWait(exit_main);

    }
	#endif // DEBUG

	#ifdef RUNMAIN
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
	float paddis;
	double confdence;
	float PixLength,PixWidth,Length,Width;
    std::vector<float> distances;
    std::vector<index_value<int, cv::Point2i>> centers;
    cv::Point2f objCenter;
    distances.push_back(AREA1);
    distances.push_back(AREA2);
    distances.push_back(AREA3);
    distances.push_back(AREA4);
    distances.push_back(AREA5);
    distances.push_back(AREA6);
    distances.push_back(AREA7);
    distances.push_back(AREA8);
    distances.push_back(AREA9);
    centers.push_back(index_value<int,cv::Point2i> (1,cv::Point2i(CENTER1)));
    centers.push_back(index_value<int,cv::Point2i> (2,cv::Point2i(CENTER2)));
    centers.push_back(index_value<int,cv::Point2i> (3,cv::Point2i(CENTER3)));
    centers.push_back(index_value<int,cv::Point2i> (4,cv::Point2i(CENTER4)));
    centers.push_back(index_value<int,cv::Point2i> (5,cv::Point2i(CENTER5)));
    centers.push_back(index_value<int,cv::Point2i> (6,cv::Point2i(CENTER6)));
    centers.push_back(index_value<int,cv::Point2i> (7,cv::Point2i(CENTER7)));
    centers.push_back(index_value<int,cv::Point2i> (8,cv::Point2i(CENTER8)));
    centers.push_back(index_value<int,cv::Point2i> (9,cv::Point2i(CENTER9)));

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
				cvWait(exit_main);
				continue;
			}
			//==================================================================

			//===================获取了轮廓->计算高度=========================

			depthRoi = depth(rects);
			calCoutousCenter(contours,objCenter);
			unsigned short BoxDistance = calculateDepthFromDepthImagInRangeCountour(depthRoi,contours,confdence);
			paddis = calDisCam2Pad(distances,centers,objCenter);
			//unsigned short PadDistance = calculateDepthFromDepthImagOutRangeCountour(depthRoi,contours,confdence);//目前弃用，这个函数猪要是在ROi中寻找蓝色区域统计距离

		//========================================================================

		//获取真实的长宽q

		//neolix::Getxy(PixLength,PixWidth,distance,Length,Width);
		Getxyz2(point,BoxDistance,Length,Width);
		std::cout<<Length<<"mm*"<<Width<<"mm*"<<(paddis-BoxDistance)<<"mm"<<std::endl;

		cv::setMouseCallback("depth",onMouse,&colorDepth);

		cvWait(exit_main);

	}
    #endif // RUNMAIN
	return 0;
}
