#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>


#include "driverdecorate/camdecorate.h"
#include"driverdecorate/getfeatures.hpp"
#include "imagepro\Utils.h"
#include"imagepro\CalDepth.h"



using namespace neolix;

#define DEBUG
#ifdef WIN32
int changeDosColor()
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
	std::cout<<"��Ҫ��ʾ��\n\n\n\t\t�������ϵͳ��ʼ����ʱ����Ҫ��ϵͳ����У�飬����\n\t\tϵͳ������ʾ��װ�����������֤����ƽ̨û���κ���\n\t\t��.\n";
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
	changeDosColor();
#endif

	help();
	//==============getgeatures================
	//getfeatures();
	//=========================================
	#ifdef DEBUG

	//===========����xml�ļ��Լ���ʼ��========

	std::vector<cv::Rect> objects;
	cv::CascadeClassifier cascade;
	if( !cascade.load(""))
    {
        std::cout<<"can not  load xml file"<<std::endl;
        exit(-1);
    }




	//===========�������豸=========
	Capturer capture;
	neolix::deviceDataBase *frame = capture.getFrame();
	capture.open();

    bool exit_main = false;
    cv::Mat RGBImage;
    cv::Mat gray;
    cv::namedWindow("RGBVideo");
    while(! exit_main)
    {
        capture>>frame;
        RGBImage = frame->leftRGB;
      //  cv::imshow("RGBVideo",RGBImage);
        cv::cvtColor(RGBImage, gray,CV_RGB2GRAY);
        cascade.detectMultiScale(gray,objects,1.1,4,0|cv::CASCADE_SCALE_IMAGE, cv::Size(100, 100));
        std::cout<<"��鵽 "<<objects.size()<<"��Ŀ��"<<std::endl;
        for(size_t i = 0; i < objects.size(); i++)
        {
            cv::rectangle(RGBImage, objects[i], cv::Scalar(255,0,255));
        }
        cv::imshow("RGBVideo",RGBImage);
        cvWait(exit_main);

    }






	#else
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
			//======================���������Ϣ================================

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

			//===================��ȡ������->����߶�=========================

			depthRoi = depth(rects);
			unsigned short BoxDistance = calculateDepthFromDepthImagInRangeCountour(depthRoi,contours,confdence);
			unsigned short PadDistance = calculateDepthFromDepthImagOutRangeCountour(depthRoi,contours,confdence);

		//========================================================================

		//��ȡ��ʵ�ĳ���q

		//neolix::Getxy(PixLength,PixWidth,distance,Length,Width);
		Getxyz2(point,BoxDistance,Length,Width);
		std::cout<<Length<<"mm*"<<Width<<"mm*"<<(1610-BoxDistance)<<"mm"<<std::endl;

		cv::setMouseCallback("depth",onMouse,&colorDepth);

		cvWait(exit_main);
	}
    #endif // DEBUG
	return 0;
}
