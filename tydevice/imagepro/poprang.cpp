//轮廓提取
#include <opencv2\opencv.hpp>
#include<cmath>
using namespace cv;
using namespace std;
#include "../driverdecorate/base.h"
namespace neolix{

void SizeGet(cv::Mat &srcImg,cv::Mat &dstImg)
{
	cv::Mat hsv_img;
	cvtColor(srcImg, hsv_img, CV_BGR2HSV);  //转换到HSV颜色空间
	int h_min=0, s_min=43, v_min=46;
	int h_max=20, s_max=255, v_max=255;
	cv::Scalar hsv_min(h_min, s_min, v_min);
	cv::Scalar hsv_max(h_max, s_max, v_max);
	dstImg = Mat::zeros(srcImg.rows, srcImg.cols, CV_8UC3);
	inRange(hsv_img, hsv_min, hsv_max, dstImg);
}
void undistort(cv::Mat &srcImg ,cv::Mat &srcimg)
	{
			//读取彩色图需要标定程序,读取参数畸变矫正
	cv::FileStorage fs2("intrinsics.yml", cv::FileStorage::READ);
	cv::Mat D1 = cv::Mat(3, 4, CV_32FC1);
	cv::Mat M2=cv::Mat::zeros(1,4,CV_32FC1);
	cv::Mat M1 = cv::Mat(3, 3, CV_32FC1);
	fs2["M1"] >> M1;
	fs2["D1"] >> D1;

	///=================鱼眼镜头标定参数=======================
	M2.at<double>(0,0) =  D1.at<double>(0,0);
	M2.at<double>(0,1) =  D1.at<double>(0,1);
	M2.at<double>(0,2) =  D1.at<double>(1,0);
	M2.at<double>(0,3) =  D1.at<double>(1,1);
	///=================普通镜头标定参数=======================
	/**M2.at<float>(0,0) =  D1.at<double>(0,0);
	M2.at<float>(0,1) =  D1.at<double>(0,1);
	M2.at<float>(0,2) =  D1.at<double>(0,2);
	M2.at<float>(0,3) =  D1.at<double>(0,3);*/
	cv::undistort(srcImg,srcimg,M1,M2);

	}


float CompareArea(const float Circumscribed_rectangleArea , const float ContoursArea)
{
	float Area_difference = Circumscribed_rectangleArea - ContoursArea ;
	return Area_difference/ContoursArea ;
}

cv::RotatedRect adjustRectSize(cv::RotatedRect &rect ,cv::RotatedRect &rec )
{
	cv::Point2f Center;
	Center.x = rect.center.x;
	Center.y = rect.center.y;
	float height = rect.size.height;
	float width = rect.size.width;
	Size2f size(height-2 ,width-2);
	float angle = rect.angle;

	cv::RotatedRect recv(Center , size , angle);

	return rec = recv;
}


/**
* author: PengCheng, pengcheng@neolix.cn
* function: 从伪彩色中提取目标物体(重构尹乐代码）
*/
inline void segmentationImage(const cv::Mat srcImg, cv::Mat &destImage)
{
    //=====利用深度图转成地方伪彩色图分割出物体=========
    cv::Mat srcimg = srcImg.clone();
    SizeGet(srcimg, destImage);

    //====形态学处理分割出看图像，先膨化，在腐蚀，补全目标区域的一些缺陷====
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3));
    cv::morphologyEx(destImage, destImage, cv::MORPH_CLOSE, element);

}
/**
* author: Pengcheng, pengcheng@neolix.cn
* function: 获取目标区域的最大轮廓(重构代码）
**/

inline NEOLIX_STATUS_LIST  getMaxContours(cv::Mat binaryImage, int &maxContourId)
{
    cv::vector<cv::vector<cv::Point>> contours;
     cv::vector<cv::Vec4i> hierarchy;
    cv::findContours(binaryImage,hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE,cv::Point(0, 0));

    if(0 == contours.size()) return NEOLIX_FALSE;
    ///查找最大轮廓
    double maxContourArea = 0.0;
    int    maxAreaIndex   = 0;
    for(size_t i =0; i < contours.size(); i++)
    {
        if(maxContourArea < cv::contourArea(contours[i]))
        {
            maxAreaIndex = i;
            maxContourArea = cv::contourArea(contours[i]);
        }
    }
    maxContourId = maxAreaIndex;
    return NEOLIX_SUCCESS;


}
/**
* author: PengCheng ,pengcheng@neolix.cn
* funtion: 寻找轮廓的最小外接矩形(重构代码)
*/
inline void findMinRectangle(cv::vector<cv::Point>(* maxcontour), cv::RotatedRect &rectangle)
{
    ///寻找最小外接矩形
    rectangle = cv::minAreaRect(*maxcontour);
    ///调整外接矩阵的大小，目的是为了提高箱体长宽的精度
    adjustRectSize(rectangle , rectangle);

}
/**
* author: PengCheng, pengcheng@neolix.cn
* funtion:计算轮廓外接矩形的长宽
*/
inline void calPixelDistance(cv::RotatedRect rect, float &xLegnth, float &yLength)
{
    cv::Point2f vertex[4];
    rect.points(vertex);
    float diffx  = 0.0f;
    float diffy  = 0.0f;
    float length_t = 0.0f;
    float temp[2] = {0};
    for(unsigned int i = 0; i < 2; i++)
    {
        diffx = vertex[i].x - vertex[i+1].x;
        diffy = vertex[i].y - vertex[i+1].y;
        length_t = std::pow(diffx,2)+std::pow(diffy,2);
        temp[i] = std::sqrt(length_t);
    }
    xLegnth = temp[0];
    yLength = temp[1];
}


NEOLIX_STATUS_LIST  PopRang(cv::Mat &srcImg,cv::vector<cv::Point>& contour,float &PixLength,float &PixWidth,cv::vector<cv::Point2f> &point)
{




	//=================获取深度图轮廓分离图=======================
	cv::Mat dstimg;
	cv::Mat srcimg = srcImg.clone();
	SizeGet(srcimg , dstimg);
	//=================获取深度图轮廓分离图=======================



	//===================形态学运算，弥补轮廓缺陷=================
	cv::Mat dst2;
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
	morphologyEx(dstimg,dst2, cv::MORPH_CLOSE, element);
	//===================形态学运算，弥补轮廓缺陷=================



	//===================查找轮廓并寻找最大轮廓===================
	cv::vector<cv::vector<cv::Point> > contours;
    cv:: vector<cv::Vec4i> hierarchy;
	findContours( dst2, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE, cv::Point(0, 0) );

	//是否查找到轮廓判断
	if (0 == contours.size())return NEOLIX_FALSE;

	float a[4]={0};
	double  Max_sizeArea=0;
	int  Max_sizeId=0;
    for( size_t i = 0; i < contours.size(); i++ )
     {

		 if(Max_sizeArea<contourArea(contours[i]))
		 {
			 Max_sizeId=i;
			 Max_sizeArea=contourArea(contours[i]);
		 }
     }
	drawContours(dst2,contours,Max_sizeId,Scalar(255,255,255),5);
	//===================查找轮廓并寻找最大轮廓===================

	//===================查找最小外包矩形轮廓=====================
	cv::RotatedRect rect=minAreaRect(contours[Max_sizeId]);
       cv::Point2f P[4];  //rect的四个点传给P
       rect.points(P);
	   //求解每条直线的长度,a[0]是宽，a[1]是高
        for(int j=0;j<=1;j++)
        {
           // line(dst2,P[j],P[(j+1)%4],cv::Scalar(255,255,255),2);
				  	float x1 = P[j].x-P[j+1].x;
						float y1 =P[j].y-P[j+1].y;
						float temp = x1*x1 + y1*y1;
					//if(j<2)
				  a[j] = sqrt(temp);
        }
		PixWidth=a[0];
		PixLength=a[1];

	//===================查找最小外包矩形轮廓=====================


	//==================重定位缩放矩形区域===================

		cv::RotatedRect rec;
		adjustRectSize(rect , rec);

		//rect的四个点传给P
		cv::Point2f P2[4];
		rec.points(P2);

		//存储关键角点位置信息，为getxyz2参数预留接口
		cv::Point2f p;
		for(size_t i=0;i<=2;i++)
		{
		p.x = P2[i].x ;
		p.y = P2[i].y ;
		point.push_back(p);
		}




	//=======================调试轮廓信息===========================
	/*
	cv::Point i;
	cv::vector<cv::Point>::iterator iter=contours[g].begin();
	while(iter!=contours[g].end())
		{
			std::cout <<*iter<<std::endl;
			iter++;
	}*/
	//=======================调试轮廓信息===========================

	//std::cout << "矩形修正的面积等于:"<<a[0]*a[1]<<std::endl;
	//std::cout << "最小包围面积："<<cv::contourArea(contours[Max_sizeId]) <<std::endl;

	float judge = CompareArea(a[0]*a[1],cv::contourArea(contours[Max_sizeId]));

	if(judge > 0.45)
	{
		std::cout << "---------------------" <<std::endl;
		std::cout << "    有遮挡物，拿开   " <<std::endl;
		std::cout << "---------------------" <<std::endl;
	return NEOLIX_FALSE;
	}



	contour = contours[Max_sizeId];
	return NEOLIX_SUCCESS;
}


}
