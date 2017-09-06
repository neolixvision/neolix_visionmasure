//������ȡ
#include <opencv2\opencv.hpp>
#include<cmath>
using namespace cv;
using namespace std;
#include "../driverdecorate/base.h"
namespace neolix{

void SizeGet(cv::Mat &srcImg,cv::Mat &dstImg)
{
	cv::Mat hsv_img;
	cvtColor(srcImg, hsv_img, CV_BGR2HSV);  //ת����HSV��ɫ�ռ�
	int h_min=0, s_min=43, v_min=46;
	int h_max=20, s_max=255, v_max=255;
	cv::Scalar hsv_min(h_min, s_min, v_min);
	cv::Scalar hsv_max(h_max, s_max, v_max);
	dstImg = Mat::zeros(srcImg.rows, srcImg.cols, CV_8UC3);
	inRange(hsv_img, hsv_min, hsv_max, dstImg);
}
void undistort(cv::Mat &srcImg ,cv::Mat &srcimg)
	{
			//��ȡ��ɫͼ��Ҫ�궨����,��ȡ�����������
	cv::FileStorage fs2("intrinsics.yml", cv::FileStorage::READ);
	cv::Mat D1 = cv::Mat(3, 4, CV_32FC1);
	cv::Mat M2=cv::Mat::zeros(1,4,CV_32FC1);
	cv::Mat M1 = cv::Mat(3, 3, CV_32FC1);
	fs2["M1"] >> M1;
	fs2["D1"] >> D1;

	///=================���۾�ͷ�궨����=======================
	M2.at<double>(0,0) =  D1.at<double>(0,0);
	M2.at<double>(0,1) =  D1.at<double>(0,1);
	M2.at<double>(0,2) =  D1.at<double>(1,0);
	M2.at<double>(0,3) =  D1.at<double>(1,1);
	///=================��ͨ��ͷ�궨����=======================
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
* function: ��α��ɫ����ȡĿ������(�ع����ִ��룩
*/
inline void segmentationImage(const cv::Mat srcImg, cv::Mat &destImage)
{
    //=====�������ͼת�ɵط�α��ɫͼ�ָ������=========
    cv::Mat srcimg = srcImg.clone();
    SizeGet(srcimg, destImage);

    //====��̬ѧ����ָ����ͼ�����򻯣��ڸ�ʴ����ȫĿ�������һЩȱ��====
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3));
    cv::morphologyEx(destImage, destImage, cv::MORPH_CLOSE, element);

}
/**
* author: Pengcheng, pengcheng@neolix.cn
* function: ��ȡĿ��������������(�ع����룩
**/

inline NEOLIX_STATUS_LIST  getMaxContours(cv::Mat binaryImage, int &maxContourId)
{
    cv::vector<cv::vector<cv::Point>> contours;
     cv::vector<cv::Vec4i> hierarchy;
    cv::findContours(binaryImage,hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE,cv::Point(0, 0));

    if(0 == contours.size()) return NEOLIX_FALSE;
    ///�����������
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
* funtion: Ѱ����������С��Ӿ���(�ع�����)
*/
inline void findMinRectangle(cv::vector<cv::Point>(* maxcontour), cv::RotatedRect &rectangle)
{
    ///Ѱ����С��Ӿ���
    rectangle = cv::minAreaRect(*maxcontour);
    ///������Ӿ���Ĵ�С��Ŀ����Ϊ��������峤��ľ���
    adjustRectSize(rectangle , rectangle);

}
/**
* author: PengCheng, pengcheng@neolix.cn
* funtion:����������Ӿ��εĳ���
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




	//=================��ȡ���ͼ��������ͼ=======================
	cv::Mat dstimg;
	cv::Mat srcimg = srcImg.clone();
	SizeGet(srcimg , dstimg);
	//=================��ȡ���ͼ��������ͼ=======================



	//===================��̬ѧ���㣬�ֲ�����ȱ��=================
	cv::Mat dst2;
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
	morphologyEx(dstimg,dst2, cv::MORPH_CLOSE, element);
	//===================��̬ѧ���㣬�ֲ�����ȱ��=================



	//===================����������Ѱ���������===================
	cv::vector<cv::vector<cv::Point> > contours;
    cv:: vector<cv::Vec4i> hierarchy;
	findContours( dst2, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE, cv::Point(0, 0) );

	//�Ƿ���ҵ������ж�
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
	//===================����������Ѱ���������===================

	//===================������С�����������=====================
	cv::RotatedRect rect=minAreaRect(contours[Max_sizeId]);
       cv::Point2f P[4];  //rect���ĸ��㴫��P
       rect.points(P);
	   //���ÿ��ֱ�ߵĳ���,a[0]�ǿ�a[1]�Ǹ�
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

	//===================������С�����������=====================


	//==================�ض�λ���ž�������===================

		cv::RotatedRect rec;
		adjustRectSize(rect , rec);

		//rect���ĸ��㴫��P
		cv::Point2f P2[4];
		rec.points(P2);

		//�洢�ؼ��ǵ�λ����Ϣ��Ϊgetxyz2����Ԥ���ӿ�
		cv::Point2f p;
		for(size_t i=0;i<=2;i++)
		{
		p.x = P2[i].x ;
		p.y = P2[i].y ;
		point.push_back(p);
		}




	//=======================����������Ϣ===========================
	/*
	cv::Point i;
	cv::vector<cv::Point>::iterator iter=contours[g].begin();
	while(iter!=contours[g].end())
		{
			std::cout <<*iter<<std::endl;
			iter++;
	}*/
	//=======================����������Ϣ===========================

	//std::cout << "�����������������:"<<a[0]*a[1]<<std::endl;
	//std::cout << "��С��Χ�����"<<cv::contourArea(contours[Max_sizeId]) <<std::endl;

	float judge = CompareArea(a[0]*a[1],cv::contourArea(contours[Max_sizeId]));

	if(judge > 0.45)
	{
		std::cout << "---------------------" <<std::endl;
		std::cout << "    ���ڵ���ÿ�   " <<std::endl;
		std::cout << "---------------------" <<std::endl;
	return NEOLIX_FALSE;
	}



	contour = contours[Max_sizeId];
	return NEOLIX_SUCCESS;
}


}
