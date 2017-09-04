#ifndef  _NEOLIX_UTILS__H__
#define  _NEOLIX_UTILS__H__

#include<opencv2/core/core.hpp>
#include<fstream>
#include<iostream>
#include<vector>

#include "../driverdecorate/base.h"

using namespace std;
using namespace  cv;

namespace neolix{

template <typename T1, typename T2>
struct index_value
{
    T1 index;
    T2 value;
    index_value(T1 idx, T2 val)
    {
        index = idx;
        value = val;
    }
    index_value(){}

};
void LaplasSharp(const cv::Mat &src, cv::Mat &dest);
void GammaConver(const cv::Mat &src, cv::Mat &dest, double gamma);
void ExtractObject(const cv::Mat &depthImage, cv::Mat &BinaryImage);
void readDepthData(cv::Mat &dest, const size_t rows, const size_t cols,const string RawPath);
void onMouse(int event, int x ,int y, int flags, void* img);
NEOLIX_STATUS_LIST  PopRang(cv::Mat &srcImg,cv::vector<cv::Point>& contour,float &PixLength,float &PixWidth,cv::vector<cv::Point2f> &point);
void seg(cv::Mat &img,cv::Mat &img1,cv::Mat &img2,const int &rows,const int &cols);
void Getxy(const float PixLength,const float PixWidth,short distance,float &Length,float &Width);
void Getxyz2(cv::vector<cv::Point2f> point,short distance,float &Length,float &Width);
unsigned short calculateDepthFromDepthImagInRangeCountour(cv::Mat &depthIamge, cv::vector<cv::Point> &contour,double &confidence);
unsigned short calculateDepthFromDepthImagOutRangeCountour(cv::Mat &depthIamge, cv::vector<cv::Point> &contour,double &confidence);
void getBigerRect(cv::Rect rect, cv::Point point);
//校准相机到测量平台的距离
bool adjustSystem( cv::Mat depthIamgeRoi,  cv::Mat mask, unsigned short &distance);
//录制RGB视频,采集训练数据
void recordVideo(const string videopath);
void padDepthMask(const cv::Mat colorDepthImage, cv::Mat &mask);
//测量摄像机到测量平台的距离，将测量平台分成9个区域
/*
输入：PadDepthImage，测量平台的深度图
输出: 1、distance,摄像头带测量平台的距离，
      2、centerPoints,测量平台上各个区域的中心上
*/
void distancesFromCamToPad(std::vector<short> &distances,std::vector<index_value<int, cv::Point2i>> &centerPoints, cv::Mat PadDepthImage);
//计算最佳的摄像头到测量平台平面的距离
/*
输出：distancesFromCamToPad函数获得的数据，和一个点
输出：最佳的距离
*/
float calDisCam2Pad(std::vector<float> &distances, std::vector<index_value<int,cv::Point2i>> &centerPoints, cv::Point2f point);

//获得轮廓的中心
void calCoutousCenter(cv::vector<cv::Point> &contour, cv::Point2f &center);
}
#endif
