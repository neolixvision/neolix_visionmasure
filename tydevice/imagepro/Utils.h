#ifndef  _UTILS__H__
#define  _UTILS__H__

#include<opencv2/core/core.hpp>
#include<fstream>
#include<iostream>
#include "../driverdecorate/base.h"

using namespace std;
using namespace  cv;

namespace neolix{

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
}
#endif
