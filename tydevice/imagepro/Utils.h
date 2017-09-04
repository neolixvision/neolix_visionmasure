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
//У׼���������ƽ̨�ľ���
bool adjustSystem( cv::Mat depthIamgeRoi,  cv::Mat mask, unsigned short &distance);
//¼��RGB��Ƶ,�ɼ�ѵ������
void recordVideo(const string videopath);
void padDepthMask(const cv::Mat colorDepthImage, cv::Mat &mask);
//���������������ƽ̨�ľ��룬������ƽ̨�ֳ�9������
/*
���룺PadDepthImage������ƽ̨�����ͼ
���: 1��distance,����ͷ������ƽ̨�ľ��룬
      2��centerPoints,����ƽ̨�ϸ��������������
*/
void distancesFromCamToPad(std::vector<short> &distances,std::vector<index_value<int, cv::Point2i>> &centerPoints, cv::Mat PadDepthImage);
//������ѵ�����ͷ������ƽ̨ƽ��ľ���
/*
�����distancesFromCamToPad������õ����ݣ���һ����
�������ѵľ���
*/
float calDisCam2Pad(std::vector<float> &distances, std::vector<index_value<int,cv::Point2i>> &centerPoints, cv::Point2f point);

//�������������
void calCoutousCenter(cv::vector<cv::Point> &contour, cv::Point2f &center);
}
#endif
