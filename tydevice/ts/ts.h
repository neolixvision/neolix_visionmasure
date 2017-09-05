
#ifndef _TS__H_
#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>


#include "../driverdecorate/camdecorate.h"
//#include"../driverdecorate/getfeatures.hpp"
#include "../imagepro\Utils.h"
#include"../imagepro\CalDepth.h"
namespace neolix{
//测试测量摄像头到测量平台的距离
void test_cal_pad_dis();
void cvWait(bool &exit_main);
void test_recognition_object();
}

#endif // _TS__H_
