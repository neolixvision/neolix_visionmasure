#include"Utils.h"
#include <opencv2/imgproc/imgproc.hpp>
#include<opencv2/core/core.hpp>
#include <opencv2//opencv.hpp>
#include <vector>
#include "../imagepro/CalDepth.h"
#include "../driverdecorate/base.h"
#include "../driverdecorate/camdecorate.h"
namespace neolix{

void LaplasSharp(const cv::Mat &src, cv::Mat &dest)
{
	cv::Mat tmp = src.clone();
	dest.create(src.size(),src.type());
	for (int row = 1; row < src.rows-1; row++)
	{
		const uchar* previous  = tmp.ptr<const uchar>(row-1);
		const uchar* current   = tmp.ptr<const uchar>(row);
		const uchar* next      = tmp.ptr<const uchar>(row+1);

		uchar *result = dest.ptr<uchar>(row);
		for (int col = 1; col < src.cols-1; col++)
		{
			*result++ = cv::saturate_cast<uchar>(
				5*current[col]-current[col-1]
				-current[col+1]-previous[col]-next[col]
				);
		}
	}
	dest.row(0).setTo(cv::Scalar(0));
	dest.row(dest.rows - 1).setTo(cv::Scalar(0));
	dest.col(0).setTo(cv::Scalar(0));
	dest.col(dest.cols -1).setTo(cv::Scalar(0));
}


void GammaConver(const cv::Mat &src, cv::Mat &dest, double gamma)
{
	cv::Mat X;
	src.convertTo(X,CV_32FC1);
	cv::pow(X,gamma,dest);
	cv::normalize(dest,dest,255,0,cv::NORM_MINMAX,CV_8UC1);
}

void ExtractObject(const cv::Mat &depthImage, cv::Mat &BinaryImage)
{
	cv::Mat temp = cv::Mat::ones(depthImage.size(),CV_8UC1);
	temp = 255*temp;
	BinaryImage = depthImage.clone();
	//归一化[0,255]
	cv::normalize(BinaryImage,BinaryImage,255,0,cv::NORM_MINMAX,CV_16SC1);

	//转换类型
	BinaryImage.convertTo(BinaryImage,CV_8UC1);
	//均衡化
	cv::equalizeHist(BinaryImage,BinaryImage);
	//像素反转
	cv::subtract(temp,BinaryImage,BinaryImage);
	//提高对比度
	BinaryImage.convertTo(BinaryImage,-1,0.8,-25);
	//中值滤波
	cv::medianBlur(BinaryImage,BinaryImage,5);
	//锐化
	LaplasSharp(BinaryImage,BinaryImage);
	GammaConver(BinaryImage,BinaryImage,1/0.4);
	//形态学处理
	cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(5,5));
	cv::morphologyEx(BinaryImage,BinaryImage,cv::MORPH_OPEN,element);
	//二值化
	cv::threshold(BinaryImage,BinaryImage,250,255,cv::THRESH_OTSU);
}
void readDepthData(cv::Mat &dest, const size_t rows, const size_t cols,const string RawPath)
{
	//从raw二进制文件读数据
	dest.create(rows,cols,CV_16SC1);
	std::ifstream inFile(RawPath, std::ios::in|std::ios::binary);
	unsigned char upBit;
	unsigned char downBit;
	char *buf = new char[rows*2];
	for (size_t col = 0; col < cols; col++)
	{
		//auto rowPtr = dest.ptr<float>(col);
		//cout<<sizeof(float)<<endl;
		inFile.read(buf,rows*2);

		for (size_t row = 0; row < rows*2; row += 2)
		{
			unsigned short dept = 0;
			downBit = buf[row];
			upBit = buf[row+1];
			dept = (int)downBit;
			dept += (int)upBit*256;
			dest.at<short>(row/2,col) = dept;
			//cout<<dest.at<float>(row/2,col)<<"== "<<dept<<"  ";
		}
		cout<<endl;
	}
	inFile.close();
	delete[] buf;

}

void onMouse(int event, int x ,int y, int flags, void* img)
{
	cv::Mat *image = (cv::Mat*)img;
	if(event == CV_EVENT_LBUTTONDOWN)
	{
		std::cout<<"x = "<<x
			<<" y = "<<y
			<<" depth = "<<image->at<short>(y,x)<<endl<<endl;
	}

}

unsigned short calculateDepthFromDepthImagInRangeCountour(cv::Mat &depthIamge, cv::vector<cv::Point> &contour,double &confidence)
{
	cv::Mat shortIamge;
	std::vector<short> depthPoints;
	if (depthIamge.depth() != CV_16SC1)
	{
		depthIamge.convertTo(shortIamge,CV_16SC1);
	}else
	{
		shortIamge = depthIamge.clone();
	}

	for (int row = 0; row < shortIamge.rows; row++)
	{
		auto rowPtr = shortIamge.ptr<short>(row);
		for (int col =0; col < shortIamge.cols; col++)
		{
			if(cv::pointPolygonTest(contour, cv::Point(col,row),false) == 1) depthPoints.push_back(rowPtr[col]);
		}
	}

	caldepth depthHist(&depthPoints);
	depthHist.calDepthHist();

	return depthHist.getdepth(confidence);

}

unsigned short calculateDepthFromDepthImagOutRangeCountour(cv::Mat &depthIamge, cv::vector<cv::Point> &contour,double &confidence)
{
	cv::Mat shortIamge;
	std::vector<short> depthPoints;
	if (depthIamge.type() != CV_16SC1)
	{
		depthIamge.convertTo(shortIamge,CV_16SC1);
	}else
	{
		shortIamge = depthIamge.clone();
	}

	for (int row = 0; row < shortIamge.rows; row++)
	{
		auto rowPtr = shortIamge.ptr<short>(row);
		for (int col =0; col < shortIamge.cols; col++)
		{
			if(cv::pointPolygonTest(contour, cv::Point(col,row),false) != 1) depthPoints.push_back(rowPtr[col]);
		}
	}

	caldepth depthHist(&depthPoints);
	depthHist.calDepthHist();
	return depthHist.getdepth(confidence);

}

bool adjustSystem( cv::Mat depthIamgeRoi,  cv::Mat mask, unsigned short &distance )
{
	cv::vector<short> vaildDepth;
	cv::Mat shortImage;
	if (depthIamgeRoi.type() != CV_16SC1)
	{
		depthIamgeRoi.convertTo(shortImage, CV_16SC1);
	}else
	{
		shortImage = depthIamgeRoi.clone();
	}

    short* ptr = shortImage.ptr < short>();

	for (size_t index = 0; index < shortImage.size().area(); index++)
	{
		if (0 != ptr[index])
		{
			vaildDepth.push_back(ptr[index]);
		}
	}
	cv::Mat vailDepthMatix(vaildDepth);
	cv::Mat mean, stddev;
	cv::meanStdDev(vailDepthMatix,mean,stddev);
	//如果测量平台到摄像机的距离点的标准差大于一定数值，认为是测量平台与摄像头不是垂直，或者是测量平台存在物体，因此校验失败，返回。调整测量系统后再校验
	if(MEADSURING_TABLE_STDDEV < stddev.at<double>(0,0)  )  return false;

    if(mask.rows == shortImage.rows && mask.cols == shortImage.cols && mask.channels() == shortImage.channels())
    {
        double confidence;
        std::vector<short> histData;
        short* depth_ptr = shortImage.ptr<short>();
        uchar* mask_ptr  = mask.ptr<uchar>();
        for(size_t index; index < shortImage.size().area(); index++) if(mask_ptr[index] != 0) histData.push_back(depth_ptr[index]);
        caldepth depthHist(&histData);
        depthHist.calDepthHist();
        distance = depthHist.getdepth(confidence);
    }else return false;
	return true;
}



void padDepthMask(const cv::Mat colorDepthImage, cv::Mat &mask)
{
    cv::Mat hsv_img;
    cv::cvtColor(colorDepthImage, hsv_img, CV_RGB2HSV);
    int h_min=52, s_min=11, v_min=28;
	int h_max=109, s_max=255, v_max=255;
	cv::Scalar hsv_min(h_min, s_min, v_min);
	cv::Scalar hsv_max(h_max, s_max, v_max);
	mask = Mat::zeros(colorDepthImage.rows, colorDepthImage.cols, CV_8UC3);
	inRange(hsv_img, hsv_min, hsv_max, mask);
}

void recordVideo(const string videopath)
{
    string savePath = videopath + "_opencv.avi";
    Capturer cap;
    cv::Mat RGBImage;
	deviceDataBase *frame = cap.getFrame();
	int32_t componetIDs = TY_COMPONENT_RGB_CAM_LEFT;
	cap.open(componetIDs);
	cap>>(frame);
	RGBImage = frame->leftRGB;
	cv::Size videoSzie = RGBImage.size();
	double rate = 20.0;
	cv::VideoWriter vw(savePath,CV_FOURCC('D','I', 'V', 'X'), rate, videoSzie);
	while(true)
    {
        cap>>(frame);
        RGBImage = frame->leftRGB;
        vw<<RGBImage;
        cv::imshow("video",RGBImage);
        if(cv::waitKey(2) == 27) break;

    }
    vw.release();
    vw.~VideoWriter();

}

}
