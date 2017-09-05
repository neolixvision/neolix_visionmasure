#include "ts.h"

namespace neolix
{

    void cvWait(bool &exit_main)
    {
        int key = cv::waitKey(30);
            switch (key & 0xff)
            {
            case 'q':
                exit_main = true;
                break;
            default:
                break;
            }
    }
    void test_cal_pad_dis()
    {
        Capturer cap;
        deviceDataBase *frame = cap.getFrame();
        int32_t componetIDs = TY_COMPONENT_DEPTH_CAM;
        cap.open(componetIDs);
        cap.setlaserPower(99);
        bool exit_main = false;
        cv::Mat depth,colorRoi,colorDepth,depthRoi;
        cv::Rect rects = cv::Rect(RECT_LEFT_UP_X_POINT,RECT_LEFT_UP_Y_POINT,RECT_BOX_WIDTH,RECT_BOX_HEIGHT);
        std::vector<short> distances;
        std::vector<index_value<int,cv::Point2i>> centerpoints;
        while(exit_main)
        {
            cap>>frame;
            depth = frame->depth;
            colorDepth =frame->render->Compute(frame->depth);
            depthRoi = depth(rects).clone();
            colorRoi = colorDepth(rects);
            cv::imshow("Pad",colorRoi);
            cv::imshow("fullImage",colorDepth);
            distancesFromCamToPad(distances,centerpoints,depthRoi);
            for(size_t i = 0; i < distances.size(); i++)
            {
                std::cout<<"µÚ"<<i<<"ÇøÓò¾àÀë= "<< distances[i]<<std::endl;
            }
            cvWait(exit_main);

        }
    }
    void test_recognition_object()
    {
     /*
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
    */
    }


}
