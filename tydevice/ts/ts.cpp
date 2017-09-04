#include "ts.h"

using namespace neolix;

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

