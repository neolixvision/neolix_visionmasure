#ifndef XYZ_MAT_VIEWER_HPP_
#define XYZ_MAT_VIEWER_HPP_

#include <opencv2/opencv.hpp>
#include <string>
#include "DepthRender.hpp"


class OpencvViewer
{
public:
    static void drawText(cv::Mat& img, const std::string& text, const cv::Point loc,
                         double scale, const cv::Scalar& color, int thickness);

    virtual void show(const std::string& win, const cv::Mat& img);
    virtual void onMouseCallback(cv::Mat& img, int event, const cv::Point pnt);

private:
    static void __onMouseCallback(int event, int x, int y, int flags, void* ustc);

    cv::Mat _img;
    std::string _win;
    void*   _ustc;
};

inline void OpencvViewer::drawText(cv::Mat& img, const std::string& text
        , const cv::Point loc, double scale, const cv::Scalar& color, int thickness)
{
    cv::putText(img, text, loc, cv::FONT_HERSHEY_SIMPLEX, scale, color, thickness);
}



class DepthViewer : public OpencvViewer
{
public:
    static std::string depthStringAtLoc(const cv::Mat& img, const cv::Point pnt);

    virtual void show(const std::string& win, const cv::Mat& depthImage);
    virtual void onMouseCallback(cv::Mat& img, int event, const cv::Point pnt);

private:
    void drawFixLoc(cv::Mat& img);

    DepthRender _render;
    cv::Mat _img;
    cv::Point   _fixLoc;
};


#endif
