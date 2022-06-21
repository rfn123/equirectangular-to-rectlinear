#ifndef EQUI2RECT_HPP
#define EQUI2RECT_HPP

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

struct ViewPortSettings
{
    double pan_angle;
    double tilt_angle;
    double roll_angle;
    int width;
    int height;
};

class Equi2Rect
{
public:
    Equi2Rect();
    auto save_rectlinear_image() -> void;
    auto show_rectlinear_image() -> void;

private:
    auto eul2rotm(double rotx, double roty, double rotz) -> cv::Mat;
    auto bilinear_interpolation() -> void;
    auto reprojection(int x_img, int y_img) -> cv::Vec2d;

    ViewPortSettings viewport;
    int focal_length;
    cv::Mat Rot;
    cv::Mat K;
    cv::Mat img_src;
    cv::Mat img_dst;
    std::string img_dst_path;
};

#endif