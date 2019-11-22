#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <stdio.h>
#include <time.h>

#define pi 3.14159265
#define ENABLE_LOG 1
#define LOG(msg) std::cout << msg << std::endl

class Equi2Rect
{
public:
    Equi2Rect()
    {
        w = 960;
        h = 1080;

        //specify viewing direction
        yaw = radians(20.0);  //pan
        pitch = radians(10.0); //tilt
        roll = radians(0.0);

        //create rotation matrix
        Rot = eul2rotm(pitch, yaw, roll);

        //specify focal length of the final pinhole image
        f = 672;

        // create camera matrix K
        K = (cv::Mat_<double>(3, 3) << f, 0, w / 2,
             0, f, h / 2,
             0, 0, 1);

        //initialize result image
        img_interp = cv::Mat(h, w, CV_8UC3, cv::Scalar(0, 0, 0));
    }

    cv::Mat img_src;
    //result image
    cv::Mat img_interp;

    cv::Mat eul2rotm(double rotx, double roty, double rotz);
    void bilinear_interpolation();
    std::pair<double, double> reprojection(int x_img, int y_img);

private:
    double yaw, pitch, roll;
    cv::Mat Rot;
    double f;
    cv::Mat K, K_inv;
    int w, h; //size of the output viewport

    double radians(double degrees)
    {
        return degrees * pi / 180;
    }
};

cv::Mat Equi2Rect::eul2rotm(double rotx, double roty, double rotz)
{

    cv::Mat R_x = (cv::Mat_<double>(3, 3) << 1, 0, 0,
                   0, cos(rotx), -sin(rotx),
                   0, sin(rotx), cos(rotx));

    cv::Mat R_y = (cv::Mat_<double>(3, 3) << cos(roty), 0, sin(roty),
                   0, 1, 0,
                   -sin(roty), 0, cos(roty));

    cv::Mat R_z = (cv::Mat_<double>(3, 3) << cos(rotz), -sin(rotz), 0,
                   sin(rotz), cos(rotz), 0,
                   0, 0, 1);

    cv::Mat R = R_z * R_y * R_x;

    return R;
}

std::pair<double, double> Equi2Rect::reprojection(int x_img, int y_img)
{

    cv::Mat xyz, xyz_norm;
    cv::Mat ray3d, RK;
    double xp, yp, zp, theta, phi;
    double x_sphere, y_sphere;

    xyz = (cv::Mat_<double>(3, 1) << (double)x_img, (double)y_img, 1);
    xyz_norm = xyz / norm(xyz);
    RK = Rot * K.inv();
    ray3d = RK * xyz_norm;

    //get 3d spherical coordinates
    xp = ray3d.at<double>(0, 0);
    yp = ray3d.at<double>(0, 1);
    zp = ray3d.at<double>(0, 2);
    //inverse formula for spherical projection, reference Szeliski book "Computer Vision: Algorithms and Applications" p439.
    theta = atan2(yp, sqrt(xp * xp + zp * zp));
    phi = atan2(xp, zp);

    //get 2D point on equirectangular map
    x_sphere = (((phi * img_src.cols) / pi + img_src.cols) / 2);
    y_sphere = (theta + pi / 2) * img_src.rows / pi;

    return std::make_pair(x_sphere, y_sphere);
}

void Equi2Rect::bilinear_interpolation()
{

    std::pair<double, double> current_pos;
    double current_x, current_y;
    // variables for bilinear interpolation
    int top_left_x, top_left_y;
    double dx, dy, wtl, wtr, wbl, wbr;
    int b, g, r;
    cv::Vec3b value, bgr;

    //loop over every pixel in output rectlinear image
    for (int v = 0; v < h; ++v)
    {
        for (int u = 0; u < w; ++u)
        {

            //determine corresponding position in the equirectangular panorama
            current_pos = reprojection(u, v);
            //extract the x and y value of the position in the equirect. panorama
            current_x = current_pos.first;
            current_y = current_pos.second;

            // determine the nearest top left pixel for bilinear interpolation
            top_left_x = (int)current_x; //convert the subpixel value to a proper pixel value (top left pixel due to int() operator)
            top_left_y = (int)current_y;

            // if the current position exceeeds the panorama image limit -- leave pixel black and skip to next iteration
            if (current_x < 0 || top_left_x > img_src.cols - 1 || current_y < 0 || top_left_y > img_src.rows - 1)
            {
                continue;
            }

            // initialize weights for bilinear interpolation
            dx = current_x - top_left_x;
            dy = current_y - top_left_y;
            wtl = (1.0 - dx) * (1.0 - dy);
            wtr = dx * (1.0 - dy);
            wbl = (1.0 - dx) * dy;
            wbr = dx * dy;

            // determine subpixel value with bilinear interpolation
            bgr = wtl * img_src.at<cv::Vec3b>(top_left_y, top_left_x) + wtr * img_src.at<cv::Vec3b>(top_left_y, top_left_x + 1) +
                  wbl * img_src.at<cv::Vec3b>(top_left_y + 1, top_left_x) + wbr * img_src.at<cv::Vec3b>(top_left_y + 1, top_left_x + 1);

            // paint the pixel in the output image with the calculated value
            img_interp.at<cv::Vec3b>(cv::Point(u, v)) = bgr;
        }
    }
    return;
}

int main(int argc, const char **argv)
{

    clock_t tStart = clock();

    Equi2Rect equi2rect;

    equi2rect.img_src = cv::imread("../images/pano.jpg", cv::IMREAD_COLOR);
    if (equi2rect.img_src.empty())
    {
        std::cout << "Error: Could not load image!" << std::endl;
        //system("pause")
        return -1;
    }

    equi2rect.bilinear_interpolation();

    printf("Time taken: %.2fs\n", (double)(clock() - tStart) / CLOCKS_PER_SEC);

    cv::imwrite( "../images/pano_rect.jpg", equi2rect.img_interp);

#if ENABLE_LOG
    cv::namedWindow("image", cv::WINDOW_AUTOSIZE);
    cv::imshow("image", equi2rect.img_src);
    cv::waitKey(0);
    cv::destroyWindow("image");
#endif

#if ENABLE_LOG
    cv::namedWindow("projected image", cv::WINDOW_AUTOSIZE);
    cv::imshow("projected image", equi2rect.img_interp);
    cv::waitKey(0);
    cv::destroyWindow("projected image");
#endif

    return 0;
}