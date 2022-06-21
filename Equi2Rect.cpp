#include "Equi2Rect.hpp"
#include <yaml-cpp/yaml.h>
#include <math.h>

Equi2Rect::Equi2Rect()
{
    YAML::Node config = YAML::LoadFile("config.yaml");

    // viewport size
    viewport.width = config["viewport"]["width"].as<int>();
    viewport.height = config["viewport"]["height"].as<int>();

    // specify viewing direction
    viewport.pan_angle = config["viewport"]["pan_angle"].as<double>();
    viewport.tilt_angle = config["viewport"]["tilt_angle"].as<double>();
    viewport.roll_angle = config["viewport"]["roll_angle"].as<double>();

    // create rotation matrix
    Rot = eul2rotm(viewport.tilt_angle, viewport.pan_angle, viewport.roll_angle);

    // specify focal length of the final pinhole image
    focal_length = config["camera"]["focal_length"].as<int>();
    ;

    // create camera matrix K
    K = (cv::Mat_<double>(3, 3) << focal_length, 0, viewport.width / 2,
         0, focal_length, viewport.height / 2,
         0, 0, 1);

    // read src image
    img_src = cv::imread(config["files"]["img_src_path"].as<std::string>(), cv::IMREAD_COLOR);
    if (img_src.empty())
    {
        throw std::invalid_argument("Error: Could not load image!");
    }

    // initialize result image
    img_dst = cv::Mat(viewport.height, viewport.width, CV_8UC3, cv::Scalar(0, 0, 0));
    img_dst_path = config["files"]["img_dst_path"].as<std::string>();
}

auto Equi2Rect::save_rectlinear_image() -> void
{
    this->bilinear_interpolation();
    cv::imwrite(img_dst_path, img_dst);
}

auto Equi2Rect::show_rectlinear_image() -> void
{
    cv::namedWindow("projected image", cv::WINDOW_AUTOSIZE);
    cv::imshow("projected image", img_dst);
    cv::waitKey(0);
    cv::destroyWindow("projected image");
}

auto Equi2Rect::eul2rotm(double rotx, double roty, double rotz) -> cv::Mat
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

auto Equi2Rect::reprojection(int x_img, int y_img) -> cv::Vec2d
{
    cv::Mat xyz = (cv::Mat_<double>(3, 1) << (double)x_img, (double)y_img, 1);
    cv::Mat ray3d = Rot * K.inv() * xyz / norm(xyz);

    // get 3d spherical coordinates
    double xp = ray3d.at<double>(0);
    double yp = ray3d.at<double>(1);
    double zp = ray3d.at<double>(2);
    // inverse formula for spherical projection, reference Szeliski book "Computer Vision: Algorithms and Applications" p439.
    double theta = atan2(yp, sqrt(xp * xp + zp * zp));
    double phi = atan2(xp, zp);

    // get 2D point on equirectangular map
    double x_sphere = (((phi * img_src.cols) / M_PI + img_src.cols) / 2);
    double y_sphere = (theta + M_PI / 2) * img_src.rows / M_PI;

    return cv::Vec2d(x_sphere, y_sphere);
}

auto Equi2Rect::bilinear_interpolation() -> void
{
    cv::Vec2d current_pos;
    // variables for bilinear interpolation
    int top_left_x, top_left_y;
    double dx, dy, wtl, wtr, wbl, wbr;
    cv::Vec3b value, bgr;

    // loop over every pixel in output rectlinear image
    for (int v = 0; v < viewport.height; ++v)
    {
        for (int u = 0; u < viewport.width; ++u)
        {

            // determine corresponding position in the equirectangular panorama
            current_pos = reprojection(u, v);

            // determine the nearest top left pixel for bilinear interpolation
            top_left_x = static_cast<int>(current_pos[0]); // convert the subpixel value to a proper pixel value (top left pixel due to int() operator)
            top_left_y = static_cast<int>(current_pos[1]);

            // if the current position exceeeds the panorama image limit -- leave pixel black and skip to next iteration
            if (current_pos[0] < 0 || top_left_x > img_src.cols - 1 || current_pos[1] < 0 || top_left_y > img_src.rows - 1)
            {
                continue;
            }

            // initialize weights for bilinear interpolation
            dx = current_pos[0] - top_left_x;
            dy = current_pos[1] - top_left_y;
            wtl = (1.0 - dx) * (1.0 - dy);
            wtr = dx * (1.0 - dy);
            wbl = (1.0 - dx) * dy;
            wbr = dx * dy;

            // determine subpixel value with bilinear interpolation
            bgr = wtl * img_src.at<cv::Vec3b>(top_left_y, top_left_x) + wtr * img_src.at<cv::Vec3b>(top_left_y, top_left_x + 1) +
                  wbl * img_src.at<cv::Vec3b>(top_left_y + 1, top_left_x) + wbr * img_src.at<cv::Vec3b>(top_left_y + 1, top_left_x + 1);

            // paint the pixel in the output image with the calculated value
            img_dst.at<cv::Vec3b>(cv::Point(u, v)) = bgr;
        }
    }
    return;
}