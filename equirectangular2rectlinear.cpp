#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <stdio.h>
#include <time.h>

#define pi 3.14159265
#define ENABLE_LOG 0 //if set to 1 then images (including intermediate results) are shown in a pop-up window
#define LOG(msg) std::cout << msg << std::endl 

using namespace cv;
using namespace std;

// initialize variables
int top, bottom, left, right;
int h1,w1;
int pad_h, pad_v;
double yaw,pitch,roll;
Mat Rot;
double f;
Mat K, K_inv;
pair<double,double> current_pos;
double current_x, current_y;
Mat xyz, xyz_norm, xyz_norm_T;
Mat ray3d;
int w,h;
double xp,yp,zp,theta,phi;
double x_sphere, y_sphere;
Mat RK;

// variables for bilinear interpolation
int top_left_x, top_left_y;
double dx,dy,wtl,wtr,wbl,wbr;
int b,g,r;
Vec3b value, bgr;

double radians(double degrees){
    return degrees*pi/180;
}

// calculates the rotation matrix given three euler angles
Mat eul2rotm(double rotx,double roty,double rotz) {

    Mat R_x = (Mat_<double>(3,3) <<
            1,       0,              0,
            0,       cos(rotx),   -sin(rotx),
            0,       sin(rotx),   cos(rotx)
    );

    Mat R_y = (Mat_<double>(3,3) <<
            cos(roty),    0,      sin(roty),
            0,               1,      0,
            -sin(roty),   0,      cos(roty)
    );

    Mat R_z = (Mat_<double>(3,3) <<
            cos(rotz),    -sin(rotz),      0,
            sin(rotz),    cos(rotz),       0,
            0,               0,                  1);

    Mat R = R_z * R_y * R_x;

    return R;
}

pair<double, double> reprojection(int x_img, int y_img, double f, Mat Rot, int w1, int h1, Mat K){
    // set the size of the output viewport
    w = 960;
    h = 1080;

    xyz = (Mat_<double>(3,1)<< (double)x_img, (double)y_img,1); 
    xyz_norm = xyz/norm(xyz); 
    RK = Rot * K.inv(); 
    ray3d = RK*xyz_norm; 

    //get 3d spherical coordinates
    xp = ray3d.at<double>(0,0);
    yp = ray3d.at<double>(0,1);
    zp = ray3d.at<double>(0,2);
    //inverse formula for spherical projection, reference Szeliski book "Computer Vision: Algorithms and Applications" p439.
    theta = atan2(yp,sqrt(xp*xp+zp*zp));
    phi = atan2(xp,zp);

    //get 2D point on equirectangular map
    x_sphere = (((phi*w1)/pi+w1)/2; 
    y_sphere = (theta+ pi/2)*h1/pi;

    return make_pair(x_sphere,y_sphere);
};

int main(int argc, const char** argv) {

    clock_t tStart = clock();
//____________________________________________parameters____________________________________________________

    //specify viewing direction
    yaw = radians(30.0); //pan
    pitch = radians(0.0); //tilt
    roll = radians(0.0);

    //create rotation matrix
    Rot = eul2rotm(pitch,yaw,roll);

    //specify focal length of the final pinhole image
    //f=831.6; //60deg fov
    f=672; //110deg fov

    // create camera matrix K
    K = (Mat_<double>(3,3) << f,0,w/2,
         0,f,h/2,
         0,0,1);
//_______________________________________________________________________________________________________

    //initialize output image of the size 960x1080
    //pixels are painted black for initialization
    Mat img_interp(1080,960, CV_8UC3, Scalar(0,0,0));

    //read pano image
    Mat img_src = imread("/images/pano.jpg",CV_LOAD_IMAGE_COLOR);

    if(img_src.empty()) 
    {
        cout << "Error: Could not load image!" << endl;
        //system("pause")
        return -1;
    }

    //determine size of pano
    h1 = img_src.rows;
    w1 = img_src.cols;

    #if ENABLE_LOG
      namedWindow("image", CV_WINDOW_AUTOSIZE);
      imshow("image", img_src);
      waitKey(0);
      destroyWindow("image");
    #endif

    //loop over every pixel in output rectlinear image
    for (int v = 0; v < 1080; ++v) {
      for (int u = 0; u < 960; ++u) {

            //determine corresponding position in the equirectangular panorama
            current_pos = reprojection(u,v,f,Rot,w1,h1,K);
            //extract the x and y value of the position in the equirect. panorama
            current_x = current_pos.first;
            current_y = current_pos.second;

            // determine the nearest top left pixel for bilinear interpolation
            top_left_x = (int)current_x; //convert the subpixel value to a proper pixel value (top left pixel due to int() operator)
            top_left_y = (int)current_y;

            // if the current position exceeeds the panorama image limit -- leave pixel black and skip to next iteration
            if(current_x<0||top_left_x>w1-1||current_y<0||top_left_y>h1-1){
              continue;
            }

            // initialize weights for bilinear interpolation
            dx = current_x - top_left_x;
            dy = current_y - top_left_y;
            wtl=(1.0-dx)*(1.0-dy); 
            wtr = dx*(1.0-dy); 
            wbl = (1.0-dx)*dy; 
            wbr = dx*dy; 

            // determine subpixel value with bilinear interpolation
            bgr=wtl*img_src.at<Vec3b>(top_left_y,top_left_x)+wtr*img_src.at<Vec3b>(top_left_y,top_left_x+1)+
               wbl*img_src.at<Vec3b>(top_left_y+1,top_left_x)+wbr*img_src.at<Vec3b>(top_left_y+1,top_left_x+1);

            // paint the pixel in the output image with the calculated value
            img_interp.at<Vec3b>(Point(u,v)) = bgr;

        }
    }

    printf("Time taken: %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);

    #if ENABLE_LOG
    namedWindow("projected image", CV_WINDOW_AUTOSIZE);
    imshow("projected image", img_interp);
    waitKey(0);
    destroyWindow("projected image");
    #endif

    return 0;

}