# Extract rectlinear image from panorama

**Disclaimer:** The code merely concentrates on how to obtain a rectlinear image (undistorted for our human eye) manually without fancy 3D rendering and texture mapping. It is not optimized to be efficient, just a small experiment project to understand the theory better.


The maths behind the reprojection is pretty staight forward [1]: The panorama is reprojected on the final image plane by determining the normalized 3D ray for each pixel in the rectlinear output image and converting it to spherical coordinates.          
With these spherical coordinates we obtain the latitude and longitude of the point on the sphere and compute the corresponding position on the equirectangular panorama to determine the source pixel intensity value.
After acquiring the correspondence between output image pixel and equirectangular source pixel,
bilinear interpolation is used to reproject the image.

The c++ code uses OpenCV (>= 3.0) to read/ output the images.        
The input image should be a 2:1 360x180 degrees field of view image to work properly.

[1]: Szeliski book "Computer Vision: Algorithms and Applications" p439


# Required dependencies
- OpenCV >= 3.0 (should be compiled with GUI support, using e.g. libgtk2.0, otherwise `cv::imshow` will fail)
- libyaml-cpp-dev (`sudo apt install libyaml-cpp-dev`)

# Build & use code
From the project root, run:
```
mkdir build
cd build
cmake ..
make
cd .. && ./build/equi2rect_example
```
The output image is written to the specified path in the `config.yaml` file. A window will additionally pop up, showing the output image (might take some seconds).
You can play around with the values in the config file to get different output images.

Example images:      
![original image](/images/pano.jpg)
![rectified image](/images/pano_rect.jpg "Rectified image")
Rectified image with 20 degrees pan angle and 10 degrees tilt angle. 

## Config file for output settings
```yaml
camera:
  focal_length: 672
files:
  img_dst_path: images/pano_rect.jpg # relativ path from directory where binary is executed
  img_src_path: images/pano.jpg
viewport:
  height: 1080 # px
  # angles are in radians
  pan_angle: 0.349066
  roll_angle: 0.0
  tilt_angle: 0.174533
  width: 960 # px
```
