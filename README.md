# Project an equirectlinear image back to rectlinear
The c++ code uses OpenCV to read/ output the images.        
The input should be a 2:1 360x180 degrees field of view image to work properly.

####Update 11/23/2019
Realized how shitty the code was and refactored it a bit to make it much more readable and compatible to newer OpenCV versions. Still needs optimization though.

## Build & use code

```
mkdir build
cd build
cmake ..
make
./equi2rect
```
If ```ENABLE_LOG``` is set to 1, a window will pop up, showing the original panorama. After pressing any key, the rectlinear image is generated and also shown in a window (might take 3-5 seconds)

Example images:      
![original image](/images/pano.jpg)
![rectified image](/images/pano_rect.jpg "Rectified image")
Rectified image with 20 degrees pan angle and 10 degrees tilt angle. 

## Extract rectlinear image from panorama

The code merely concentrates on how to obtain a rectlinear image (undistorted for our human eye) manually without fancy 3D rendering and texture mapping. 
The maths behind this is pretty staight forward [1]: The panorama is reprojected on the final image plane by determining the normalized 3D ray for each pixel in the rectlinear output image and converting it to spherical coordinates. 
With these spherical coordinates we obtain the latitude and longitude of the point on the sphere and compute the corresponding position on the equirectangular panorama to determine the source pixel intensity value.
After acquiring the correspondence between output image pixel and equirectangular source pixel,
bilinear interpolation is used to reproject the image.

[1]: Szeliski book "Computer Vision: Algorithms and Applications" p439

## TODO
* parse commandline input arguments
* further optimize code 
