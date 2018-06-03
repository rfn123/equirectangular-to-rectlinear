# Project an equirectlinear image back to rectlinear
The c++ code uses OpenCV to read/ output the images.        
The input should be a 2:1 360x180 degree field of view image to work properly.

## Build & use code

```
mkdir build
cd build
make
./equirectangular2rectlinear
```
If ```ENABLE_LOG``` is set to 1, a window will pop up, showing the original panorama. After pressing any key, the rectlinear image is generated and also shown in a window (might take 3-5 seconds)

## Extract rectlinear image from panorama

The code merely concentrates on how to obtain a rectlinear image (undistorted for our human eye) manually without fancy 3D rendering and texture mapping. 
The maths behind this is pretty staight forward: The panorama is reprojected on the final image plane by determining the normalized 3D ray for each pixel in the rectlinear output image and converting it to spherical coordinates. 
With these spherical coordinates we obtain the latitude and longitude of the point on the sphere and compute the corresponding position on the equirectangular panorama to determine the source pixel intensity value.
After acquiring the correspondence between output image pixel and equirectangular source pixel,
bilinear interpolation is used to reproject the image.

