cmake_minimum_required(VERSION 3.0)
project(equirectangular2rectlinear)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
find_package( OpenCV REQUIRED )

set(SOURCE_FILES equirectangular2rectlinear.cpp)
add_executable(equirectangular2rectlinear ${SOURCE_FILES})

target_link_libraries(equirectangular2rectlinear ${OpenCV_LIBS} )
