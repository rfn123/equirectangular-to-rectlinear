cmake_minimum_required(VERSION 3.0)
project(equirectangular2rectlinear)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
find_package( OpenCV REQUIRED )

add_executable(equi2rect equi2rect.cpp)

target_link_libraries(equi2rect ${OpenCV_LIBS} )
