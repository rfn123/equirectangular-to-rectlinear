cmake_minimum_required(VERSION 3.10)
project(equirectangular2rectlinear)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
find_package(OpenCV REQUIRED )
find_package(yaml-cpp REQUIRED)

add_library(Equi2Rect
            Equi2Rect.hpp
            Equi2Rect.cpp)

target_link_libraries(Equi2Rect 
                    ${OpenCV_LIBS}
                    yaml-cpp )


add_executable(equi2rect_example main.cpp)

target_link_libraries(equi2rect_example Equi2Rect)
