cmake_minimum_required(VERSION 3.0)
project(backprojection)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
find_package( OpenCV REQUIRED )

set(SOURCE_FILES backprojection.cpp)
add_executable(backprojection ${SOURCE_FILES})

target_link_libraries(backprojection ${OpenCV_LIBS} )
