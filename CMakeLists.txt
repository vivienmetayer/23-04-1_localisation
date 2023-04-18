cmake_minimum_required(VERSION 3.20)
project(23_04_1_localisation)

set(CMAKE_CXX_STANDARD 20)

# add OpenCV
set(OpenCV_DIR "" CACHE PATH "Path to OpenCVConfig.cmake")
find_package(OpenCV REQUIRED)

add_library(triangulationLib SHARED triangulation.cpp triangulation.h)
target_link_libraries(triangulationLib ${OpenCV_LIBS})

add_executable(23_04_1_localisation main.cpp)
target_link_libraries(23_04_1_localisation triangulationLib ${OpenCV_LIBS})
