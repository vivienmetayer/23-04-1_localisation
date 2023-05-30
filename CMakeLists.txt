cmake_minimum_required(VERSION 3.20)
project(23_04_1_localisation)

set(CMAKE_CXX_STANDARD 20)

# add OpenCV
set(OpenCV_DIR "" CACHE PATH "Path to OpenCVConfig.cmake")
find_package(OpenCV REQUIRED)

# add Arena SDK
set(ArenaSDK_INCLUDE_DIR "" CACHE PATH "Path to Arena SDK include folder")
set(ArenaSDK_GenTL_INCLUDE_DIR "" CACHE PATH "Path to Arena SDK include folder")
set(ArenaSDK_Save_INCLUDE_DIR "" CACHE PATH "Path to Arena SDK include folder")
set(ArenaSDK_LIB_DIR "" CACHE PATH "Path to Arena SDK lib folder")
set(GenICam_INCLUDE_DIR "" CACHE PATH "Path to GenICam include folder")
set(GenICam_LIB_DIR "" CACHE PATH "Path to GenICam lib folder")

# Triangulation
add_library(triangulationLib SHARED triangulation.cpp triangulation.h)
target_link_libraries(triangulationLib ${OpenCV_LIBS})

add_executable(TriangulationTest main.cpp)
target_link_libraries(TriangulationTest triangulationLib ${OpenCV_LIBS})

# Cam 3D
add_library(Cam3dLib SHARED Cam3d.cpp Cam3d.h Cam3d_functions.cpp Cam3d_functions.h ArenaCam.cpp ArenaCam.h Cam2d.cpp Cam2d.h)
target_link_libraries(Cam3dLib ${OpenCV_LIBS})
target_include_directories(Cam3dLib PUBLIC ${ArenaSDK_INCLUDE_DIR} ${GenICam_INCLUDE_DIR})
target_link_directories(Cam3dLib PUBLIC ${ArenaSDK_LIB_DIR} ${GenICam_LIB_DIR})
if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    target_link_libraries(Cam3dLib Arenad_v140)
else()
    target_link_libraries(Cam3dLib Arena_v140)
endif()
target_compile_definitions(Cam3dLib PRIVATE CAM3DLIB)

add_executable(Cam3DTest cam3d_main.cpp)
target_link_libraries(Cam3DTest Cam3dLib ${OpenCV_LIBS})
target_compile_definitions(Cam3DTest PUBLIC GENICAM_USER_ACCEPTS_ANY_COMPILER)

