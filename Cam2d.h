//
// Created by Vivien on 30/05/2023.
//

#ifndef INC_23_04_1_LOCALISATION_CAM2D_H
#define INC_23_04_1_LOCALISATION_CAM2D_H

#include <vector>
#include <memory>
#include <opencv2/opencv.hpp>
#include "ArenaApi.h"
#include "ArenaCam.h"

#ifdef CAM3DLIB
#define DLL_EXPORT_CAM3D __declspec(dllexport)
#else
#define DLL_EXPORT_CAM3D __declspec(dllimport)
#endif

class DLL_EXPORT_CAM3D Cam2d : public ArenaCam{
public:
    Cam2d(Arena::ISystem *system, int deviceIndex);
    Cam2d(Arena::ISystem *system, int deviceIndex,
            uint64_t forcedIp, uint64_t subnetMask, uint64_t gateway);
    ~Cam2d();

    int getImage(cv::Mat &image, uint64_t timeout = 1000);
};


#endif //INC_23_04_1_LOCALISATION_CAM2D_H
