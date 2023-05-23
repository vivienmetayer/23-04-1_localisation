#ifndef INC_23_04_1_LOCALISATION_CAM3D_H
#define INC_23_04_1_LOCALISATION_CAM3D_H

#include <vector>
#include <memory>
#include <opencv2/opencv.hpp>
#include "ArenaApi.h"

#ifdef CAM3DLIB
#define DLL_EXPORT_CAM3D __declspec(dllexport)
#else
#define DLL_EXPORT_CAM3D __declspec(dllimport)
#endif

class DLL_EXPORT_CAM3D Cam3d {
public:
    Cam3d(Arena::ISystem *system, int deviceIndex);
    ~Cam3d();

    void getNode(std::string nodeName, std::string &nodeValue);
    void setNode(std::string nodeName, std::string nodeValue);
    void startStream();
    void stopStream();
    int getData(std::vector<cv::Point3d> &points, uint64_t timeout = 1000);

private:
    Arena::ISystem *_system;
    Arena::IDevice *_device;
};


#endif //INC_23_04_1_LOCALISATION_CAM3D_H
