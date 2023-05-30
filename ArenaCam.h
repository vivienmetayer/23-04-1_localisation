#ifndef INC_23_04_1_LOCALISATION_ARENACAM_H
#define INC_23_04_1_LOCALISATION_ARENACAM_H

#include <vector>
#include <memory>
#include <opencv2/opencv.hpp>
#include "ArenaApi.h"

#ifdef CAM3DLIB
#define DLL_EXPORT_CAM3D __declspec(dllexport)
#else
#define DLL_EXPORT_CAM3D __declspec(dllimport)
#endif


class DLL_EXPORT_CAM3D ArenaCam {
public:
    ArenaCam(Arena::ISystem *system, int deviceIndex);
    ArenaCam(Arena::ISystem *system, int deviceIndex,
            uint64_t forcedIp, uint64_t subnetMask, uint64_t gateway);
    ~ArenaCam();

    void setNodeStr(const std::string& nodeName, const std::string& nodeValue);
    void setNodeInt(const std::string& nodeName, int64_t nodeValue);
    void setNodeBool(const std::string& nodeName, bool nodeValue);
    void getNodeStr(const std::string& nodeName, std::string& nodeValue);
    void getNodeInt(const std::string& nodeName, int64_t *nodeValue);
    void getNodeBool(const std::string& nodeName, bool *nodeValue);
    void startStream();
    void stopStream();

protected:
    Arena::ISystem *_system;
    Arena::IDevice *_device;
};


#endif //INC_23_04_1_LOCALISATION_ARENACAM_H
