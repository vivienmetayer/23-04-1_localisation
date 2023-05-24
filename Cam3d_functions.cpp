#include "Cam3d_functions.h"

Arena::ISystem* CreateArenaSystem() {
    Arena::ISystem* system = Arena::OpenSystem();
    return system;
}

void DestroyArenaSystem(Arena::ISystem *system) {
    Arena::CloseSystem(system);
}

int getNumDevices(Arena::ISystem *system) {
    Arena::InterfaceInfo interfaceInfo;
    system->UpdateDevices(100);
    return (int)system->GetDevices().size();
}

Cam3d* createCam3d(int a) {
    return nullptr;
}

std::string getDeviceIPAddress(Arena::ISystem *system, int deviceIndex) {
    Arena::InterfaceInfo interfaceInfo;
    system->UpdateDevices(100);
    std::vector<Arena::DeviceInfo> deviceInfos = system->GetDevices();
    return const_cast<char *>(deviceInfos[deviceIndex].IpAddressStr().c_str());
}

std::string getDeviceModelName(Arena::ISystem *system, int deviceIndex) {
    Arena::InterfaceInfo interfaceInfo;
    system->UpdateDevices(100);
    std::vector<Arena::DeviceInfo> deviceInfos = system->GetDevices();
    return const_cast<char *>(deviceInfos[deviceIndex].ModelName().c_str());
}

Cam3d* createCam3d(Arena::ISystem *system, int deviceIndex) {
    return new Cam3d(system, deviceIndex);
}

Cam3d* createCam3dForcedIP(Arena::ISystem *system, int deviceIndex,
                           uint64_t forcedIp, uint64_t subnetMask, uint64_t gateway) {
    return new Cam3d(system, deviceIndex, forcedIp, subnetMask, gateway);
}

void destroyCam3d(Cam3d *cam3d) {
    delete cam3d;
}

void setNode(Cam3d *cam3d, std::string nodeName, std::string nodeValue) {
    cam3d->setNode(nodeName, nodeValue);
}

void startStream(Cam3d *cam3d) {
    cam3d->startStream();
}

void stopStream(Cam3d *cam3d) {
    cam3d->stopStream();
}

int getData(Cam3d *cam3d, double* points, uint64_t timeout) {
    std::vector<cv::Point3d> cvPoints;
    int numPoints = cam3d->getData(cvPoints, timeout);
    for (int i = 0; i < numPoints; i++) {
        points[i * 3] = cvPoints[i].x;
        points[i * 3 + 1] = cvPoints[i].y;
        points[i * 3 + 2] = cvPoints[i].z;
    }
    return numPoints;
}