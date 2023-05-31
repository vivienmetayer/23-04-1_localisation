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

void getDeviceIPAddress(Arena::ISystem *system, int deviceIndex, char *ipAddressOut) {
    Arena::InterfaceInfo interfaceInfo;
    system->UpdateDevices(100);
    std::vector<Arena::DeviceInfo> deviceInfos = system->GetDevices();
    std::string ipAddress = deviceInfos[deviceIndex].IpAddressStr().c_str();
    for (int i = 0; i < ipAddress.size(); i++) {
        ipAddressOut[i] = ipAddress[i];
    }
}

void getDeviceModelName(Arena::ISystem *system, int deviceIndex, char *modelNameOut) {
    Arena::InterfaceInfo interfaceInfo;
    system->UpdateDevices(100);
    std::vector<Arena::DeviceInfo> deviceInfos = system->GetDevices();
    std::string modelName = deviceInfos[deviceIndex].ModelName().c_str();
    for (int i = 0; i < modelName.size(); ++i) {
        modelNameOut[i] = modelName[i];
    }
}

ArenaCam* createCam(Arena::ISystem *system, int deviceIndex) {
    return new ArenaCam(system, deviceIndex);
}

ArenaCam* createCamForcedIP(Arena::ISystem *system, int deviceIndex,
                           uint64_t forcedIp, uint64_t subnetMask, uint64_t gateway) {
    return new ArenaCam(system, deviceIndex, forcedIp, subnetMask, gateway);
}

void destroyCam(ArenaCam *arenaCam) {
    delete arenaCam;
}

void setNodeStr(ArenaCam *cam, const char* nodeName, const char* nodeValue) {
    cam->setNodeStr(nodeName, nodeValue);
}

void setNodeInt(ArenaCam *cam, const char* nodeName, int64_t nodeValue) {
    cam->setNodeInt(nodeName, nodeValue);
}

void setNodeBool(ArenaCam *cam, const char* nodeName, bool nodeValue) {
    cam->setNodeBool(nodeName, nodeValue);
}

void getNodeStr(ArenaCam *cam, const char* nodeName, char* nodeValue) {
    std::string str;
    cam->getNodeStr(nodeName, str);
    for (int i = 0; i < str.size(); i++) {
        nodeValue[i] = str[i];
    }
}

void getNodeInt(ArenaCam *cam, const char* nodeName, int64_t *nodeValue) {
    cam->getNodeInt(nodeName, nodeValue);
}

void getNodeBool(ArenaCam *cam, const char* nodeName, bool *nodeValue) {
    cam->getNodeBool(nodeName, nodeValue);
}

void startStream(ArenaCam *cam) {
    cam->startStream();
}

void stopStream(ArenaCam *cam) {
    cam->stopStream();
}

int getData(Cam3d *cam3d, double* points, uint16_t *luminance, uint64_t timeout) {
    std::vector<cv::Point3d> cvPoints;
    std::vector<std::vector<uint16_t>> lum;
    int numPoints = cam3d->getData(cvPoints, lum, timeout);

    // copy points to points array
    for (int i = 0; i < numPoints; i++) {
        points[i * 3] = cvPoints[i].x;
        points[i * 3 + 1] = cvPoints[i].y;
        points[i * 3 + 2] = cvPoints[i].z;
    }

    // copy luminance to lum array
    for (int i = 0; i < lum.size(); i++) {
        for (int j = 0; j < lum[i].size(); j++) {
            luminance[i * lum[i].size() + j] = lum[i][j];
        }
    }
    return numPoints;
}

int getImage(Cam2d *cam2d, unsigned char* imagePtr, int width, int height, int stride, uint64_t timeout) {
    cv::Mat image(height, width, CV_8UC4, imagePtr, stride);
    cam2d->getImage(image, timeout);
    return 0;
}