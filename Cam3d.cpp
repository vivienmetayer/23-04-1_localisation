#include "Cam3d.h"


#include "opencv2/opencv.hpp"
#include <fstream>

Cam3d::Cam3d(Arena::ISystem *system, int deviceIndex) {
    _system = system;
    Arena::InterfaceInfo interfaceInfo;
    _system->UpdateDevices(100);
    std::vector<Arena::DeviceInfo> deviceInfos = _system->GetDevices();
    _device = _system->CreateDevice(deviceInfos[deviceIndex]);
}

Cam3d::Cam3d(Arena::ISystem *system, int deviceIndex,
             uint64_t forcedIp, uint64_t subnetMask, uint64_t gateway) {
    _system = system;
    Arena::InterfaceInfo interfaceInfo;
    _system->UpdateDevices(100);
    std::vector<Arena::DeviceInfo> deviceInfos = _system->GetDevices();

    uint64_t macAddress = deviceInfos[deviceIndex].MacAddress();

    _system->ForceIp(macAddress, forcedIp, subnetMask, gateway);

    _system->UpdateDevices(100);
    deviceInfos = _system->GetDevices();

    //
    auto it = std::find_if(
            deviceInfos.begin(),
            deviceInfos.end(),
            [&macAddress](Arena::DeviceInfo deviceInfo) {
                return deviceInfo.MacAddress() == macAddress;
            });
    _device = _system->CreateDevice(*it);

    Arena::SetNodeValue<bool>(_device->GetTLStreamNodeMap(), "StreamAutoNegotiatePacketSize", true);
    Arena::SetNodeValue<bool>(_device->GetTLStreamNodeMap(), "StreamPacketResendEnable", true);
}

Cam3d::~Cam3d() {
    _system->DestroyDevice(_device);
}

void Cam3d::getNode(const std::string& nodeName, std::string &nodeValue) {
    GenApi::INodeMap* pNodeMap = _device->GetNodeMap();
    GenICam::gcstring value = Arena::GetNodeValue<GenICam::gcstring>(pNodeMap, nodeName.c_str());
    nodeValue = value.c_str();
}

void Cam3d::setNode(const std::string& nodeName, const std::string& nodeValue) {
    GenApi::INodeMap* pNodeMap = _device->GetNodeMap();
    Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, nodeName.c_str(), nodeValue.c_str());
}

void Cam3d::startStream() {
    _device->StartStream();
}

void Cam3d::stopStream() {
    _device->StopStream();
}

int Cam3d::getData(std::vector<cv::Point3d> &points, uint64_t timeout) {
    Arena::IImage* pImage = _device->GetImage(timeout);

    size_t width = pImage->GetWidth();
    size_t height = pImage->GetHeight();
    size_t srcBpp = pImage->GetBitsPerPixel();
    size_t srcPixelSize = srcBpp / 8;

    GenApi::INodeMap* pNodeMap = _device->GetNodeMap();
    float scale = static_cast<float>(Arena::GetNodeValue<double>(pNodeMap, "Scan3dCoordinateScale"));

    const uint8_t* dataPtr = pImage->GetData();
    const uint8_t* pIn = dataPtr;

    points.resize(height * width);
    int numPoints = 0;
    for (size_t i = 0; i < height * width; i++) {
        int16_t ix = *reinterpret_cast<const int16_t*>(pIn);
//            x = int16_t(double(x) * scale);
        int16_t iy = *reinterpret_cast<const int16_t*>((pIn + 2));
//            y = int16_t(double(y) * scale);
        int16_t iz = *reinterpret_cast<const int16_t*>((pIn + 4));
//            z = int16_t(double(z) * scale);

        if (iz == -1)
            continue;

        uint16_t x = reinterpret_cast<const uint16_t&>(ix);
        uint16_t y = reinterpret_cast<const uint16_t&>(iy);
        uint16_t z = reinterpret_cast<const uint16_t&>(iz);
        points[numPoints] = cv::Point3d(x, y, z) * scale;
        pIn += srcPixelSize;
        numPoints++;
    }
    points.resize(numPoints);
    return numPoints;
}












