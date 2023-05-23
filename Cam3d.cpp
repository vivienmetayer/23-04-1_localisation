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

Cam3d::~Cam3d() {
    _system->DestroyDevice(_device);
}

void Cam3d::getNode(std::string nodeName, std::string &nodeValue) {
    GenApi::INodeMap* pNodeMap = _device->GetNodeMap();
    GenICam::gcstring value = Arena::GetNodeValue<GenICam::gcstring>(pNodeMap, nodeName.c_str());
    nodeValue = value.c_str();
}

void Cam3d::setNode(std::string nodeName, std::string nodeValue) {
    GenApi::INodeMap* pNodeMap = _device->GetNodeMap();
    Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, nodeName.c_str(), nodeValue.c_str());
}

void Cam3d::startStream() {
    _device->StartStream();
}

void Cam3d::stopStream() {
    _device->StopStream();
}

void Cam3d::getData(std::vector<std::vector<cv::Point3d>> &points, uint64_t timeout) {
    Arena::IImage* pImage = _device->GetImage(timeout);

    size_t width = pImage->GetWidth();
    size_t height = pImage->GetHeight();
    size_t srcBpp = pImage->GetBitsPerPixel();
    size_t srcPixelSize = srcBpp / 8;

    GenApi::INodeMap* pNodeMap = _device->GetNodeMap();
    float scale = static_cast<float>(Arena::GetNodeValue<double>(pNodeMap, "Scan3dCoordinateScale"));

    const uint8_t* dataPtr = pImage->GetData();
    const uint8_t* pIn = dataPtr;

    points.resize(height);
    for (size_t i = 0; i < height; i++) {
        points[i].resize(width);
        for (size_t j = 0; j < width; ++j) {
            int16_t x = *reinterpret_cast<const int16_t*>(pIn);
            x = int16_t(double(x) * scale);
            int16_t y = *reinterpret_cast<const int16_t*>((pIn + 2));
            y = int16_t(double(y) * scale);
            int16_t z = *reinterpret_cast<const int16_t*>((pIn + 4));
            z = int16_t(double(z) * scale);

            points[i][j] = cv::Point3d(x, y, z);
            pIn += srcPixelSize;
        }
    }
}










