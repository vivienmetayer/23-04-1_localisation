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








