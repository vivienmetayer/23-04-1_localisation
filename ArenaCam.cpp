#include "ArenaCam.h"

ArenaCam::ArenaCam(Arena::ISystem *system, int deviceIndex) {
    _system = system;
    Arena::InterfaceInfo interfaceInfo;
    _system->UpdateDevices(100);
    std::vector<Arena::DeviceInfo> deviceInfos = _system->GetDevices();
    _device = _system->CreateDevice(deviceInfos[deviceIndex]);
}

ArenaCam::ArenaCam(Arena::ISystem *system, int deviceIndex, uint64_t forcedIp, uint64_t subnetMask, uint64_t gateway) {
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

ArenaCam::~ArenaCam() {
    _system->DestroyDevice(_device);
}

void ArenaCam::setNodeStr(const std::string &nodeName, const std::string &nodeValue) {
    GenApi::INodeMap* pNodeMap = _device->GetNodeMap();
    Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, nodeName.c_str(), nodeValue.c_str());
}

void ArenaCam::setNodeInt(const std::string &nodeName, int64_t nodeValue) {
    GenApi::INodeMap* pNodeMap = _device->GetNodeMap();
    Arena::SetNodeValue<int64_t>(pNodeMap, nodeName.c_str(), nodeValue);
}

void ArenaCam::setNodeBool(const std::string &nodeName, bool nodeValue) {
    GenApi::INodeMap* pNodeMap = _device->GetNodeMap();
    Arena::SetNodeValue<bool>(pNodeMap, nodeName.c_str(), nodeValue);
}

void ArenaCam::setNodeDouble(const std::string& nodeName, double nodeValue) {
    GenApi::INodeMap* pNodeMap = _device->GetNodeMap();
    Arena::SetNodeValue<double>(pNodeMap, nodeName.c_str(), nodeValue);
}

void ArenaCam::startStream() {
    _device->StartStream();
}

void ArenaCam::stopStream() {
    _device->StopStream();
}

void ArenaCam::getNodeStr(const std::string &nodeName, std::string &nodeValue) {
    GenApi::INodeMap* pNodeMap = _device->GetNodeMap();
    GenICam::gcstring value = Arena::GetNodeValue<GenICam::gcstring>(pNodeMap, nodeName.c_str());
    nodeValue = value.c_str();
}

void ArenaCam::getNodeInt(const std::string &nodeName, int64_t *nodeValue) {
    GenApi::INodeMap* pNodeMap = _device->GetNodeMap();
    *nodeValue = Arena::GetNodeValue<int64_t>(pNodeMap, nodeName.c_str());
}

void ArenaCam::getNodeBool(const std::string &nodeName, bool *nodeValue) {
    GenApi::INodeMap* pNodeMap = _device->GetNodeMap();
    *nodeValue = Arena::GetNodeValue<bool>(pNodeMap, nodeName.c_str());
}

void ArenaCam::getNodeDouble(const std::string& nodeName, double *nodeValue) {
    GenApi::INodeMap* pNodeMap = _device->GetNodeMap();
    *nodeValue = Arena::GetNodeValue<double>(pNodeMap, nodeName.c_str());
}
