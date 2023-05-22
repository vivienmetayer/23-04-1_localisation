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

void destroyCam3d(Cam3d *cam3d) {
    delete cam3d;
}

void setNode(Cam3d *cam3d, std::string nodeName, std::string nodeValue) {
    cam3d->setNode(nodeName, nodeValue);
}