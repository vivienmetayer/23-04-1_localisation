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

char* getDeviceIPAddress(Arena::ISystem *system, int deviceIndex) {
    Arena::InterfaceInfo interfaceInfo;
    system->UpdateDevices(100);
    std::vector<Arena::DeviceInfo> deviceInfos = system->GetDevices();
    return const_cast<char *>(deviceInfos[deviceIndex].IpAddressStr().c_str());
}