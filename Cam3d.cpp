#include "Cam3d.h"


#include "opencv2/opencv.hpp"
#include <fstream>

std::vector<std::unique_ptr<Cam3d>> Cam3d::_instances;
Arena::ISystem* Cam3d::_arenaSystem;

Cam3d::Cam3d() {
    // open num.txt file and write 1 to it
    std::ofstream file("num.txt");
    file << 1;
    file.close();
}

Cam3d::~Cam3d() {
    // open num.txt file and write 0 to it
    std::ofstream file("num.txt");
    file << 0;
    file.close();
}

Cam3d *Cam3d::create() {
    _instances.emplace_back(std::make_unique<Cam3d>());
    return _instances.back().get();
}

void Cam3d::remove() {
    _instances.pop_back();
}

void Cam3d::CreateArenaSystem()
{
    _arenaSystem = Arena::OpenSystem();
}

void Cam3d::DestroyArenaSystem()
{
    Arena::CloseSystem(_arenaSystem);
}

int Cam3d::getNumDevices()
{
    Arena::InterfaceInfo interfaceInfo;
    _arenaSystem->UpdateDevices(100);
    return (int)_arenaSystem->GetDevices().size();
}







