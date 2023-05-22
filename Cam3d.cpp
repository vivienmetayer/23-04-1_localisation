#include <fstream>
#include "Cam3d.h"

std::vector<std::unique_ptr<Cam3d>> Cam3d::_instances;

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





