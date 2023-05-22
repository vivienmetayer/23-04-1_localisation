#ifndef INC_23_04_1_LOCALISATION_CAM3D_H
#define INC_23_04_1_LOCALISATION_CAM3D_H

#include <vector>
#include <memory>

#ifdef CAM3DLIB
#define DLL_EXPORT __declspec(dllexport)
#else
#define DLL_EXPORT __declspec(dllimport)
#endif

class DLL_EXPORT Cam3d {
public:
    Cam3d();
    ~Cam3d();

    int getA() const { return _a; }
    void setA(int a) { _a = a; }

    static std::vector<std::unique_ptr<Cam3d>>& getInstances() { return _instances; }
    static Cam3d* create();
    static void remove();

private:
    int _a;
    static std::vector<std::unique_ptr<Cam3d>> _instances;
};


#endif //INC_23_04_1_LOCALISATION_CAM3D_H
