#ifndef INC_23_04_1_LOCALISATION_CAM3D_FUNCTIONS_H
#define INC_23_04_1_LOCALISATION_CAM3D_FUNCTIONS_H

#include "Cam3d.h"

#define DLL_EXPORT extern "C" __declspec(dllexport)

DLL_EXPORT int test();

DLL_EXPORT Cam3d* createCam3d(int a);

#endif //INC_23_04_1_LOCALISATION_CAM3D_FUNCTIONS_H
