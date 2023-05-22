#ifndef INC_23_04_1_LOCALISATION_CAM3D_FUNCTIONS_H
#define INC_23_04_1_LOCALISATION_CAM3D_FUNCTIONS_H

#include "Cam3d.h"
#include "ArenaApi.h"

#define DLL_EXPORT extern "C" __declspec(dllexport)

// Arena System
DLL_EXPORT Arena::ISystem* CreateArenaSystem();
DLL_EXPORT void DestroyArenaSystem(Arena::ISystem *system);
DLL_EXPORT int getNumDevices(Arena::ISystem *system);
DLL_EXPORT char* getDeviceIPAddress(Arena::ISystem *system, int deviceIndex);

// Cam3d
DLL_EXPORT Cam3d* createCam3d(int a);


#endif //INC_23_04_1_LOCALISATION_CAM3D_FUNCTIONS_H
