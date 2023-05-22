#ifndef INC_23_04_1_LOCALISATION_CAM3D_FUNCTIONS_H
#define INC_23_04_1_LOCALISATION_CAM3D_FUNCTIONS_H

#include "Cam3d.h"
#include "ArenaApi.h"

#define DLL_EXPORT __declspec(dllexport)
#define DLL_EXPORT_C extern "C" __declspec(dllexport)

// Arena System
DLL_EXPORT_C Arena::ISystem* CreateArenaSystem();
DLL_EXPORT_C void DestroyArenaSystem(Arena::ISystem *system);
DLL_EXPORT_C int getNumDevices(Arena::ISystem *system);
DLL_EXPORT std::string getDeviceIPAddress(Arena::ISystem *system, int deviceIndex);
DLL_EXPORT std::string getDeviceModelName(Arena::ISystem *system, int deviceIndex);

// Cam3d
DLL_EXPORT_C Cam3d* createCam3d(Arena::ISystem *system, int deviceIndex);
DLL_EXPORT_C void destroyCam3d(Cam3d *cam3d);
DLL_EXPORT void setNode(Cam3d *cam3d, std::string nodeName, std::string nodeValue);


#endif //INC_23_04_1_LOCALISATION_CAM3D_FUNCTIONS_H
