#ifndef INC_23_04_1_LOCALISATION_CAM3D_FUNCTIONS_H
#define INC_23_04_1_LOCALISATION_CAM3D_FUNCTIONS_H

#include "Cam3d.h"
#include "ArenaApi.h"
#include "Cam2d.h"

#define DLL_EXPORT __declspec(dllexport)
#define DLL_EXPORT_C extern "C" __declspec(dllexport)

// Arena System
DLL_EXPORT_C Arena::ISystem* CreateArenaSystem();
DLL_EXPORT_C void DestroyArenaSystem(Arena::ISystem *system);
DLL_EXPORT_C int getNumDevices(Arena::ISystem *system);
DLL_EXPORT_C void getDeviceIPAddress(Arena::ISystem *system, int deviceIndex, char *ipAddressOut);
DLL_EXPORT_C void getDeviceModelName(Arena::ISystem *system, int deviceIndex, char *modelNameOut);

// Cam3d
DLL_EXPORT_C ArenaCam* createCam(Arena::ISystem *system, int deviceIndex);
DLL_EXPORT_C ArenaCam* createCamForcedIP(Arena::ISystem *system, int deviceIndex,
                                           uint64_t forcedIp, uint64_t subnetMask, uint64_t gateway);
DLL_EXPORT_C void destroyCam(ArenaCam *arenaCam);
DLL_EXPORT_C void setNodeStr(ArenaCam *cam, const char* nodeName, const char* nodeValue);
DLL_EXPORT_C void setNodeInt(ArenaCam *cam, const char* nodeName, int64_t nodeValue);
DLL_EXPORT_C void setNodeBool(ArenaCam *cam, const char* nodeName, bool nodeValue);
DLL_EXPORT_C void getNodeStr(ArenaCam *cam, const char* nodeName, char* nodeValue);
DLL_EXPORT_C void getNodeInt(ArenaCam *cam, const char* nodeName, int64_t *nodeValue);
DLL_EXPORT_C void getNodeBool(ArenaCam *cam, const char* nodeName, bool *nodeValue);
DLL_EXPORT_C void startStream(ArenaCam *cam);
DLL_EXPORT_C void stopStream(ArenaCam *cam);
DLL_EXPORT_C int getData(Cam3d *cam3d, double* points, uint64_t timeout = 1000);
DLL_EXPORT_C int getImage(Cam2d *cam2d, unsigned char* imagePtr, int width, int height, int stride, uint64_t timeout = 1000);


#endif //INC_23_04_1_LOCALISATION_CAM3D_FUNCTIONS_H
