#ifndef INC_23_04_1_LOCALISATION_TRIANGULATION_H
#define INC_23_04_1_LOCALISATION_TRIANGULATION_H

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include "TriangulationEngine.h"

#define DLL_EXPORT extern "C" __declspec(dllexport)

DLL_EXPORT int findBoardCorners(unsigned char *imagePtr, int width, int height, int lineWidth,
                                int boardWidth, int boardHeight, float squareLength, float markerLength, int dictionaryId,
                                double *corners, double *objectPoints, int *ids, bool drawMarkers);

DLL_EXPORT void calibrate(const double* corners, const double* corners3D, int numCorners,
                          int width, int height, const char* calib_filename);

DLL_EXPORT void calibrateByCalculus(const double *corners, const double *corners3D, int *ids, int numCorners, int width,
                                    int height, int dict, double *cameraMatrixValues, double *distCoeffs,
                                    int boardWidth, int boardHeight, double squareLength, double markerLength,
                                    const char *calib_filename);

DLL_EXPORT void readCalibrationImage(const char* calib_image_path, float* map2D);

DLL_EXPORT double calibrateCamera(double *corners, int *ids, const int *markersPerFrame, int numFrames,
                       int boardWidth, int boardHeight, float squareLength, float markerLength, int dict, double *cameraMatrix, double *distCoeffs);

DLL_EXPORT int detectMarkers(unsigned char *imagePtr, int width, int height, int lineWidth, int dict,
                             double *corners, int *ids, int *numMarkers, int maxMarkers, bool drawMarkers);

DLL_EXPORT void undistort(unsigned char *imagePtr, int width, int height, int lineWidth, double *cameraMatrix,
               double *distCoeffs);

DLL_EXPORT void createUndistortMap(double *cameraMatrix, double *distCoeffs, int width, int height,
                                   float *mapDataX, float *mapDataY);

DLL_EXPORT void remap(unsigned char *imagePtr, int width, int height, int lineWidth, float *mapDataX, float *mapDataY);

// TriangulationEngine functions
DLL_EXPORT void createTriangulationEngine(TriangulationEngine *engineOut);
DLL_EXPORT void TE_initUndistortMaps(TriangulationEngine *engine, double *cameraMatrix, double *distCoeffs, int width, int height);
DLL_EXPORT void TE_setExtractionParameters(TriangulationEngine *engine, int threshold, bool firstSignal, int minLineWidth);
DLL_EXPORT void TE_setImage(TriangulationEngine *engine, unsigned char *imagePtr, int width, int height, int lineWidth);
DLL_EXPORT void TE_extractLaserLine(TriangulationEngine *engine);
DLL_EXPORT void TE_remapImage(TriangulationEngine *engine);
DLL_EXPORT void TE_remapLine(TriangulationEngine *engine);
DLL_EXPORT void TE_getLine(TriangulationEngine *engine, double *line, int size);

#endif //INC_23_04_1_LOCALISATION_TRIANGULATION_H
