#ifndef INC_23_04_1_LOCALISATION_TRIANGULATION_H
#define INC_23_04_1_LOCALISATION_TRIANGULATION_H

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include "TriangulationEngine.h"

#define DLL_EXPORT extern "C" __declspec(dllexport)

DLL_EXPORT int findBoardCorners(Protection *protection, unsigned char *imagePtr, int width, int height, int lineWidth,
                                int boardWidth, int boardHeight, float squareLength, float markerLength, int dictionaryId,
                                double *corners, double *objectPoints, int *ids, bool drawMarkers);

//DLL_EXPORT void calibrate(const double* corners, const double* corners3D, int numCorners,
//                          int width, int height, const char* calib_filename);

DLL_EXPORT int calibrateByCalculus(Protection *protection, const double *corners, const double *corners3D, int *ids, int numCorners, int width,
                                    int height, int dict, double *cameraMatrixValues, double *distCoeffs,
                                    int boardWidth, int boardHeight, double squareLength, double markerLength,
                                    const char *calib_filename);

DLL_EXPORT int readCalibrationImage(Protection *protection, const char* calib_image_path, float* map2D);

DLL_EXPORT double calibrateCamera(Protection *protection, double *corners, int *ids, const int *markersPerFrame, int numFrames,
                       int boardWidth, int boardHeight, float squareLength, float markerLength, int dict, double *cameraMatrix, double *distCoeffs);

DLL_EXPORT int detectMarkers(Protection *protection, unsigned char *imagePtr, int width, int height, int lineWidth, int dict,
                             double *corners, int *ids, int *numMarkers, int maxMarkers, bool drawMarkers);

//DLL_EXPORT void undistort(unsigned char *imagePtr, int width, int height, int lineWidth, double *cameraMatrix,
//               double *distCoeffs);

DLL_EXPORT int createUndistortMap(Protection *protection, double *cameraMatrix, double *distCoeffs, int width, int height,
                                   float *mapDataX, float *mapDataY);

//DLL_EXPORT void remap(unsigned char *imagePtr, int width, int height, int lineWidth, float *mapDataX, float *mapDataY);

// TriangulationEngine functions
DLL_EXPORT TriangulationEngine* createTriangulationEngine(Protection *protection);
DLL_EXPORT int TE_initUndistortMaps(TriangulationEngine *engine, Protection *protection, double *cameraMatrix, double *distCoeffs, int width, int height);
DLL_EXPORT int TE_setExtractionParameters(TriangulationEngine *engine, Protection *protection, int threshold, bool firstSignal, int minLineWidth, int orientation);
DLL_EXPORT int TE_setImage(TriangulationEngine *engine, Protection *protection, unsigned char *imagePtr, int width, int height, int lineWidth);
DLL_EXPORT int TE_extractLaserLine(TriangulationEngine *engine, Protection *protection);
DLL_EXPORT int TE_remapImage(TriangulationEngine *engine, Protection *protection);
//DLL_EXPORT void TE_remapLine(TriangulationEngine *engine);
DLL_EXPORT int TE_getLine(TriangulationEngine *engine, Protection *protection, double *line, int *lineWidths, int *size);
DLL_EXPORT void deleteTriangulationEngine(TriangulationEngine *engine);
DLL_EXPORT int TE_calibrateByCalculus(TriangulationEngine *engine, Protection *protection,
                                      const double *corners, const double *corners3D, int *ids, int numCorners,
                                      int width, int height, int dict, double *cameraMatrixValues,
                                      double *distCoeffs, int boardWidth, int boardHeight, double squareLength,
                                      double markerLength, const char *calib_filename);
DLL_EXPORT int TE_readCalibrationImage(TriangulationEngine *engine, Protection *protection, const char *fileName);

#endif //INC_23_04_1_LOCALISATION_TRIANGULATION_H
