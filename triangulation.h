#ifndef INC_23_04_1_LOCALISATION_TRIANGULATION_H
#define INC_23_04_1_LOCALISATION_TRIANGULATION_H

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#define DLL_EXPORT extern "C" __declspec(dllexport)

DLL_EXPORT int findBoardCorners(unsigned char *imagePtr, int width, int height, int lineWidth,
                                int boardWidth, int boardHeight, float squareLength, float markerLength,
                                double *corners, double *objectPoints, int *ids);

DLL_EXPORT void calibrate(double* corners, double* corners3D, int numCorners,
                          int width, int height, const char* calib_filename);

#endif //INC_23_04_1_LOCALISATION_TRIANGULATION_H
