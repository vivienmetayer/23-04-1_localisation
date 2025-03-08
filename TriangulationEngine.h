//
// Created by vivien on 27/12/2024.
//

#ifndef TRIANGULATIONENGINE_H
#define TRIANGULATIONENGINE_H

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

enum orientation {
    HORIZONTAL = 0,
    VERTICAL
};

class TriangulationEngine {
public:
    void initUndistortMaps(double *cameraMatrix, double *distCoeffs, int width, int height);
    void setExtractionParameters(int threshold, bool firstSignal, int minLineWidth, int orientation);
    void setImage(unsigned char *imagePtr, int width, int height, int lineWidth);
    void extractLaserLine();
    void remapImage();
//    void remapLine();
    void getLine(double *line, int *lineWidths, int *size) const;
    void calibrateByCalculus(const double *corners, const double *corners3D, int *ids, int numCorners, int width,
                             int height, int dict, double *cameraMatrixValues, double *distCoeffs,
                             int boardWidth, int boardHeight, double squareLength, double markerLength,
                             const char *calib_filename);
    void readCalibrationImage(const char *calib_image_path);
    cv::Vec3f getPosition(double x, double y);
private:
    cv::Mat _image;
    cv::Mat _mapX;
    cv::Mat _mapY;
    cv::Mat _calibImage;
    std::vector<cv::Point2f> _line;
    std::vector<int> _lineWidths;
    int _threshold {25};
    bool _firstSignal {true};
    int _minLineWidth {2};
    orientation _orientation {VERTICAL};
};



#endif //TRIANGULATIONENGINE_H
