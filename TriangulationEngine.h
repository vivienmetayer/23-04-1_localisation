//
// Created by vivien on 27/12/2024.
//

#ifndef TRIANGULATIONENGINE_H
#define TRIANGULATIONENGINE_H

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

class TriangulationEngine {
public:
    void initUndistortMaps(double *cameraMatrix, double *distCoeffs, int width, int height);
    void setExtractionParameters(int threshold, bool firstSignal, int minLineWidth);
    void setImage(unsigned char *imagePtr, int width, int height, int lineWidth);
    void extractLaserLine();
    void remapImage();
//    void remapLine();
    void getLine(double *line, int *lineWidths, int *size) const;
private:
    cv::Mat _image;
    cv::Mat _mapX;
    cv::Mat _mapY;
    std::vector<cv::Point2f> _line;
    std::vector<int> _lineWidths;
    int _threshold {25};
    bool _firstSignal {true};
    int _minLineWidth {2};
};



#endif //TRIANGULATIONENGINE_H
