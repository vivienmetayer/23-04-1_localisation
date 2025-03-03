//
// Created by vivien on 27/12/2024.
//

#include "TriangulationEngine.h"

void TriangulationEngine::initUndistortMaps(double *cameraMatrix, double *distCoeffs, int width, int height) {
    cv::Mat cameraMatrixMat(3, 3, CV_64F, cameraMatrix);
    cv::Mat distCoeffsMat(1, 5, CV_64F, distCoeffs);
    cv::Mat R = cv::Mat::eye(3, 3, CV_64F);
    cv::initUndistortRectifyMap(cameraMatrixMat, distCoeffsMat, R, cameraMatrixMat,
                                cv::Size(width, height), CV_32FC1, _mapX, _mapY);
}

void TriangulationEngine::setExtractionParameters(const int threshold, const bool firstSignal, const int minLineWidth) {
    _threshold = threshold;
    _firstSignal = firstSignal;
    _minLineWidth = minLineWidth;
}

void TriangulationEngine::setImage(unsigned char *imagePtr, const int width, const int height, const int lineWidth) {
    _image = cv::Mat(cv::Size(width, height), CV_8UC1, imagePtr, lineWidth);
}

void TriangulationEngine::extractLaserLine() {
    _line.clear();
    _line.reserve(_image.rows);

    // for each column, find laser line
    for (int i = 0; i < _image.cols; ++i) {
        cv::Mat column = _image.col(i);
        cv::Point2f laserPoint;
        int lineWidth;
        // read pixels from column, threshold and find line
        for (int j = 0; j < _image.rows; ++j) {
            if (column.at<uchar>(j) > _threshold) {
                int sum = column.at<uchar>(j);
                int positionWeight = j * column.at<uchar>(j);
                lineWidth = 0;

                // search for signal end
                int k = j;
                while (k < _image.rows && column.at<uchar>(k) > _threshold) {
                    k++;
                    sum += column.at<uchar>(k);
                    positionWeight += k * column.at<uchar>(k);
                    lineWidth++;
                }

                // Check if the detected line width is greater than or equal to _minLineWidth
                if (lineWidth >= _minLineWidth) {
                    float position = static_cast<float>(positionWeight) / static_cast<float>(sum);
                    laserPoint = cv::Point2f(static_cast<float>(i), position);
                    if (_firstSignal) break;
                }
            }
        }
        _line.push_back(laserPoint);
        _lineWidths.push_back(lineWidth);
    }
}

void TriangulationEngine::remapImage() {
    cv::Mat imageUndistorted;
    cv::remap(_image, imageUndistorted, _mapX, _mapY, cv::INTER_LINEAR);
    imageUndistorted.copyTo(_image);
}

//void TriangulationEngine::remapLine() {
//    for (auto &point : _line) {
//        // Interpolate the point using the maps
//        float x = point.x;
//        float y = point.y;
//
//        // Bilinear interpolation to get the remapped point
//        int x0 = static_cast<int>(x);
//        int y0 = static_cast<int>(y);
//        int x1 = x0 + 1;
//        int y1 = y0 + 1;
//
//        float a = x - x0;
//        float b = y - y0;
//
//        float newX = (1 - a) * (1 - b) * _mapX.at<float>(y0, x0) +
//                     a * (1 - b) * _mapX.at<float>(y0, x1) +
//                     (1 - a) * b * _mapX.at<float>(y1, x0) +
//                     a * b * _mapX.at<float>(y1, x1);
//
//        float newY = (1 - a) * (1 - b) * _mapY.at<float>(y0, x0) +
//                     a * (1 - b) * _mapY.at<float>(y0, x1) +
//                     (1 - a) * b * _mapY.at<float>(y1, x0) +
//                     a * b * _mapY.at<float>(y1, x1);
//
//        point.x = newX;
//        point.y = newY;
//    }
//}

void TriangulationEngine::getLine(double *line, int *lineWidths, int *size) const {
    for (int i = 0; i < _line.size(); ++i) {
        line[2 * i] = _line[i].x;
        line[2 * i + 1] = _line[i].y;
        lineWidths[i] = _lineWidths[i];
    }
    *size = static_cast<int>(_line.size());
}
