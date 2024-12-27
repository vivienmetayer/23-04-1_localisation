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
        // read pixels from column, threshold and find line
        for (int j = 0; j < _image.rows; ++j) {
            if (column.at<uchar>(j) > _threshold) {
                // search for signal end
                int k = j;
                while (k < _image.rows && column.at<uchar>(k) > _threshold) {
                    k++;
                }
                laserPoint = cv::Point2f(i, (j + k) / 2);
                if (_firstSignal) break;
            }
        }
        _line.push_back(laserPoint);
    }
}

void TriangulationEngine::remapImage() {
    cv::Mat imageUndistorted;
    cv::remap(_image, imageUndistorted, _mapX, _mapY, cv::INTER_LINEAR);
    imageUndistorted.copyTo(_image);
}

void TriangulationEngine::remapLine() {
    for (auto &point : _line) {
        cv::Mat pointMat(1, 1, CV_32FC2, &point);
        cv::remap(pointMat, pointMat, _mapX, _mapY, cv::INTER_LINEAR);
    }
}

void TriangulationEngine::getLine(double *line, const int size) const {
    for (int i = 0; i < size; ++i) {
        line[2 * i] = _line[i].x;
        line[2 * i + 1] = _line[i].y;
    }
}
