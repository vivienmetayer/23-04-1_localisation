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
    _cameraMatrix = cameraMatrixMat.clone();
    _distCoeffs = distCoeffsMat.clone();
}

void TriangulationEngine::setExtractionParameters(const int threshold, const bool firstSignal, const int minLineWidth, int scanOrientation) {
    _threshold = threshold;
    _firstSignal = firstSignal;
    _minLineWidth = minLineWidth;
    _orientation = static_cast<orientation>(scanOrientation);
}

void TriangulationEngine::setImage(unsigned char *imagePtr, const int width, const int height, const int lineWidth) {
    _image = cv::Mat(cv::Size(width, height), CV_8UC1, imagePtr, lineWidth);
}

void TriangulationEngine::extractLaserLine() {
    int numLines = _orientation == VERTICAL ? _image.rows : _image.cols;
    int lineLength = _orientation == VERTICAL ? _image.cols : _image.rows;
    _line.clear();
    _line.reserve(numLines);

    // for each line, find laser line
    for (int i = 0; i < numLines; ++i) {
        cv::Mat column = _orientation == VERTICAL ? _image.col(i) : _image.row(i);
        cv::Point2f laserPoint;
        int lineWidth = 0;
        // read pixels from column, threshold and find line
        for (int j = 0; j < lineLength; ++j) {
            if (column.at<uchar>(j) > _threshold) {
                int sum = column.at<uchar>(j);
                int positionWeight = j * column.at<uchar>(j);
                lineWidth = 0;

                // search for signal end
                int k = j;
                while (k < lineLength && column.at<uchar>(k) > _threshold) {
                    k++;
                    sum += column.at<uchar>(k);
                    positionWeight += k * column.at<uchar>(k);
                    lineWidth++;
                }

                // Check if the detected line width is greater than or equal to _minLineWidth
                if (lineWidth >= _minLineWidth) {
                    float position = static_cast<float>(positionWeight) / static_cast<float>(sum);
                    laserPoint = cv::Point2f(static_cast<float>(i), position);
                    if (_orientation == HORIZONTAL) {
                        laserPoint = cv::Point2f(position, static_cast<float>(i));
                    }
                    if (_firstSignal) break;
                    else j = k;
                }
            }
        }
        if (lineWidth >= _minLineWidth) {
            _line.push_back(laserPoint);
            _lineWidths.push_back(lineWidth);
        } else {
            _line.emplace_back(-1, -1);
            _lineWidths.push_back(-1);
        }
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

void TriangulationEngine::calibrateByCalculus(const double *corners, const double *corners3D, int *ids, int numCorners,
                                              int width, int height, int dict, int boardWidth, int boardHeight, double squareLength,
                                              double markerLength, const char *calib_filename) {
    // create board
    cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(dict);
    cv::Size boardSize(boardWidth, boardHeight);
    cv::Ptr<cv::aruco::CharucoBoard> board = cv::makePtr<cv::aruco::CharucoBoard>(boardSize, squareLength, markerLength, dictionary);

    // create vectors
    std::vector<cv::Point2f> allCorners;
    std::vector<int> allIds;
    std::vector<cv::Point3f> allObjectPoints;

    for (int i = 0; i < numCorners; i++) {
        allCorners.emplace_back(corners[2 * i], corners[2 * i + 1]);
        allIds.push_back(ids[i]);
        allObjectPoints.emplace_back(static_cast<float>(corners3D[3 * i]),
                                     static_cast<float>(corners3D[3 * i + 1]),
                                     static_cast<float>(corners3D[3 * i + 2]));
    }

    // compute board position
    cv::Mat rvec, tvec;
    cv::aruco::estimatePoseCharucoBoard(allCorners, allIds, board, _cameraMatrix, _distCoeffs, rvec, tvec);

    // for each pixel in the image, compute the 3D position by casting a ray from the camera to the board plane
    cv::Mat calib_image(height, width, CV_32FC3);
    for (int i = 0; i < height; ++i) {
        for (int j = 0; j < width; ++j) {
            cv::Point2f p{static_cast<float>(j), static_cast<float>(i)};
            cv::Mat ray(3, 1, CV_64F);

            // Compute ray in camera system
            ray.at<double>(0) = (p.x - _cameraMatrix.at<double>(0, 2)) / _cameraMatrix.at<double>(0, 0);
            ray.at<double>(1) = (p.y - _cameraMatrix.at<double>(1, 2)) / _cameraMatrix.at<double>(1, 1);
            ray.at<double>(2) = 1;

            // Apply inverse transform to get to world system
            cv::Mat R, R_inv;
            cv::Rodrigues(rvec, R);
            R_inv = R.t();
            cv::Mat t_inv = -R_inv * tvec;
            cv::Mat rayWorld = R_inv * ray;

            // define board plane Z = 0
            double scale = -t_inv.at<double>(2) / rayWorld.at<double>(2); // scale to intercept z=0 plane
            cv::Mat intersection = t_inv + scale * rayWorld;
            cv::Point3f result{
                    static_cast<float>(intersection.at<double>(0)),
                    static_cast<float>(intersection.at<double>(1)),
                    static_cast<float>(intersection.at<double>(2))
            };

            // Store position in image
            calib_image.at<cv::Vec3f>(i * width + j) = cv::Vec3f(result.x, result.y, result.z);
        }
    }
    cv::imwrite(calib_filename, calib_image);

}

void TriangulationEngine::readCalibrationImage(const char *calib_image_path, float *map2D) {
    _calibImage = cv::imread(calib_image_path, cv::IMREAD_UNCHANGED);

    for (int i = 0; i < _calibImage.rows; ++i) {
        for (int j = 0; j < _calibImage.cols; ++j) {
            cv::Vec3f p3D = _calibImage.at<cv::Vec3f>(i, j);
            map2D[(i * _calibImage.cols + j) * 2] = p3D[0];
            map2D[(i * _calibImage.cols + j) * 2 + 1] = p3D[1];
        }
    }
}

cv::Vec3f TriangulationEngine::getPosition(double x, double y) {
    if (_calibImage.empty()) return {0, 0, 0};

    // neighbor pixels
    int x1 = static_cast<int>(std::floor(x));
    int y1 = static_cast<int>(std::floor(y));
    int x2 = x1 + 1;
    int y2 = y1 + 1;

    // check image borders
    if (x1 < 0 || x2 >= _calibImage.cols || y1 < 0 || y2 >= _calibImage.rows) {
        return {0, 0, 0};
    }

    // get neighbor pixels values
    cv::Vec3f Q11 = _calibImage.at<cv::Vec3f>(y1, x1);
    cv::Vec3f Q21 = _calibImage.at<cv::Vec3f>(y1, x2);
    cv::Vec3f Q12 = _calibImage.at<cv::Vec3f>(y2, x1);
    cv::Vec3f Q22 = _calibImage.at<cv::Vec3f>(y2, x2);

    // compute interpolation weights
    double x2_x = x2 - x;
    double x_x1 = x - x1;
    double y2_y = y2 - y;
    double y_y1 = y - y1;

    // Interpolation
    cv::Vec3f interpolatedValue = Q11 * x2_x * y2_y +
                                  Q21 * x_x1 * y2_y +
                                  Q12 * x2_x * y_y1 +
                                  Q22 * x_x1 * y_y1;

    return interpolatedValue;
}
