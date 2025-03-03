#include "triangulation.h"

double distance_squared_2d(const cv::Point2f &p1, const cv::Point2f &p2) {
    return (p1.x - p2.x) * (p1.x - p2.x) + 3 * (p1.y - p2.y) * (p1.y - p2.y);
}

double distance_squared_3d(const cv::Point3f &p1, const cv::Point3f &p2) {
    return (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z);
}

double scalarProduct(const cv::Point2f &p1, const cv::Point2f &p2) {
    return p1.x * p2.x + p1.y * p2.y;
}

double norm(const cv::Point2f &p) {
    return std::sqrt(p.x * p.x + p.y * p.y);
}

bool Aligned(const std::vector<cv::Point3f> &points) {
    cv::Point3f AB = points[1] - points[0];
    cv::Point3f AC = points[2] - points[0];
    AB = (1 / norm(AB)) * AB;
    AC = (1 / norm(AC)) * AC;
    return abs(AB.x * AC.x + AB.y * AC.y) > 0.75;
}

std::vector<double> getBarycentricCoordinates(const cv::Point2f &p, const std::vector<cv::Point2f> &points) {
    double area = (points[1].y - points[2].y) * (points[0].x - points[2].x) +
                  (points[2].x - points[1].x) * (points[0].y - points[2].y);

    double w0 = ((points[1].y - points[2].y) * (p.x - points[2].x) +
                 (points[2].x - points[1].x) * (p.y - points[2].y)) / area;

    double w1 = ((points[2].y - points[0].y) * (p.x - points[2].x) +
                 (points[0].x - points[2].x) * (p.y - points[2].y)) / area;

    double w2 = 1 - w0 - w1;

    return {w0, w1, w2};
}

cv::Point3f applyBarycentricCoords(const std::vector<double> &coords, const std::vector<cv::Point3f> &points) {
    //    cv::Point3f result{ 0,0,0 };
    //    double sum = coords[0] + coords[1] + coords[2];
    //    result += coords[0] / sum * (points[0]);
    //    result += coords[1] / sum * (points[1]);
    //    result += coords[2] / sum * (points[2]);
    //    return result;

    cv::Point3f result = points[0];
    double sum = coords[0] + coords[1] + coords[2];
    result += coords[1] / sum * (points[1] - points[0]);
    result += coords[2] / sum * (points[2] - points[0]);
    return result;
}

int findBoardCorners(unsigned char *imagePtr, int width, int height, int lineWidth,
                     int boardWidth, int boardHeight, float squareLength, float markerLength, int dictionaryId,
                     double *corners, double *objectPoints, int *ids, bool drawMarkers) {
    // get image from LabVIEW pointer
    cv::Mat image(cv::Size(width, height), CV_8UC1, imagePtr, lineWidth);

    // create board
    cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(dictionaryId);
    cv::Size boardSize(boardWidth, boardHeight);
    cv::aruco::CharucoBoard board(boardSize, squareLength, markerLength, dictionary);

    // find corners
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f> > markerCorners;
    std::vector<int> charucoIds;
    std::vector<cv::Point2f> charucoCorners;
    cv::aruco::CharucoDetector charucoDetector(board);
    cv::aruco::CharucoParameters charucoParams;
    charucoParams.minMarkers = 1;
    charucoDetector.setCharucoParameters(charucoParams);
    charucoDetector.detectBoard(image, charucoCorners, charucoIds, markerCorners, markerIds);

    // export corners
    for (int i = 0; i < charucoCorners.size(); i++) {
        corners[2 * i] = charucoCorners[i].x;
        corners[2 * i + 1] = charucoCorners[i].y;
    }

    // export object points
    for (int i = 0; i < charucoIds.size(); i++) {
        objectPoints[3 * i] = board.getChessboardCorners()[charucoIds[i]].x;
        objectPoints[3 * i + 1] = board.getChessboardCorners()[charucoIds[i]].y;
        objectPoints[3 * i + 2] = 0;
    }

    // export ids
    for (int i = 0; i < charucoIds.size(); i++) {
        ids[i] = charucoIds[i];
    }

    // draw markers
    if (drawMarkers)
        cv::aruco::drawDetectedCornersCharuco(image, charucoCorners, charucoIds);

    return static_cast<int>(charucoCorners.size());
}

bool isPointInTriangle(const cv::Point2f &p, const std::vector<cv::Point2f> &triangle) {
    auto sign = [](const cv::Point2f p1, const cv::Point2f p2, const cv::Point2f p3) {
        return (p1.x - p3.x) * (p2.y - p3.y) - (p2.x - p3.x) * (p1.y - p3.y);
    };

    const bool b1 = sign(p, triangle[0], triangle[1]) < 0.0f;
    const bool b2 = sign(p, triangle[1], triangle[2]) < 0.0f;
    const bool b3 = sign(p, triangle[2], triangle[0]) < 0.0f;

    return ((b1 == b2) && (b2 == b3));
}

bool searchClosestPoints(cv::Point2f point,
                         const std::vector<cv::Point2f> &corners,
                         const std::vector<cv::Point3f> &objectPoints,
                         std::vector<int> &indices) {
    int size = static_cast<int>(corners.size());
    if (size < 3) return false;
    indices = {};
    std::vector<std::pair<double, int> > distances_indices(size);

    //store distance / indices pairs calculated from point and corners
    for (int i = 0; i < size; ++i) {
        distances_indices[i] = std::pair<double, int>(distance_squared_2d(point, corners[i]), i);
    }
    // sort by distance
    std::ranges::sort(distances_indices,
                      [](const std::pair<double, int> &d1, const std::pair<double, int> &d2) {
                          return d1.first < d2.first;
                      });

    // get first three points
    indices.push_back(distances_indices[0].second);
    indices.push_back(distances_indices[1].second);
    indices.push_back(distances_indices[2].second);

    // check further points if those 3 are aligned
    int i = 2;
    std::vector<cv::Point3f> baryPoints(3);
    baryPoints[0] = objectPoints[indices[0]];
    baryPoints[1] = objectPoints[indices[1]];
    baryPoints[2] = objectPoints[indices[2]];

    while (Aligned(baryPoints)) {
        if (i++ == size) return false;
        indices.pop_back();
        indices.push_back(distances_indices[i].second);
        baryPoints.pop_back();
        baryPoints.push_back(objectPoints[indices[2]]);
    }

    return true;
}

void calibrate(const double *corners, const double *corners3D, int numCorners, int width, int height,
               const char *calib_filename) {
    std::vector<cv::Point2f> m_corners(numCorners);
    for (int i = 0; i < numCorners; ++i) {
        m_corners[i] = cv::Point2f{static_cast<float>(corners[2 * i]), static_cast<float>(corners[2 * i + 1])};
    }
    std::vector<cv::Point3f> m_corners3D(numCorners);
    for (int i = 0; i < numCorners; ++i) {
        m_corners3D[i] = cv::Point3f{
            static_cast<float>(corners3D[3 * i]),
            static_cast<float>(corners3D[3 * i + 1]),
            static_cast<float>(corners3D[3 * i + 2])
        };
    }

    cv::Mat calib_image(height, width, CV_32FC3);
    for (int i = 0; i < height; ++i) {
        for (int j = 0; j < width; ++j) {
            cv::Point2f p{static_cast<float>(j), static_cast<float>(i)};
            std::vector<int> indices;
            searchClosestPoints(p, m_corners, m_corners3D, indices);

            std::vector<cv::Point2f> points{m_corners[indices[0]], m_corners[indices[1]], m_corners[indices[2]]};
            std::vector<double> barycentricCoords = getBarycentricCoordinates(p, points);

            std::vector<cv::Point3f> points3D{
                m_corners3D[indices[0]], m_corners3D[indices[1]], m_corners3D[indices[2]]
            };
            cv::Point3f result = applyBarycentricCoords(barycentricCoords, points3D);

            calib_image.at<cv::Vec3f>(i * width + j) = cv::Vec3f(result.x, result.y, result.z);
        }
    }
    cv::imwrite(calib_filename, calib_image);
}

void calibrateByCalculus(const double *corners, const double *corners3D, int *ids, int numCorners, int width,
                         int height, int dict, double *cameraMatrixValues, double *distCoeffs,
                         int boardWidth, int boardHeight, double squareLength, double markerLength,
                         const char *calib_filename) {
    // get cam matrix and dist coeffs
    cv::Mat cameraMatrix(3, 3, CV_64F, cameraMatrixValues);
    cv::Mat distCoeffsMat(1, 5, CV_64F, distCoeffs);

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
    cv::aruco::estimatePoseCharucoBoard(allCorners, allIds, board, cameraMatrix, distCoeffsMat, rvec, tvec);

    // for each pixel in the image, compute the 3D position by casting a ray from the camera to the board plane
    cv::Mat calib_image(height, width, CV_32FC3);
    for (int i = 0; i < height; ++i) {
        for (int j = 0; j < width; ++j) {
            cv::Point2f p{static_cast<float>(j), static_cast<float>(i)};
            cv::Mat ray(3, 1, CV_64F);

            // Compute ray in camera system
            ray.at<double>(0) = (p.x - cameraMatrix.at<double>(0, 2)) / cameraMatrix.at<double>(0, 0);
            ray.at<double>(1) = (p.y - cameraMatrix.at<double>(1, 2)) / cameraMatrix.at<double>(1, 1);
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

void readCalibrationImage(const char *calib_image_path, float *map2D) {
    cv::Mat image = cv::imread(calib_image_path, cv::IMREAD_UNCHANGED);

    for (int i = 0; i < image.rows; ++i) {
        for (int j = 0; j < image.cols; ++j) {
            cv::Vec3f p3D = image.at<cv::Vec3f>(i, j);
            map2D[(i * image.cols + j) * 2] = p3D[0];
            map2D[(i * image.cols + j) * 2 + 1] = p3D[1];
        }
    }
}

double calibrateCamera(double *corners, int *ids, const int *markersPerFrame, int numFrames,
                       int boardWidth, int boardHeight, float squareLength, float markerLength, int dict, double *cameraMatrix, double *distCoeffs) {
    // create board
    cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(dict);
    cv::Size boardSize(boardWidth, boardHeight);
    cv::Ptr<cv::aruco::CharucoBoard> board = cv::makePtr<cv::aruco::CharucoBoard>(boardSize, squareLength, markerLength, dictionary);

    // create vectors
    std::vector<std::vector<cv::Point2f> > allCorners;
    std::vector<std::vector<int> > allIds;
    std::vector<cv::Point3f> allObjectPoints;

    // fill vectors
    int offset = 0;
    for (int i = 0; i < numFrames; i++) {
        std::vector<int> frameIds;
        std::vector<cv::Point2f> frameCorners;
        std::vector<cv::Point3f> frameObjectPoints;
        for (int j = 0; j < markersPerFrame[i]; j++) {
            frameCorners.emplace_back(corners[2 * (j + offset)],
                                      corners[2 * (j + offset) + 1]);
            frameIds.push_back(ids[j + offset]);
            allObjectPoints.push_back(board->getChessboardCorners()[ids[j + offset]]);
        }
        offset += markersPerFrame[i];
        allCorners.push_back(frameCorners);
        allIds.push_back(frameIds);
    }

    // calibrate camera
    cv::Mat cameraMatrixMat(3, 3, CV_64F, cameraMatrix);
    cv::Mat distCoeffsMat(1, 5, CV_64F, distCoeffs);
    double error = cv::aruco::calibrateCameraCharuco(
        allCorners, allIds, board,
        cv::Size(2592, 1942),
        cameraMatrixMat, distCoeffsMat);
    //    double error = cv::aruco::calibrateCameraAruco(
    //            allCorners, allIds, allMarkersPerFrame, board,
    //            cv::Size(2592, 1942), cameraMatrixMat, distCoeffsMat);

    // fill camera matrix
    for (int i = 0; i < 9; ++i) {
        cameraMatrix[i] = cameraMatrixMat.at<double>(i);
    }

    // fill distortion coefficients
    for (int i = 0; i < 5; ++i) {
        distCoeffs[i] = distCoeffsMat.at<double>(i);
    }

    return error;
}

void undistort(unsigned char *imagePtr, int width, int height, int lineWidth, double *cameraMatrix,
               double *distCoeffs) {
    cv::Mat image(height, width, CV_8UC1, imagePtr, lineWidth);
    cv::Mat imageUndistorted;
    cv::Mat cameraMatrixMat(3, 3, CV_64F, cameraMatrix);
    cv::Mat distCoeffsMat(1, 5, CV_64F, distCoeffs);
    cv::undistort(image, imageUndistorted, cameraMatrixMat, distCoeffsMat);
    imageUndistorted.copyTo(image);
}

void createUndistortMap(double *cameraMatrix, double *distCoeffs, int width, int height,
                        float *mapDataX, float *mapDataY) {
    auto t0 = std::chrono::high_resolution_clock::now();
    cv::Mat cameraMatrixMat(3, 3, CV_64F, cameraMatrix);
    cv::Mat distCoeffsMat(1, 5, CV_64F, distCoeffs);
    cv::Mat map1(height, width, CV_32FC1);
    cv::Mat map2(height, width, CV_32FC1);
    cv::Mat R = cv::Mat::eye(3, 3, CV_64F);
    cv::initUndistortRectifyMap(cameraMatrixMat, distCoeffsMat, R, cameraMatrixMat,
                                cv::Size(width, height), CV_32FC1, map1, map2);

    auto t1 = std::chrono::high_resolution_clock::now();
    std::cout << "Time to create undistort map: "
              << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count()
              << " milliseconds" << std::endl;

    for (int i = 0; i < height; ++i) {
        for (int j = 0; j < width; ++j) {
            mapDataX[i * width + j] = map1.at<float>(i, j);
            mapDataY[i * width + j] = map2.at<float>(i, j);
        }
    }

    auto t2 = std::chrono::high_resolution_clock::now();
    std::cout << "Time to copy undistort map: "
              << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count()
              << " milliseconds" << std::endl;
}

void remap(unsigned char *imagePtr, int width, int height, int lineWidth, float *mapDataX, float* mapDataY) {
    cv::Mat image(height, width, CV_8UC1, imagePtr, lineWidth);
    cv::Mat imageUndistorted;
    cv::Mat mapX(height, width, CV_32FC1, mapDataX);
    cv::Mat mapY(height, width, CV_32FC1, mapDataY);
    cv::remap(image, imageUndistorted, mapX, mapY, cv::INTER_LINEAR);
    imageUndistorted.copyTo(image);
}

int detectMarkers(unsigned char *imagePtr, int width, int height, int lineWidth, int dict,
                  double *corners, int *ids, int *numMarkers, int maxMarkers, bool drawMarkers) {
    cv::Mat image(height, width, CV_8UC1, imagePtr, lineWidth);
    cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(dict);

    cv::aruco::ArucoDetector detector(dictionary);
    std::vector<std::vector<cv::Point2f> > markerCorners;
    std::vector<int> markerIds;
    detector.detectMarkers(image, markerCorners, markerIds);

    // output markers positions and ids
    int size = std::min(maxMarkers, static_cast<int>(markerIds.size()));
    for (int i = 0; i < size; ++i) {
        ids[i] = markerIds[i];
        for (int j = 0; j < 4; ++j) {
            corners[2 * (i * 4 + j)] = markerCorners[i][j].x;
            corners[2 * (i * 4 + j) + 1] = markerCorners[i][j].y;
        }
    }
    *numMarkers = size;

    // draw markers
    if (drawMarkers) {
        std::vector<std::vector<cv::Point2f> > markersToDraw;
        std::vector<int> idsToDraw;
        markersToDraw.insert(markersToDraw.end(), markerCorners.begin(), markerCorners.begin() + size);
        idsToDraw.insert(idsToDraw.end(), markerIds.begin(), markerIds.begin() + size);
        cv::aruco::drawDetectedMarkers(image, markersToDraw, idsToDraw);
    }

    return 0;
}

TriangulationEngine* createTriangulationEngine() {
    auto* engine = new TriangulationEngine();
    return engine;
}

void deleteTriangulationEngine(TriangulationEngine *engine) {
    delete engine;
}

void TE_initUndistortMaps(TriangulationEngine *engine, double *cameraMatrix, double *distCoeffs, int width, int height) {
    engine->initUndistortMaps(cameraMatrix, distCoeffs, width, height);
}

void TE_setExtractionParameters(TriangulationEngine *engine, int threshold, bool firstSignal, int minLineWidth) {
    engine->setExtractionParameters(threshold, firstSignal, minLineWidth);
}

void TE_setImage(TriangulationEngine *engine, unsigned char *imagePtr, int width, int height, int lineWidth) {
    engine->setImage(imagePtr, width, height, lineWidth);
}

void TE_extractLaserLine(TriangulationEngine *engine) {
    engine->extractLaserLine();
}

void TE_remapImage(TriangulationEngine *engine) {
    engine->remapImage();
}

void TE_remapLine(TriangulationEngine *engine) {
    engine->remapLine();
}

void TE_getLine(TriangulationEngine *engine, double *line, int size) {
    engine->getLine(line, size);
}
