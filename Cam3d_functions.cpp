#include "Cam3d_functions.h"
#include <opencv2/aruco.hpp>

Arena::ISystem* CreateArenaSystem() {
    Arena::ISystem* system = Arena::OpenSystem();
    return system;
}

void DestroyArenaSystem(Arena::ISystem *system) {
    Arena::CloseSystem(system);
}

int getNumDevices(Arena::ISystem *system) {
    Arena::InterfaceInfo interfaceInfo;
    system->UpdateDevices(100);
    return (int)system->GetDevices().size();
}

void getDeviceIPAddress(Arena::ISystem *system, int deviceIndex, char *ipAddressOut) {
    Arena::InterfaceInfo interfaceInfo;
    system->UpdateDevices(100);
    std::vector<Arena::DeviceInfo> deviceInfos = system->GetDevices();
    std::string ipAddress = deviceInfos[deviceIndex].IpAddressStr().c_str();
    for (int i = 0; i < ipAddress.size(); i++) {
        ipAddressOut[i] = ipAddress[i];
    }
}

void getDeviceModelName(Arena::ISystem *system, int deviceIndex, char *modelNameOut) {
    Arena::InterfaceInfo interfaceInfo;
    system->UpdateDevices(100);
    std::vector<Arena::DeviceInfo> deviceInfos = system->GetDevices();
    std::string modelName = deviceInfos[deviceIndex].ModelName().c_str();
    for (int i = 0; i < modelName.size(); ++i) {
        modelNameOut[i] = modelName[i];
    }
}

ArenaCam* createCam(Arena::ISystem *system, int deviceIndex) {
    return new ArenaCam(system, deviceIndex);
}

ArenaCam* createCamForcedIP(Arena::ISystem *system, int deviceIndex,
                           uint64_t forcedIp, uint64_t subnetMask, uint64_t gateway) {
    return new ArenaCam(system, deviceIndex, forcedIp, subnetMask, gateway);
}

void destroyCam(ArenaCam *arenaCam) {
    delete arenaCam;
}

void setNodeStr(ArenaCam *cam, const char* nodeName, const char* nodeValue) {
    cam->setNodeStr(nodeName, nodeValue);
}

void setNodeInt(ArenaCam *cam, const char* nodeName, int64_t nodeValue) {
    cam->setNodeInt(nodeName, nodeValue);
}

void setNodeBool(ArenaCam *cam, const char* nodeName, bool nodeValue) {
    cam->setNodeBool(nodeName, nodeValue);
}

void getNodeStr(ArenaCam *cam, const char* nodeName, char* nodeValue) {
    std::string str;
    cam->getNodeStr(nodeName, str);
    for (int i = 0; i < str.size(); i++) {
        nodeValue[i] = str[i];
    }
}

void getNodeInt(ArenaCam *cam, const char* nodeName, int64_t *nodeValue) {
    cam->getNodeInt(nodeName, nodeValue);
}

void getNodeBool(ArenaCam *cam, const char* nodeName, bool *nodeValue) {
    cam->getNodeBool(nodeName, nodeValue);
}

void startStream(ArenaCam *cam) {
    cam->startStream();
}

void stopStream(ArenaCam *cam) {
    cam->stopStream();
}

int getData(Cam3d *cam3d, double* points, uint16_t *luminance, uint64_t timeout) {
    std::vector<cv::Point3d> cvPoints;
    std::vector<std::vector<uint16_t>> lum;
    int numPoints = cam3d->getData(cvPoints, lum, timeout);

    // copy points to points array
    for (int i = 0; i < numPoints; i++) {
        points[i * 3] = cvPoints[i].x;
        points[i * 3 + 1] = cvPoints[i].y;
        points[i * 3 + 2] = cvPoints[i].z;
    }

    // copy luminance to lum array
    for (int i = 0; i < lum.size(); i++) {
        for (int j = 0; j < lum[i].size(); j++) {
            luminance[i * lum[i].size() + j] = lum[i][j];
        }
    }
    return numPoints;
}

int getImage(Cam2d *cam2d, unsigned char* imagePtr, int width, int height, int stride, uint64_t timeout) {
    cv::Mat image(height, width, CV_8UC4, imagePtr, stride);
    cam2d->getImage(image, timeout);
    return 0;
}

double calibrateCamera(double *corners, int *ids, const int *markersPerFrame, int numFrames,
                       int boardWidth, int boardHeight, double checkerSize, double markerSize,
                       double *cameraMatrix, double *distCoeffs)
{
    // create board
    cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    cv::Size boardSize(boardWidth, boardHeight);
    cv::Ptr <cv::aruco::CharucoBoard> board = cv::makePtr<cv::aruco::CharucoBoard>(boardSize, checkerSize, markerSize, dictionary);

    // create vectors
    std::vector<std::vector<cv::Point2f>> allCorners;
    std::vector<std::vector<int>> allIds;
    std::vector <cv::Point3f> allObjectPoints;

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

int stereoMatch(double *points3dData, double *points2dData, int numPoints,
                double *rotationMatrix, double *translationVector) {
    // create vectors of points
    std::vector<cv::Point3d> points3d(numPoints);
    std::vector<cv::Point2d> points2d(numPoints);
    for (int i = 0; i < numPoints; i++) {
        points3d[i].x = points3dData[i * 3];
        points3d[i].y = points3dData[i * 3 + 1];
        points3d[i].z = points3dData[i * 3 + 2];
        points2d[i].x = points2dData[i * 2];
        points2d[i].y = points2dData[i * 2 + 1];
    }

    // solve
    cv::Mat rvec, tvec;
    cv::solvePnP(points3d, points2d, cv::Mat::eye(3, 3, CV_64F), cv::Mat::zeros(5, 1, CV_64F), rvec, tvec);

    // fill output
    cv::Mat rotationMatrixMat(3, 3, CV_64F, rotationMatrix);
    cv::Rodrigues(rvec, rotationMatrixMat);
    for (int i = 0; i < 3; ++i) {
        translationVector[i] = tvec.at<double>(i);
    }
    for (int i = 0; i < 9; ++i) {
        rotationMatrix[i] = rotationMatrixMat.at<double>(i);
    }
    return 0;
}