#include "triangulation.h"

double distance_squared_2d(cv::Point2f p1, cv::Point2f p2)
{
    return (p1.x - p2.x) * (p1.x - p2.x) + 3 * (p1.y - p2.y) * (p1.y - p2.y);
}

double distance_squared_3d(cv::Point3f p1, cv::Point3f p2)
{
    return (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z);
}

double scalarProduct(cv::Point2f p1, cv::Point2f p2)
{
    return p1.x * p2.x + p1.y * p2.y;
}

double norm(cv::Point2f p)
{
    return std::sqrt(p.x * p.x + p.y * p.y);
}

bool Aligned(std::vector<cv::Point3f>& points)
{
    cv::Point3f AB = points[1] - points[0];
    cv::Point3f AC = points[2] - points[0];
    AB = (1 / norm(AB)) * AB;
    AC = (1 / norm(AC)) * AC;
    return abs(AB.x * AC.x + AB.y * AC.y) > 0.5;
}

std::vector<double> getBarycentricCoordinates(cv::Point2f p, std::vector<cv::Point2f>& points)
{
    double xa = points[0].x;
    double xb = points[1].x;
    double xc = points[2].x;
    double xp = p.x;

    double ya = points[0].y;
    double yb = points[1].y;
    double yc = points[2].y;
    double yp = p.y;

    double v_num = yp - ya - (yb - ya) * (xp - xa) / (xb - xa);
    double v_den = yc - ya - (xc - xa) * (yb - ya) / (xb - xa);
    double v = v_num / v_den;

    double u = (xp - xa) / (xb - xa) - v * (xc - xa) / (xb - xa);
    return { 1 - u - v, u, v };
}

cv::Point3f applyBarycentricCoords(std::vector<double>& coords, std::vector<cv::Point3f>& points)
{
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
                     int boardWidth, int boardHeight, float squareLength, float markerLength,
                     double *corners, double *objectPoints, int *ids)
{
    // get image from LabVIEW pointer
    cv::Mat image(cv::Size(width, height), CV_8UC1, imagePtr, lineWidth);

    // create board
    cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100);
    cv::Size boardSize(boardWidth, boardHeight);
    cv::aruco::CharucoBoard board(boardSize, squareLength, markerLength, dictionary);

    // find corners
    std::vector<int> markerIds;
    std::vector <std::vector<cv::Point2f>> markerCorners;
    std::vector<int> charucoIds;
    std::vector <cv::Point2f> charucoCorners;
    cv::aruco::CharucoDetector charucoDetector(board);
    cv::aruco::CharucoParameters charucoParams;
    charucoParams.minMarkers = 1;
    charucoDetector.setCharucoParameters(charucoParams);
    charucoDetector.detectBoard(image, charucoCorners, charucoIds, markerCorners, markerIds);

    // export corners
    for (int i = 0; i < charucoCorners.size(); i++)
    {
        corners[2 * i] = charucoCorners[i].x;
        corners[2 * i + 1] = charucoCorners[i].y;
    }

    // export object points
    for (int i = 0; i < charucoIds.size(); i++)
    {
        objectPoints[3 * i] = board.getChessboardCorners()[charucoIds[i]].x;
        objectPoints[3 * i + 1] = board.getChessboardCorners()[charucoIds[i]].y;
        objectPoints[3 * i + 2] = 0;
    }

    // export ids
    for (int i = 0; i < charucoIds.size(); i++)
    {
        ids[i] = charucoIds[i];
    }

    return (int) charucoCorners.size();
}

bool searchClosestPoints(cv::Point2f point,
                         const std::vector<cv::Point2f>& corners,
                         const std::vector<cv::Point3f>& objectPoints,
                         std::vector<int>& indices)
{
    int size = (int)corners.size();
    if (size < 3) return false;
    indices = {};
    std::vector<std::pair<double, int>> distances_indices(size);

    //store distance / indices pairs calculated from point and corners
    for (int i = 0; i < size; ++i) {
        distances_indices[i] = std::pair<double, int>(distance_squared_2d(point, corners[i]), i);
    }
    // sort by distance
    std::sort(distances_indices.begin(), distances_indices.end(),
              [](std::pair<double, int> &d1, std::pair<double, int>& d2) {return d1.first < d2.first; });

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

    while(Aligned(baryPoints)) {
        if (i++ == size) return false;
        indices.pop_back();
        indices.push_back(distances_indices[i].second);
        baryPoints.pop_back();
        baryPoints.push_back(objectPoints[indices[2]]);
    }

    return true;
}

void calibrate(double* corners, double* corners3D, int numCorners, int width, int height, const char* calib_filename)
{
    std::vector<cv::Point2f> m_corners(numCorners);
    for (int i = 0; i < numCorners; ++i) {
        m_corners[i] = cv::Point2f{ (float) corners[2 * i], (float) corners[2 * i + 1] };
    }
    std::vector<cv::Point3f> m_corners3D(numCorners);
    for (int i = 0; i < numCorners; ++i) {
        m_corners3D[i] = cv::Point3f{ (float) corners3D[3 * i], (float) corners3D[3 * i + 1], (float) corners3D[3 * i + 2] };
    }

    cv::Mat calib_image(height, width, CV_32FC3);
    for (int i = 0; i < height; ++i) {
        for (int j = 0; j < width; ++j) {
            cv::Point2f p{ (float) j, (float) i };
            std::vector<int> indices;
            searchClosestPoints(p, m_corners, m_corners3D, indices);

            std::vector<cv::Point2f> points{ m_corners[indices[0]], m_corners[indices[1]], m_corners[indices[2]] };
            std::vector<double> barycentricCoords = getBarycentricCoordinates(p, points);

            std::vector<cv::Point3f> points3D{ m_corners3D[indices[0]], m_corners3D[indices[1]], m_corners3D[indices[2]] };
            cv::Point3f result = applyBarycentricCoords(barycentricCoords, points3D);

            calib_image.at<cv::Vec3f>(i * width + j) = cv::Vec3f(result.x, result.y, result.z);
        }
    }
    cv::imwrite(calib_filename, calib_image);
}

void readCalibrationImage(const char* calib_image_path, float* map2D)
{
    cv::Mat image = cv::imread(calib_image_path, cv::IMREAD_UNCHANGED);

    for (int i = 0; i < image.rows; ++i) {
        for (int j = 0; j < image.cols; ++j) {
            cv::Vec3f p3D = image.at<cv::Vec3f>(i, j);
            map2D[(i * image.cols + j) * 2] = p3D[0];
            map2D[(i * image.cols + j) * 2 + 1] = p3D[1];
        }
    }
}