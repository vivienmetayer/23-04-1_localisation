#include "triangulation.h"

int main()
{
    // load image
    cv::Mat image = cv::imread(R"(F:\Travail\Affaires\ARDPI\23-04-1 Systeme de localisation\data\board_view.png)");
    cv::Mat imageGray;
    cv::cvtColor(image, imageGray, cv::COLOR_BGR2GRAY);

    // find corners
    std::vector<double> cornersArray(2 * 11 * 11);
    std::vector<double> objectPointsArray(3 * 11 * 11);
    std::vector<int> ids(11 * 11);
    int n = findBoardCorners(imageGray.data, imageGray.cols, imageGray.rows, (int) imageGray.step,
                             11, 11, 17, 13,
                             cornersArray.data(), objectPointsArray.data(), ids.data());
    std::cout << "Found " << n << " corners" << std::endl;
    ids.resize(n);

    // turn into vectors of points
    std::vector<cv::Point2f> corners;
    std::vector<cv::Point3f> objectPoints;
    for (int i = 0; i < n; ++i)
    {
        corners.emplace_back(cornersArray[2 * i], cornersArray[2 * i + 1]);
        objectPoints.emplace_back(objectPointsArray[3 * i], objectPointsArray[3 * i + 1], objectPointsArray[3 * i + 2]);
    }

    // draw corners
    cv::aruco::drawDetectedCornersCharuco(image, corners, ids);
    cv::namedWindow("corners", cv::WINDOW_NORMAL);
    cv::imshow("corners", image);
    cv::waitKey(0);

    return 0;
}
