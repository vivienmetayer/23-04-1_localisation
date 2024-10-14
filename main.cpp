#include "triangulation.h"

int testFindBoardCorners() {
    cv::Mat image = cv::imread(R"(E:\images\stereo\cam3d.png)");
    cv::Mat imageGray;
    cv::cvtColor(image, imageGray, cv::COLOR_BGR2GRAY);

    // find corners
    std::vector<double> cornersArray(2 * 11 * 11);
    std::vector<double> objectPointsArray(3 * 11 * 11);
    std::vector<int> ids(11 * 11);
    int n = findBoardCorners(imageGray.data, imageGray.cols, imageGray.rows, (int) imageGray.step,
                             6, 4, 40, 31,
                             cornersArray.data(), objectPointsArray.data(), ids.data(), true);

    // show image
    cv::namedWindow("corners", cv::WINDOW_NORMAL);
    cv::imshow("corners", imageGray);
    cv::waitKey(0);

    return n;
}

void testCalibration() {
    // load image
    cv::Mat image = cv::imread(
        R"(D:\Travail\Affaires\ARDPI\triangulation_2\data\Triangulation\blender_test\board_view.png)");
    cv::Mat imageGray;
    cv::cvtColor(image, imageGray, cv::COLOR_BGR2GRAY);

    // find corners
    int boardWidth = 14;
    int boardHeight = 9;
    std::vector<double> cornersArray(2 * boardWidth * boardHeight);
    std::vector<double> objectPointsArray(3 * boardWidth * boardHeight);
    std::vector<int> ids(boardWidth * boardHeight);
    int n = findBoardCorners(imageGray.data, imageGray.cols, imageGray.rows, (int) imageGray.step,
                             boardWidth, boardHeight, 17, 13,
                             cornersArray.data(), objectPointsArray.data(), ids.data(), true);
    std::cout << "Found " << n << " corners" << std::endl;
    ids.resize(n);

    // turn into vectors of points
    std::vector<cv::Point2f> corners;
    std::vector<cv::Point3f> objectPoints;
    for (int i = 0; i < n; ++i) {
        corners.emplace_back(cornersArray[2 * i], cornersArray[2 * i + 1]);
        objectPoints.emplace_back(objectPointsArray[3 * i], objectPointsArray[3 * i + 1], objectPointsArray[3 * i + 2]);
    }

    // draw corners
    //    cv::aruco::drawDetectedCornersCharuco(image, corners, ids);
    //    cv::namedWindow("corners", cv::WINDOW_NORMAL);
    //    cv::imshow("corners", image);
    //    cv::waitKey(0);

    // calibrate
    calibrate(cornersArray.data(), objectPointsArray.data(),
              n, imageGray.cols, imageGray.rows,
              R"(D:\Travail\Affaires\ARDPI\triangulation_2\dev\python\TriangulationTest\calib.exr)");
}

void testCalibrationByCalculus() {
    // load image
    cv::Mat image = cv::imread(
        R"(D:\Travail\Affaires\ARDPI\triangulation_2\data\Triangulation\blender_test\board_view.png)");
    cv::Mat imageGray;
    cv::cvtColor(image, imageGray, cv::COLOR_BGR2GRAY);

    // find board corners
    int boardWidth = 14;
    int boardHeight = 9;
    std::vector<double> cornersArray(2 * boardWidth * boardHeight);
    std::vector<double> objectPointsArray(3 * boardWidth * boardHeight);
    std::vector<int> ids(boardWidth * boardHeight);
    int n = findBoardCorners(imageGray.data, imageGray.cols, imageGray.rows, (int) imageGray.step,
                             boardWidth, boardHeight, 17, 13,
                             cornersArray.data(), objectPointsArray.data(), ids.data(), true);

    // calibrate by calculus
    double cameraMatrix[9];
    double distCoeffs[5];
    cameraMatrix[0] = 426.0;
    cameraMatrix[4] = 426.0;
    cameraMatrix[2] = image.size().width / 2.0;
    cameraMatrix[5] = image.size().height / 2.0;
    cameraMatrix[8] = 1.0;
    calibrateByCalculus(cornersArray.data(), objectPointsArray.data(), ids.data(), n, imageGray.cols, imageGray.rows,
                        cameraMatrix, distCoeffs, boardWidth, boardHeight,
                        R"(D:\Travail\Affaires\ARDPI\triangulation_2\dev\python\TriangulationTest\calib.exr)");

}

void testfindMmarkers() {
    // load test image
    cv::Mat image = cv::imread(
        R"(C:\Travail\Clients\ARDPI\23-02-1 Controle de position emetteur recepteur\dev\dodecaruco\images\calibration\cam1\image0.jpg)");
    cv::Mat imageGray;
    cv::cvtColor(image, imageGray, cv::COLOR_BGR2GRAY);
    std::cout << "image loaded" << std::endl;

    // find markers
    int maxMarkers = 10;
    int numMarkers;
    std::vector<double> cornersArray(2 * 4 * maxMarkers);
    std::vector<int> ids(maxMarkers);
    detectMarkers(imageGray.data, imageGray.cols, imageGray.rows, (int) imageGray.step,
                  cornersArray.data(), ids.data(), &numMarkers, maxMarkers, true);
    std::cout << "Found " << numMarkers << " markers" << std::endl;
    ids.resize(numMarkers);
    cornersArray.resize(2 * 4 * numMarkers);

    // draw markers
    std::vector<std::vector<cv::Point2f> > corners;
    for (int i = 0; i < numMarkers; ++i) {
        std::vector<cv::Point2f> markerCorners;
        for (int j = 0; j < 4; ++j) {
            markerCorners.emplace_back(cornersArray[2 * (4 * i + j)], cornersArray[2 * (4 * i + j) + 1]);
        }
        corners.push_back(markerCorners);
    }
    cv::aruco::drawDetectedMarkers(image, corners, ids);

    // show image
    cv::namedWindow("markers", cv::WINDOW_NORMAL);
    cv::imshow("markers", image);
    cv::waitKey(0);
}

void printBoard() {
    // create board
    int boardWidth = 14;
    int boardHeight = 9;
    float squareLength = 17;
    float markerLength = 13;
    cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);
    cv::Size boardSize(boardWidth, boardHeight);
    cv::aruco::CharucoBoard board(boardSize, squareLength, markerLength, dictionary);

    // print board
    cv::Mat boardImage;
    board.generateImage(cv::Size(1000, 1000), boardImage, 1, 1);
    cv::imwrite(R"(D:\Travail\Affaires\ARDPI\triangulation_2\data\Triangulation\board.png)", boardImage);
}

int main() {
    testCalibrationByCalculus();
    return 0;
}
