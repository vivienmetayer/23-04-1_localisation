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
                             6, 4, 40, 31, cv::aruco::DICT_4X4_250,
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
                             boardWidth, boardHeight, 17, 13, cv::aruco::DICT_4X4_250,
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
    double squareLength = 20;
    double markerLength = 15;
    std::vector<double> cornersArray(2 * boardWidth * boardHeight);
    std::vector<double> objectPointsArray(3 * boardWidth * boardHeight);
    std::vector<int> ids(boardWidth * boardHeight);
    int n = findBoardCorners(imageGray.data, imageGray.cols, imageGray.rows, (int) imageGray.step,
                             boardWidth, boardHeight, 17, 13, cv::aruco::DICT_4X4_250,
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
                        cv::aruco::DICT_4X4_250, cameraMatrix, distCoeffs, boardWidth, boardHeight, squareLength, markerLength,
                        R"(D:\Travail\Affaires\ARDPI\triangulation_2\dev\python\TriangulationTest\calib.exr)");

}

void testRemap() {
    std::vector<double> camMatrix;
    std::vector<double> distCoeffs;

    camMatrix.emplace_back(4590.0);
    camMatrix.emplace_back(0.0);
    camMatrix.emplace_back(2267);
    camMatrix.emplace_back(0.0);
    camMatrix.emplace_back(4590.0);
    camMatrix.emplace_back(2267);
    camMatrix.emplace_back(0.0);
    camMatrix.emplace_back(0.0);
    camMatrix.emplace_back(1.0);

    distCoeffs.emplace_back(-0.146);
    distCoeffs.emplace_back(0.132);
    distCoeffs.emplace_back(-0.000294);
    distCoeffs.emplace_back(-0.000213);
    distCoeffs.emplace_back(-0.0943);

    std::vector<float> mapX(4512 * 4512 * 2);
    std::vector<float> mapY(4512 * 4512 * 2);
    createUndistortMap(camMatrix.data(), distCoeffs.data(), 4512, 4512, mapX.data(), mapY.data());

    cv::Mat image = cv::imread(R"(D:\Work\Clients\ARDPI\23-04-1_localisation\images\20241022093602.png)");

    cv::namedWindow("original", cv::WINDOW_NORMAL);
    cv::imshow("original", image);

    cv::Mat imageUndistorted;
    remap(image.data, image.cols, image.rows, (int) image.step, mapX.data(), mapY.data());

    cv::namedWindow("undistorted", cv::WINDOW_NORMAL);
    cv::imshow("undistorted", image);
    cv::waitKey(0);
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
    detectMarkers(imageGray.data, imageGray.cols, imageGray.rows, (int) imageGray.step,  cv::aruco::DICT_4X4_250,
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
