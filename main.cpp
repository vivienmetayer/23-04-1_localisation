#include <filesystem>

#include "triangulation.h"
#include "TriangulationProtectionDebug.h"

void generateImage() {
    int squaresX = 24;
    int squaresY = 14;
    float squareLength = 8;
    float markerLength = 6;

    cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_250);

    cv::Size boardSize(squaresX, squaresY);
    cv::aruco::CharucoBoard board(boardSize, squareLength, markerLength, dictionary);
    board.setLegacyPattern(true);

    cv::Mat boardImage;
    board.generateImage(cv::Size(2400, 1400), boardImage, 10, 1);

    cv::imshow("ChArUco Board", boardImage);
    cv::imwrite("D:\\Work\\Clients\\ARDPI\\23-04-1_localisation\\images\\20250505161709cam12\\charuco_board.png", boardImage);
    cv::waitKey(0);
}

int testFindBoardCorners() {
//    cv::Mat image = cv::imread(R"(C:\Users\Vivien\Documents\ShareX\Screenshots\2025-05\2025_05_20_chrome_JF.png)");
    cv::Mat image = cv::imread(R"(D:\Work\Clients\ARDPI\23-04-1_localisation\images\20250505161709cam12\20250505160801cam12.bmp)");
    cv::Mat imageGray;
    cv::cvtColor(image, imageGray, cv::COLOR_BGR2GRAY);
    //cv::GaussianBlur(imageGray, imageGray, cv::Size(5, 5), 0);

    // find corners
    std::vector<double> cornersArray(2 * 24 * 14);
    std::vector<double> objectPointsArray(3 * 24 * 14);
    std::vector<int> ids(24 * 14);
    TriangulationProtectionDebug *protection = createTriangulationProtection(0);
    int n = findBoardCorners(protection, imageGray.data, imageGray.cols, imageGray.rows, (int) imageGray.step,
                             24, 14, 8, 6, cv::aruco::DICT_5X5_250,
                             cornersArray.data(), objectPointsArray.data(), ids.data(), true,
                             5, 31, 71);

    // show image
    cv::namedWindow("corners", cv::WINDOW_NORMAL);
    cv::imshow("corners", imageGray);
    cv::waitKey(0);

    return n;
//    return 0;
}

void testCalibration() {
    double cameraMatrix[9];
    double distCoeffs[5];
    cameraMatrix[0] = 3238.29;
    cameraMatrix[4] = 3183.7;
    cameraMatrix[2] = 1708.29;
    cameraMatrix[5] = 1837.37;
    cameraMatrix[8] = 1.0;
    distCoeffs[0] = -0.292626;
    distCoeffs[1] = 0.301862;
    distCoeffs[2] = -0.0190655;
    distCoeffs[3] = -0.0146109;
    distCoeffs[4] = -0.117675;

    // load image
    cv::Mat image = cv::imread(R"(D:\Travail\Affaires\ARDPI\23-04-1 Systeme de localisation\data\Lower\20241022151758.png)");
    cv::Mat imageGray;
    cv::cvtColor(image, imageGray, cv::COLOR_BGR2GRAY);

    // find corners
    int boardWidth = 14;
    int boardHeight = 9;
    float squareLength = 20;
    float markerLength = 15;
    std::vector<double> cornersArray(2 * boardWidth * boardHeight);
    std::vector<double> objectPointsArray(3 * boardWidth * boardHeight);
    std::vector<int> ids(boardWidth * boardHeight);
//    int n = findBoardCorners(imageGray.data, imageGray.cols, imageGray.rows, (int) imageGray.step,
//                             boardWidth, boardHeight, squareLength, markerLength, cv::aruco::DICT_4X4_250,
//                             cornersArray.data(), objectPointsArray.data(), ids.data(), true);
//    std::cout << "Found " << n << " corners" << std::endl;
//    ids.resize(n);
//    cornersArray.resize(2 * n);
//    objectPointsArray.resize(3 * n);

    // calibrate
//    calibrate(cornersArray.data(), objectPointsArray.data(),
//              n, imageGray.cols, imageGray.rows,
//              R"(D:\Travail\Affaires\ARDPI\23-04-1 Systeme de localisation\data\calib.exr)");

    // compare distance between observed and calculated points
    cv::Mat imageCalib = cv::imread(R"(D:\Travail\Affaires\ARDPI\23-04-1 Systeme de localisation\data\calib.exr)", cv::IMREAD_UNCHANGED);

    // take first and last corner of original image
    cv::Point2f firstCorner = cv::Point2f(cornersArray[0], cornersArray[1]);
    cv::Point2f lastCorner = cv::Point2f(cornersArray[2 * 11], cornersArray[2 * 11 + 1]);
    cv::Point3f firstCornerCalib = imageCalib.at<cv::Vec3f>(firstCorner.y, firstCorner.x);
    cv::Point3f lastCornerCalib = imageCalib.at<cv::Vec3f>(lastCorner.y, lastCorner.x);
    double distance = cv::norm(firstCornerCalib - lastCornerCalib);
    std::cout << "Distance between first and last corner (original): " << distance << std::endl;
    std::cout << "first corner: " << firstCornerCalib << std::endl;
    std::cout << "last corner: " << lastCornerCalib << std::endl;

    // undistort image
//    undistort(imageGray.data, imageGray.cols, imageGray.rows, (int) imageGray.step, cameraMatrix, distCoeffs);

    // take first and last corner of undistorted image
//    n = findBoardCorners(imageGray.data, imageGray.cols, imageGray.rows, (int) imageGray.step,
//                         boardWidth, boardHeight, squareLength, markerLength, cv::aruco::DICT_4X4_250,
//                         cornersArray.data(), objectPointsArray.data(), ids.data(), true);
    firstCorner = cv::Point2f(cornersArray[0], cornersArray[1]);
    lastCorner = cv::Point2f(cornersArray[2 * 11], cornersArray[2 * 11 + 1]);
    firstCornerCalib = imageCalib.at<cv::Vec3f>(firstCorner.y, firstCorner.x);
    lastCornerCalib = imageCalib.at<cv::Vec3f>(lastCorner.y, lastCorner.x);
    distance = cv::norm(firstCornerCalib - lastCornerCalib);
    std::cout << "Distance between first and last corner (undistorted): " << distance << std::endl;
    std::cout << "first corner: " << firstCornerCalib << std::endl;
    std::cout << "last corner: " << lastCornerCalib << std::endl;
    cv::namedWindow("img", cv::WINDOW_NORMAL);
    cv::imshow("img", imageGray);
    cv::waitKey(0);
}

void testCalibrationByCalculus() {
    double cameraMatrix[9];
    double distCoeffs[5];
    cameraMatrix[0] = 3040.022781196563;
    cameraMatrix[4] = 3038.316007361436;
    cameraMatrix[2] = 2253.351931432469;
    cameraMatrix[5] = 2273.846344756578;
    cameraMatrix[8] = 1.0;
    distCoeffs[0] = -0.128096393794965;
    distCoeffs[1] = 0.04437890045144942;
    distCoeffs[2] = -0.0003225185663205268;
    distCoeffs[3] = -6.917872926871078e-05;
    distCoeffs[4] = -0.02;

    // load image
    cv::Mat image = cv::imread(R"(D:\Travail\Affaires\ARDPI\23-04-1 Systeme de localisation\data\Lower\20241022151758.png)");
    cv::Mat imageGray;
    cv::cvtColor(image, imageGray, cv::COLOR_BGR2GRAY);

    // find board corners
    int boardWidth = 14;
    int boardHeight = 9;
    float squareLength = 20;
    float markerLength = 15;
    std::vector<double> cornersArray(2 * boardWidth * boardHeight);
    std::vector<double> objectPointsArray(3 * boardWidth * boardHeight);
    std::vector<int> ids(boardWidth * boardHeight);
    // undistort(imageGray.data, imageGray.cols, imageGray.rows, (int) imageGray.step, cameraMatrix, distCoeffs);
//    int n = findBoardCorners(imageGray.data, imageGray.cols, imageGray.rows, (int) imageGray.step,
//                             boardWidth, boardHeight, squareLength, markerLength, cv::aruco::DICT_4X4_250,
//                             cornersArray.data(), objectPointsArray.data(), ids.data(), false);
//    ids.resize(n);
//    cornersArray.resize(2 * n);
//    objectPointsArray.resize(3 * n);

    // calibrate
//    calibrateByCalculus(cornersArray.data(), objectPointsArray.data(), ids.data(), n, imageGray.cols, imageGray.rows,
//                        cv::aruco::DICT_4X4_250, cameraMatrix, distCoeffs, boardWidth, boardHeight, squareLength, markerLength,
//                        R"(D:\Travail\Affaires\ARDPI\23-04-1 Systeme de localisation\data\calib.exr)");

    // compare distance between observed and calculated points
    cv::Mat imageCalib = cv::imread(R"(D:\Travail\Affaires\ARDPI\23-04-1 Systeme de localisation\data\calib.exr)", cv::IMREAD_UNCHANGED);

    // take first and last corner of original image
    cv::Point2f firstCorner = cv::Point2f(cornersArray[0], cornersArray[1]);
    cv::Point2f lastCorner = cv::Point2f(cornersArray[2 * 11], cornersArray[2 * 11 + 1]);
    cv::Point3f firstCornerCalib = imageCalib.at<cv::Vec3f>(firstCorner.y, firstCorner.x);
    cv::Point3f lastCornerCalib = imageCalib.at<cv::Vec3f>(lastCorner.y, lastCorner.x);
    double distance = cv::norm(firstCornerCalib - lastCornerCalib);
    std::cout << "Distance between first and last corner (original): " << distance << std::endl;
    std::cout << "first corner: " << firstCornerCalib << std::endl;
    std::cout << "last corner: " << lastCornerCalib << std::endl;

    // undistort image
//    undistort(imageGray.data, imageGray.cols, imageGray.rows, (int) imageGray.step, cameraMatrix, distCoeffs);

    // take first and last corner of undistorted image
//    n = findBoardCorners(imageGray.data, imageGray.cols, imageGray.rows, (int) imageGray.step,
//                         boardWidth, boardHeight, squareLength, markerLength, cv::aruco::DICT_4X4_250,
//                         cornersArray.data(), objectPointsArray.data(), ids.data(), true);
    firstCorner = cv::Point2f(cornersArray[0], cornersArray[1]);
    lastCorner = cv::Point2f(cornersArray[2 * 11], cornersArray[2 * 11 + 1]);
    firstCornerCalib = imageCalib.at<cv::Vec3f>(firstCorner.y, firstCorner.x);
    lastCornerCalib = imageCalib.at<cv::Vec3f>(lastCorner.y, lastCorner.x);
    distance = cv::norm(firstCornerCalib - lastCornerCalib);
    std::cout << "Distance between first and last corner (undistorted): " << distance << std::endl;
    std::cout << "first corner: " << firstCornerCalib << std::endl;
    std::cout << "last corner: " << lastCornerCalib << std::endl;
    cv::namedWindow("img", cv::WINDOW_NORMAL);
    cv::imshow("img", imageGray);
    cv::waitKey(0);
    cv::imwrite("C:/Users/vivie/Desktop/test.png", imageGray);
}

void testRemap() {
    double cameraMatrix[9];
    double distCoeffs[5];
    cameraMatrix[0] = 3040.022781196563;
    cameraMatrix[4] = 3038.316007361436;
    cameraMatrix[2] = 2253.351931432469;
    cameraMatrix[5] = 2273.846344756578;
    cameraMatrix[8] = 1.0;
    distCoeffs[0] = -0.128096393794965;
    distCoeffs[1] = 0.04437890045144942;
    distCoeffs[2] = -0.0003225185663205268;
    distCoeffs[3] = -6.917872926871078e-05;
    distCoeffs[4] = -0.02;

    std::vector<float> mapX(4512 * 4512, 0.0f);
    std::vector<float> mapY(4512 * 4512, 0.0f);
//    createUndistortMap(cameraMatrix, distCoeffs, 4512, 4512, mapX.data(), mapY.data());

    cv::Mat image = cv::imread(R"(D:\Travail\Affaires\ARDPI\23-04-1 Systeme de localisation\data\DataCalib\Lower\20241022151758.png)");
    cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);

    cv::namedWindow("original", cv::WINDOW_NORMAL);
    cv::imshow("original", image);

//    remap(image.data, image.cols, image.rows, (int) image.step, mapX.data(), mapY.data());

    cv::namedWindow("undistorted", cv::WINDOW_NORMAL);
    cv::imshow("undistorted", image);
    cv::waitKey(0);
}

void testFindMarkers() {
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
//    detectMarkers(imageGray.data, imageGray.cols, imageGray.rows, (int) imageGray.step,  cv::aruco::DICT_4X4_250,
//                  cornersArray.data(), ids.data(), &numMarkers, maxMarkers, true);
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
    float squareLength = 20;
    float markerLength = 15;
    cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100);
    cv::Size boardSize(boardWidth, boardHeight);
    cv::aruco::CharucoBoard board(boardSize, squareLength, markerLength, dictionary);

    // print board
    cv::Mat boardImage;
    board.generateImage(cv::Size(1000, 1000), boardImage, 0, 1);
    // cv::imwrite(R"(D:\Travail\Affaires\ARDPI\23-04-1 Systeme de localisation\data\board.png)", boardImage);
    cv::namedWindow("board", cv::WINDOW_NORMAL);
    cv::imshow("board", boardImage);
    cv::waitKey(0);
}

void calibrateCamera() {
    // load image from folder using filesystem
    std::string folder = R"(D:\Travail\Affaires\ARDPI\23-04-1 Systeme de localisation\data\Lower\)";
    cv::Size imageSize;
    std::vector<cv::Mat> images;
    for (const auto &entry : std::filesystem::directory_iterator(folder)) {
        if (!entry.path().string().ends_with(".png")) continue;
        cv::Mat image = cv::imread(entry.path().string());
        images.push_back(image);
        imageSize = image.size();
    }

    int boardWidth = 14;
    int boardHeight = 9;
    float squareLength = 20;
    float markerLength = 15;
    double cameraMatrix[9];
    double distCoeffs[5];

    // create vectors
    std::vector<std::vector<cv::Point2f> > allCorners;
    std::vector<std::vector<int> > allIds;
    std::vector<cv::Point3f> allObjectPoints;

    // fill vectors with board found in images
    cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);
    cv::Size boardSize(boardWidth, boardHeight);
    cv::Ptr<cv::aruco::CharucoBoard> board = cv::makePtr<cv::aruco::CharucoBoard>(boardSize, squareLength, markerLength, dictionary);
    for (auto &img : images) {
        cv::Mat imgGray;
        cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);
        std::vector<int> markerIds;
        std::vector<std::vector<cv::Point2f> > markerCorners;
        std::vector<int> charucoIds;
        std::vector<cv::Point2f> charucoCorners;
        cv::aruco::CharucoDetector charucoDetector(*board);
        cv::aruco::CharucoParameters charucoParams;
        charucoParams.minMarkers = 1;
        charucoDetector.setCharucoParameters(charucoParams);
        charucoDetector.detectBoard(img, charucoCorners, charucoIds, markerCorners, markerIds);
        allCorners.push_back(charucoCorners);
        allIds.push_back(charucoIds);
        for (int i = 0; i < charucoCorners.size(); i++) {
            allObjectPoints.push_back(board->getChessboardCorners()[charucoIds[i]]);
        }
    }

    // calibrate
    cv::Mat cameraMatrixMat(3, 3, CV_64F, cameraMatrix);
    cv::Mat distCoeffsMat(1, 5, CV_64F, distCoeffs);
    double error = 0;
    error = cv::aruco::calibrateCameraCharuco(
        allCorners, allIds, board,
        imageSize, cameraMatrixMat, distCoeffsMat);

    // print results
    std::cout << "Camera matrix: " << cameraMatrixMat << std::endl;
    std::cout << "Distortion coefficients: " << distCoeffsMat << std::endl;
    std::cout << "Error: " << error << std::endl;

    // undistort image
    cv::Mat imgUndistorted;
    cv::undistort(images[0], imgUndistorted, cameraMatrixMat, distCoeffsMat);
    cv::namedWindow("undistorted", cv::WINDOW_NORMAL);
    cv::imshow("undistorted", imgUndistorted);
    cv::waitKey(0);

    std::cout << "Calibration done" << std::endl;
    std::cout << "dist coeffs: " << distCoeffs[0] << " " << distCoeffs[1] << " " << distCoeffs[2] << " " << distCoeffs[3] << " " << distCoeffs[4] << std::endl;
    std::cout << "camera matrix: "
            << cameraMatrix[0] << " " << cameraMatrix[1] << " " << cameraMatrix[2] << " "
            << cameraMatrix[3] << " " << cameraMatrix[4] << " " << cameraMatrix[5] << " "
            << cameraMatrix[6] << " " << cameraMatrix[7] << " " << cameraMatrix[8] << std::endl;
}

int testTriangulationEngine() {
    double cameraMatrix[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    double distCoeffs[5];
    cameraMatrix[0] = 4538.86;
    cameraMatrix[4] = 4536.97;
    cameraMatrix[2] = 2270.74;
    cameraMatrix[5] = 2290.24;
    cameraMatrix[8] = 1.0;
    distCoeffs[0] = -0.168353;
    distCoeffs[1] = 0.154076;
    distCoeffs[2] = -0.000354333;
    distCoeffs[3] = -6.917872926871078e-05;
    distCoeffs[4] = -0.02;

    TriangulationEngine engine;
    engine.initUndistortMaps(cameraMatrix, distCoeffs, 4512, 4512);
    std::vector<float> map2D(4512*4512*2);
    engine.readCalibrationImage(R"(D:\Work\Clients\ARDPI\23-04-1_localisation\images\calib.exr)", map2D.data());
    cv::Mat image = cv::imread(R"(D:\Work\Clients\ARDPI\23-04-1_localisation\images\20250311105219cam9.bmp)");
    cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);
    engine.setImage(image.data, image.cols, image.rows, image.step);
    engine.setExtractionParameters(100, true, 5, orientation::VERTICAL);
    engine.extractLaserLine();
    std::vector<double> line(4512 * 2);
    std::vector<int> lineWidths(4512);
    int size;
    engine.getLine(line.data(), lineWidths.data(), &size);

    cv::Mat checkImage = cv::imread(R"(D:\Work\Clients\ARDPI\23-04-1_localisation\images\20250311105219cam9.bmp)");
    if (checkImage.type() == CV_8UC3) {
        for (int i = 0; i < 4512; ++i) {
            if (lineWidths[i] != -1) {
                cv::Point2i point(line[2*i], line[2*i+1]);
                checkImage.at<cv::Vec3b>(point) = cv::Vec3b(0, 0, 255);
            }
        }
    }
    cv::namedWindow("image", cv::WINDOW_NORMAL);
    cv::imshow("image", checkImage);
    cv::waitKey(0);
    return 0;
}

int main() {
    int n = testFindBoardCorners();
//    generateImage();
    return n;
}
