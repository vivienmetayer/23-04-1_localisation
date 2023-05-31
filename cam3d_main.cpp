#include <iostream>
#include <fstream>

#include "Cam3d_functions.h"
#include "Cam3d.h"

#include "ArenaApi.h"
#include "opencv2/opencv.hpp"

#define TAB1 "  "
#define TAB2 "    "
#define IMAGE_TIMEOUT 2000
#define PIXEL_FORMAT BGR8

void AcquireImageAndCreateHeatMapColoring(Arena::IDevice* pDevice)
{
    GenApi::INodeMap* pNodeMap = pDevice->GetNodeMap();

    // validate if Scan3dCoordinateSelector node exists. If not - probaly not
    // Helios camera used running the example
    GenApi::CEnumerationPtr checkpCoordSelector = pNodeMap->GetNode("Scan3dCoordinateSelector");
    if (!checkpCoordSelector)
    {
        std::cout << TAB1 << "Scan3dCoordinateSelector node is not found. Please make sure that Helios device is used for the example.\n";
        return;
    }

    // validate if Scan3dCoordinateOffset node exists. If not - probaly Helios
    // has an old firmware
    GenApi::CFloatPtr checkpCoord = pNodeMap->GetNode("Scan3dCoordinateOffset");
    if (!checkpCoord)
    {
        std::cout << TAB1 << "Scan3dCoordinateOffset node is not found. Please update Helios firmware.\n";
        return;
    }

    // check if Helios2 camera used for the example
    bool isHelios2 = false;
    GenICam::gcstring deviceModelName = Arena::GetNodeValue<GenICam::gcstring>(pDevice->GetNodeMap(), "DeviceModelName");
    std::string deviceModelName_tmp = deviceModelName.c_str();
    if (deviceModelName_tmp.rfind("HLT", 0) == 0 || deviceModelName_tmp.rfind("HTP", 0) == 0)
    {
        isHelios2 = true;
    }


    // get node values that will be changed in order to return their values at
    // the end of the example
    GenICam::gcstring pixelFormatInitial = Arena::GetNodeValue<GenICam::gcstring>(pNodeMap, "PixelFormat");
    GenICam::gcstring operatingModeInitial = Arena::GetNodeValue<GenICam::gcstring>(pNodeMap, "Scan3dOperatingMode");

    // set pixel format
    std::cout << TAB1 << "Set Coord3D_ABCY16 to pixel format\n";

    Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "PixelFormat", "Coord3D_ABCY16");

    // set operating mode distance
    if (isHelios2)
    {
        std::cout << TAB1 << "Set 3D operating mode to Distance3000mm\n";
        Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "Scan3dOperatingMode", "Distance3000mmSingleFreq");
    }
    else
    {
        std::cout << TAB1 << "Set 3D operating mode to Distance1500mm\n";
        Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "Scan3dOperatingMode", "Distance1500mm");
    }

    // get the z coordinate scale in order to convert z values to mm
    std::cout << TAB1 << "Get z coordinate scale\n\n";

    Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "Scan3dCoordinateSelector", "CoordinateC");

    // getting scale as float by casting since SetPly() will expect it passed as
    // float
    float scale = static_cast<float>(Arena::GetNodeValue<double>(pNodeMap, "Scan3dCoordinateScale"));

    // enable stream auto negotiate packet size
    Arena::SetNodeValue<bool>(pDevice->GetTLStreamNodeMap(), "StreamAutoNegotiatePacketSize", true);

    // enable stream packet resend
    Arena::SetNodeValue<bool>(pDevice->GetTLStreamNodeMap(), "StreamPacketResendEnable", true);

    // retrieve image
    std::cout << TAB2 << "Acquire image\n";

    pDevice->StartStream();
    Arena::IImage* pImage = pDevice->GetImage(IMAGE_TIMEOUT);

    // prepare info from input buffer
    size_t width = pImage->GetWidth();
    size_t height = pImage->GetHeight();
    size_t size = width * height;
    size_t srcBpp = pImage->GetBitsPerPixel();
    size_t srcPixelSize = srcBpp / 8; // divide by the number of bits in a byte
    const uint8_t* pInput = pImage->GetData();

    // prepare memory output buffer
    size_t dstBpp = Arena::GetBitsPerPixel(PIXEL_FORMAT);
    size_t dstPixelSize = dstBpp / 8;				  // divide by the number of bits in a byte
    size_t dstDataSize = width * height * dstBpp / 8; // divide by the number of bits in a byte
    auto* pOutput = new uint8_t[dstDataSize];
    memset(pOutput, 0, dstDataSize);

    // Prepare coloring buffer for ply image
    //    Saving ply with color takes RGB coloring compared to the BGR coloring
    //    the jpg image uses, therefore we need a separate buffer for this data.
    auto* pColoring = new uint8_t[dstDataSize];
    memset(pColoring, 0, dstDataSize);
    uint8_t* pColor = pColoring;

    // manually convert to BGR image

    const uint8_t* pIn = pInput;
    uint8_t* pOut = pOutput;

    const double RGBmin = 0;
    const double RGBmax = 255;

    double redColorBorder;
    double yellowColorBorder;
    double greenColorBorder;
    double cyanColorBorder;
    double blueColorBorder;

    if (isHelios2)
    {
        redColorBorder = 0;      // = 0 // start
        yellowColorBorder = 750; // = Scan3dOperatingMode / 4 // 1-st boarder
        greenColorBorder = 1500; // = (Scan3dOperatingMode / 4) * 2 // 2-nd boarder
        cyanColorBorder = 2250;  // = (Scan3dOperatingMode / 4) * 3 // 3-rd boarder
        blueColorBorder = 3000;  //  = Scan3dOperatingMode  // finish - maximum distance
    }
    else
    {
        redColorBorder = 0;
        yellowColorBorder = 375;
        greenColorBorder = 750;
        cyanColorBorder = 1125;
        blueColorBorder = 1500;
    }

    // iterate through each pixel and assign a color to it according to a
    // distance
    for (size_t i = 0; i < size; i++)
    {
        // Isolate the z data
        //    The first channel is the x coordinate, second channel is the y
        //    coordinate, the third channel is the z coordinate (which is what we
        //    will use to determine the coloring) and the fourth channel is
        //    intensity.
        int16_t z = *reinterpret_cast<const int16_t*>((pIn + 4));

        // Convert z to millimeters
        //    The z data converts at a specified ratio to mm, so by multiplying
        //    it by the Scan3dCoordinateScale for CoordinateC, we are able to
        //    convert it to mm and can then compare it to the maximum distance of
        //    1500mm (in this case 3000mm for Helios2).
        z = int16_t(double(z) * scale);

        double coordinateColorBlue;
        double coordinateColorGreen;
        double coordinateColorRed;

        // colors between red and yellow
        if ((z >= redColorBorder) && (z <= yellowColorBorder))
        {
            double yellowColorPercentage = z / yellowColorBorder;

            coordinateColorBlue = RGBmin;
            coordinateColorGreen = RGBmax * yellowColorPercentage;
            coordinateColorRed = RGBmax;
        }

            // colors between yellow and green
        else if ((z > yellowColorBorder) && (z <= greenColorBorder))
        {
            double greenColorPercentage = (z - yellowColorBorder) / yellowColorBorder;

            coordinateColorBlue = RGBmin;
            coordinateColorGreen = RGBmax;
            coordinateColorRed = RGBmax - RGBmax * greenColorPercentage;
        }

            // colors between green and cyan
        else if ((z > greenColorBorder) && (z <= cyanColorBorder))
        {
            double cyanColorPercentage = (z - greenColorBorder) / yellowColorBorder;

            coordinateColorBlue = RGBmax * cyanColorPercentage;
            coordinateColorGreen = RGBmax;
            coordinateColorRed = RGBmin;
        }

            // colors between cyan and blue
        else if ((z > cyanColorBorder) && (z <= blueColorBorder))
        {
            double blueColorPercentage = (z - cyanColorBorder) / yellowColorBorder;

            coordinateColorBlue = RGBmax;
            coordinateColorGreen = RGBmax - RGBmax * blueColorPercentage;
            coordinateColorRed = RGBmin;
        }
        else
        {
            coordinateColorBlue = RGBmin;
            coordinateColorGreen = RGBmin;
            coordinateColorRed = RGBmin;
        }

        // set pixel format values and move to next pixel
        *pOut = static_cast<int8_t>(coordinateColorBlue);
        *(pOut + 1) = static_cast<int8_t>(coordinateColorGreen);
        *(pOut + 2) = static_cast<int8_t>(coordinateColorRed);

        pIn += srcPixelSize;
        pOut += dstPixelSize;

        // set RGB pixel coloring for ply
        *pColor = static_cast<int8_t>(coordinateColorRed);
        *(pColor + 1) = static_cast<int8_t>(coordinateColorGreen);
        *(pColor + 2) = static_cast<int8_t>(coordinateColorBlue);
        pColor += dstPixelSize;
    }

    // create jpg image from buffer and save
    std::cout << TAB2 << "Create BGR heatmap using z data from 3D image\n";

    Arena::IImage* pCreate = Arena::ImageFactory::Create(pOutput, dstDataSize, width, height, PIXEL_FORMAT);
    std::cout << pCreate->GetBitsPerPixel() << "\n";
    std::cout << pCreate->GetOffsetX() << "\n";
    std::cout << pCreate->GetPaddingX() << "\n";
    std::cout << pCreate->GetSizeOfBuffer() << "\n";
    std::cout << pCreate->GetWidth() << "\n";
    std::cout << pCreate->GetSizeFilled() << "\n";
    std::cout << pCreate->GetPixelFormat() << "\n";
    cv::Mat image((int)pImage->GetHeight(), (int)pImage->GetWidth(), CV_8UC3, (void*)pOutput);
    cv::imwrite("test.jpg", image);
//    Save::ImageParams jpgParams(width, height, dstBpp);
//    Save::ImageWriter jpgWriter(jpgParams, JPG_FILE_NAME);
//    jpgWriter << pCreate->GetData();
//
//    std::cout << TAB2 << "Save heatmap image as jpg to " << jpgWriter.GetLastFileName() << "\n";

//    // prepare image parameters and writer for ply
//    Save::ImageParams plyParams(
//            pImage->GetWidth(),
//            pImage->GetHeight(),
//            pImage->GetBitsPerPixel());
//
//    Save::ImageWriter plyWriter(
//            plyParams,
//            PLY_FILE_NAME);
//
//    // set default parameters for SetPly()
//    bool filterPoints = true;		  // default
//    bool isSignedPixelFormat = false; // default
//    float offsetA = 0.0f;			  // default
//    float offsetB = 0.0f;			  // default
//    float offsetC = 0.0f;			  // default
//
//    // set the output file format of the image writer to .ply
//    plyWriter.SetPly(".ply", filterPoints, isSignedPixelFormat, scale, offsetA, offsetB, offsetC);
//
//    // save image
//    plyWriter.Save(pImage->GetData(), pColoring, true);
//
//    std::cout << TAB2 << "Save 3D image as ply to " << plyWriter.GetLastFileName() << "\n\n";

    // clean up
    Arena::ImageFactory::Destroy(pCreate);
    pOutput = nullptr;
    delete[] pOutput;
    pDevice->RequeueBuffer(pImage);
    pDevice->StopStream();

    // return nodes to their initial values
    Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "Scan3dOperatingMode", operatingModeInitial);
    Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "PixelFormat", pixelFormatInitial);
    std::cout << TAB1 << "Nodes were set back to initial values\n";
}

int testHelios() {
    std::cout << "Cpp_Acquisition_" << std::endl;

    try {
        Arena::ISystem *pSystem = Arena::OpenSystem();
        Arena::InterfaceInfo interfaceInfo;
        pSystem->UpdateDevices(100);
        std::vector<Arena::DeviceInfo> deviceInfos = pSystem->GetDevices();

        std::cout << "\nCameras detected: " << deviceInfos.size() << std::endl;

        if (deviceInfos.empty()) {
            std::cout << "\nNo camera connected" << std::endl;
            return 0;
        }

        for (auto & deviceInfo : deviceInfos)
        {
            std::cout << "IP: " << deviceInfo.IpAddressStr() << std::endl;
            std::cout << "IP: " << deviceInfo.IpAddress() << std::endl;
            std::cout << deviceInfo.ModelName() << std::endl;
        }

        Arena::IDevice *pDevice = pSystem->CreateDevice(deviceInfos[0]);

        AcquireImageAndCreateHeatMapColoring(pDevice);

        pSystem->DestroyDevice(pDevice);
        Arena::CloseSystem(pSystem);
    }
    catch (GenICam::GenericException &ge) {
        std::cout << "\nGenICam exception thrown: " << ge.what() << "\n";
        return -1;
    }
    catch (std::exception &ex) {
        std::cout << "\nStandard exception thrown: " << ex.what() << "\n";
        return -1;
    }
    catch (...) {
        std::cout << "\nUnexpected exception thrown\n";
        return -1;
    }

    return 0;
}

int testHeliosDLLInterface() {
    // Create arena system
    auto arenaSystemPtrInt = (uint64_t)CreateArenaSystem();

    // Get devices and find helios index
    int numDevices = getNumDevices(reinterpret_cast<Arena::ISystem *>(arenaSystemPtrInt));
    int deviceIndex = -1;
    for (int i = 0; i < numDevices; ++i) {
        char ipAddress[128];
        char modelName[128];
        getDeviceIPAddress(reinterpret_cast<Arena::ISystem *>(arenaSystemPtrInt), i, ipAddress);
        std::string deviceIPAddress = ipAddress;
        std::cout << "IP: " << deviceIPAddress << std::endl;
        getDeviceModelName(reinterpret_cast<Arena::ISystem *>(arenaSystemPtrInt), i, modelName);
        std::string modelNameStr = modelName;
        std::cout << modelNameStr << std::endl;

        if (modelNameStr.starts_with("HLT")) {
            std::cout << "Found Helios" << std::endl;
            deviceIndex = i;
            break;
        }
    }

    if (deviceIndex == -1) {
        std::cout << "No Helios found" << std::endl;
        return -1;
    }

    // Create device
    uint64_t ipAddress = 169 << 24 | 254 << 16 | 0u << 8 | 41u;
    uint64_t subnetMask = 0xFFFF0000;
    uint64_t defaultGateway = 0;
    Cam3d *cam3d = reinterpret_cast<Cam3d*>(createCam(reinterpret_cast<Arena::ISystem *>(arenaSystemPtrInt), deviceIndex));
//    Cam3d *cam3d = createCam3dForcedIP(reinterpret_cast<Arena::ISystem *>(arenaSystemPtrInt),
//                                       deviceIndex, ipAddress, subnetMask, defaultGateway);

    // set nodes)
    setNodeStr(cam3d, "PixelFormat", "Coord3D_ABCY16");
    setNodeStr(cam3d, "Scan3dOperatingMode", "Distance5000mmMultiFreq");
//    setNode(cam3d, "Scan3dOperatingMode", "Distance3000mmSingleFreq");
    setNodeInt(cam3d, "Scan3dConfidenceThresholdMin", (int64_t)500);
    setNodeBool(cam3d, "Scan3dConfidenceThresholdEnable", false);
    setNodeStr(cam3d, "Scan3dCoordinateSelector", "CoordinateC");

    // start stream
    startStream(cam3d);

    // get data
    std::vector<double> pointsData(3 * 640 * 480);
    std::vector<uint16_t> luminanceData(640 * 480);
    int numPoints;
    for (int i = 0; i < 5; ++i) {
        numPoints = getData(cam3d, pointsData.data(), luminanceData.data());
        std::cout << "Points: " << numPoints << std::endl;
    }
    pointsData.resize(numPoints * 3);


    // save data as xyz file
    std::ofstream myfile;
    myfile.open ("points.xyz");
    for (int i = 0; i < pointsData.size(); i += 3) {
        myfile << pointsData[i] << " " << pointsData[i+1] << " " << pointsData[i+2] << "\n";
    }

    // stop stream
    stopStream(cam3d);

    // delete device
    destroyCam(cam3d);

    // Destroy arena system
    DestroyArenaSystem(reinterpret_cast<Arena::ISystem *>(arenaSystemPtrInt));
    return 0;
}

int testTritonDLLInterface() {
    // Create arena system
    auto arenaSystemPtrInt = (uint64_t)CreateArenaSystem();

    // Get devices and find helios index
    int numDevices = getNumDevices(reinterpret_cast<Arena::ISystem *>(arenaSystemPtrInt));
    int deviceIndex = -1;
    for (int i = 0; i < numDevices; ++i) {
        char ipAddress[128];
        char modelName[128];
        getDeviceIPAddress(reinterpret_cast<Arena::ISystem *>(arenaSystemPtrInt), i, ipAddress);
        std::string deviceIPAddress = ipAddress;
        std::cout << "IP: " << deviceIPAddress << std::endl;
        getDeviceModelName(reinterpret_cast<Arena::ISystem *>(arenaSystemPtrInt), i, modelName);
        std::string modelNameStr = modelName;
        std::cout << modelNameStr << std::endl;

        if (modelNameStr.starts_with("TRI")) {
            std::cout << "Found Triton" << std::endl;
            deviceIndex = i;
            break;
        }
    }

    if (deviceIndex == -1) {
        std::cout << "No Triton found" << std::endl;
        return -1;
    }

    // Create device
    uint64_t ipAddress = 169 << 24 | 254 << 16 | 0u << 8 | 41u;
    uint64_t subnetMask = 0xFFFF0000;
    uint64_t defaultGateway = 0;
    Cam2d *cam2d = reinterpret_cast<Cam2d*>(createCam(reinterpret_cast<Arena::ISystem *>(arenaSystemPtrInt), deviceIndex));
//    Cam3d *cam3d = createCam3dForcedIP(reinterpret_cast<Arena::ISystem *>(arenaSystemPtrInt),
//                                       deviceIndex, ipAddress, subnetMask, defaultGateway);

    // set nodes)
//    setNodeStr(cam3d, "PixelFormat", "Coord3D_ABCY16");
//    setNodeStr(cam3d, "Scan3dOperatingMode", "Distance5000mmMultiFreq");
////    setNode(cam3d, "Scan3dOperatingMode", "Distance3000mmSingleFreq");
//    setNodeInt(cam3d, "Scan3dConfidenceThresholdMin", (int64_t)500);
//    setNodeBool(cam3d, "Scan3dConfidenceThresholdEnable", false);
//    setNodeStr(cam3d, "Scan3dCoordinateSelector", "CoordinateC");

    // show widrh and height
    int64_t width, height;
    getNodeInt(cam2d, "Width", &width);
    getNodeInt(cam2d, "Height", &height);
    std::cout << "Width: " << width << std::endl;
    std::cout << "Height: " << height << std::endl;

    // start stream
    startStream(cam2d);

    // get data
    cv::Mat image(height, width, CV_8UC4);
    getImage(cam2d, image.data, image.cols, image.rows, image.step);

    // show image
    cv::imshow("image", image);
    cv::waitKey(0);

    // stop stream
    stopStream(cam2d);

    // delete device
    destroyCam(cam2d);

    // Destroy arena system
    DestroyArenaSystem(reinterpret_cast<Arena::ISystem *>(arenaSystemPtrInt));
    return 0;
}

int main() {
    try {
        testTritonDLLInterface();
    }
    catch (GenICam::GenericException &ge) {
        std::cout << "\nGenICam exception thrown: " << ge.what() << "\n";
        return -1;
    }
    catch (std::exception &ex) {
        std::cout << "\nStandard exception thrown: " << ex.what() << "\n";
        return -1;
    }
    catch (...) {
        std::cout << "\nUnexpected exception thrown\n";
        return -1;
    }
    return 0;
}