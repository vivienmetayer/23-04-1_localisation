#include "Cam3d.h"


#include "opencv2/opencv.hpp"
#include <fstream>

Cam3d::Cam3d(Arena::ISystem *system, int deviceIndex) : ArenaCam(system, deviceIndex) {

}

Cam3d::Cam3d(Arena::ISystem *system, int deviceIndex, uint64_t forcedIp, uint64_t subnetMask, uint64_t gateway)
        : ArenaCam(system, deviceIndex, forcedIp, subnetMask, gateway) {

}

Cam3d::~Cam3d() {

}

int Cam3d::getData(std::vector<cv::Point3d> &points, std::vector<std::vector<uint16_t>> &luminance, uint64_t timeout) {
    // grab
    Arena::IImage* pImage = _device->GetImage(timeout);
    size_t width = pImage->GetWidth();
    size_t height = pImage->GetHeight();
    size_t srcBpp = pImage->GetBitsPerPixel();
    size_t srcPixelSize = srcBpp / 8;

    // get scale
    GenApi::INodeMap* pNodeMap = _device->GetNodeMap();
    float scale = static_cast<float>(Arena::GetNodeValue<double>(pNodeMap, "Scan3dCoordinateScale"));

    // get data ptr
    const uint8_t* dataPtr = pImage->GetData();
    const uint8_t* pIn = dataPtr;

    // copy to points
    points.resize(height * width);
    luminance.resize(height);
    int numPoints = 0;
    for (size_t i = 0; i < height; i++) {
        luminance[i].resize(width);
        for (size_t j = 0; j < width; j++) {
            int16_t ix = *reinterpret_cast<const int16_t *>(pIn);
            int16_t iy = *reinterpret_cast<const int16_t *>((pIn + 2));
            int16_t iz = *reinterpret_cast<const int16_t *>((pIn + 4));
            uint16_t l = *reinterpret_cast<const uint16_t *>((pIn + 6));
            pIn += srcPixelSize;

            luminance[i][j] = l;

            if (iz == -1)
                continue;

            uint16_t x = reinterpret_cast<const uint16_t &>(ix);
            uint16_t y = reinterpret_cast<const uint16_t &>(iy);
            uint16_t z = reinterpret_cast<const uint16_t &>(iz);
            points[numPoints] = cv::Point3d(x, y, z) * scale;
            numPoints++;
        }
    }
    points.resize(numPoints);

    _device->RequeueBuffer(pImage);

    return numPoints;
}














