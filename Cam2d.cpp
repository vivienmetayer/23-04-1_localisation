//
// Created by Vivien on 30/05/2023.
//

#include "Cam2d.h"

Cam2d::Cam2d(Arena::ISystem *system, int deviceIndex) : ArenaCam(system, deviceIndex) {

}

Cam2d::Cam2d(Arena::ISystem *system, int deviceIndex, uint64_t forcedIp, uint64_t subnetMask, uint64_t gateway)
        : ArenaCam(system, deviceIndex, forcedIp, subnetMask, gateway) {

}

Cam2d::~Cam2d() {

}

int Cam2d::getImage(cv::Mat &image, uint64_t timeout) {
    // grab
    Arena::IImage* pImage = _device->GetImage(timeout);

    // save to temp image
    size_t width = pImage->GetWidth();
    size_t height = pImage->GetHeight();
    cv::Mat imageTemp(height, width, CV_8UC3);
    cv::Mat imageTemp2;
    memcpy(imageTemp.data, pImage->GetData(), height * width * 3);

    // change format to RGBA
    cv::cvtColor(imageTemp, imageTemp2, cv::COLOR_BGR2BGRA);

    // copy to lv image with stride
    for (int i = 0; i < height; i++) {
        memcpy(image.data + i * image.step, imageTemp2.data + i * imageTemp2.step, width * 4);
    }

    _device->RequeueBuffer(pImage);
    return 0;
}
