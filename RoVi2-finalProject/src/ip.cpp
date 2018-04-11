#include "ip.h"

cv::Mat ip::toOpenCVImage(const rw::sensor::Image& img) {
    cv::Mat res(img.getHeight(),img.getWidth(), CV_8UC3);
    res.data = (uchar*)img.getImageData();
    return res;
}

