#ifndef DEPTH_SOURCE_H
#define DEPTH_SOURCE_H

#include <opencv2/core.hpp>
#include <OpenNI.h>

// Object that read depth image from file list
class DepthSource{

private:
    enum MODE{
        CAMERA,
        FILE
    };

public:

    std::vector<std::string> depthFileList;
    size_t frameIdx;
    
    openni::Status mStatus;
    openni::Device mDevice;

    openni::VideoStream mDepthStream;
    openni::VideoStream mColorStream;
    openni::VideoStream** mStreams;
    
    openni::VideoFrameRef mDepthFrame;
    openni::VideoFrameRef mColorFrame;

    openni::VideoMode mDepthVideoMode;
    
    openni::DepthPixel* mDepthPixel;

    const int mMode;
    const char* deviceURI;

    DepthSource(cv::String fileListName);
    DepthSource();

    // Read depth image path from depth.txt in TUM dataset
    std::vector<std::string> readDepth(std::string fileList);

    // Open camera device
    void initDevice();

    // read depth image list
    cv::UMat getDepth();
    cv::UMat getDepth(openni::VideoStream depth);

};

#endif