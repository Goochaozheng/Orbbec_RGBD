#include "depthSource.hpp"

#include <iostream>
#include <fstream>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/rgbd/kinfu.hpp>
#include <opencv2/imgcodecs.hpp>
#include <openni2/OpenNI.h>

DepthSource::DepthSource(cv::String fileListName):
    depthFileList(fileListName.empty() ? std::vector<std::string>() : readDepth(fileListName)),
    frameIdx(0),
    mMode(MODE::FILE)
{ }

DepthSource::DepthSource():
    frameIdx(0),
    mMode(MODE::CAMERA),
    deviceURI(openni::ANY_DEVICE)
{ 
    initDevice();
}

void DepthSource::initDevice(){

    mStatus = openni::OpenNI::initialize();
    std::cout << "OpenNI initialized" << std::endl;

    if(mStatus != openni::STATUS_OK){
        throw std::runtime_error("OpenNI initialize fail");
    }

    mStatus = mDevice.open(openni::ANY_DEVICE);
    std::cout << "Camera opened" << std::endl;

    if(mStatus != openni::STATUS_OK){
        printf("%s\n", openni::OpenNI::getExtendedError());
        openni::OpenNI::shutdown();
        throw std::runtime_error("Fail to initialize camera");
    }else{
        std::cout << "Camera open" << std::endl;
    }
    
    mStatus = mDepthStream.create(mDevice, openni::SENSOR_DEPTH);
    if(mStatus != openni::STATUS_OK){
        printf("%s\n", openni::OpenNI::getExtendedError());
        openni::OpenNI::shutdown();
        throw std::runtime_error("Fail to create depth stream");
    }else{
        std::cout << "Stream Created" << std::endl;
    }

    mStatus = mDepthStream.start();
    if(mStatus != openni::STATUS_OK){
        printf("Can not start depth stream:\n%s\n", openni::OpenNI::getExtendedError());
        openni::OpenNI::shutdown();
        throw std::runtime_error("Fail to start depth stream");
    }else{
        std::cout << "Stream start" << std::endl;
    }
    
    if(mDepthStream.isValid()){
        mDepthVideoMode = mDepthStream.getVideoMode();
        mStreams = new openni::VideoStream* [1];
        mStreams[0] = &mDepthStream;
    }

}

std::vector<std::string> DepthSource::readDepth(std::string fileList){

    std::vector<std::string> v;
    std::fstream file(fileList);
    if(!file.is_open())
        throw std::runtime_error("Failed to read depth list");

    std::string dir;
    size_t slashIdx = fileList.rfind('/');
    slashIdx = slashIdx != std::string::npos ? slashIdx : fileList.rfind('\\');
    dir = fileList.substr(0, slashIdx);

    while(!file.eof()){
        std::string s, imgPath;
        std::getline(file, s);
        if(s.empty() || s[0]=='#') continue;
        std::stringstream ss;
        ss << s;
        double thumb;
        ss >> thumb >> imgPath;
        v.push_back(dir+'/'+imgPath);
    }

    return v;

}

cv::UMat DepthSource::getDepth(){
    cv::UMat out;

    if(mMode == MODE::FILE){
        if(frameIdx < depthFileList.size()){
            cv::Mat f = cv::imread(depthFileList[frameIdx++], cv::IMREAD_ANYDEPTH);
            f.copyTo(out);
        }

        if(out.empty())
            throw std::runtime_error("Matrix is empty");

    }else if(mMode == MODE::CAMERA){

        int streamReadyIndex;
        mStatus = openni::OpenNI::waitForAnyStream(mStreams, 1, &streamReadyIndex);

        if(mStatus != openni::STATUS_OK){
            throw std::runtime_error("No stream from camera");
        }

        mDepthStream.readFrame(&mDepthFrame);
        int timestamp = mDepthFrame.getTimestamp();

        if(!mDepthFrame.isValid()){
            throw std::runtime_error("Frame invalid");
        }else{
            std::cout << "Frame read, " << timestamp << std::endl;
            cv::Mat f = cv::Mat(mDepthFrame.getHeight(), mDepthFrame.getWidth(), CV_16U, (void*)mDepthFrame.getData());
            
            // double maxVal, minVal;
            // cv::Point maxLoc, minLoc;
            // cv::minMaxLoc(f, &minVal, &maxVal, &minLoc, &maxLoc);
            // std::cout << minVal << std::endl << maxVal << std::endl;

            f.copyTo(out);
        }

    }

    return out;

}