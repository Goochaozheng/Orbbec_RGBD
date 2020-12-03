#include <iostream>
#include <fstream>
#include <opencv2/rgbd/kinfu.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/viz.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/core/utility.hpp>

#include "depthSource.hpp"
// using namespace std;
// using namespace cv;
// using namespace cv::kinfu;


// Visualization
struct PauseCallbackArgs{
    PauseCallbackArgs(cv::kinfu::KinFu& _kf) : kf(_kf) {}
    cv::kinfu::KinFu& kf;
};

void pauseCallback(const cv::viz::MouseEvent& me, void* args){
    if(me.type == cv::viz::MouseEvent::Type::MouseMove ||
       me.type == cv::viz::MouseEvent::Type::MouseScrollDown ||
       me.type == cv::viz::MouseEvent::Type::MouseScrollUp)
    {
        PauseCallbackArgs pca = *((PauseCallbackArgs*)(args));
        cv::viz::Viz3d window("Cloud");
        cv::UMat rendered;
        pca.kf.render(rendered, window.getViewerPose().matrix);
        cv::imshow("render", rendered);
        cv::waitKey(1);      
    }
}

static const char* keys =
{
    "{help h usage ? | | print this message   }"
    "{depth  | | Path to depth.txt file listing a set of depth images }"
    "{coarse | | Run on coarse settings (fast but ugly) or on default (slow but looks better),"
        " in coarse mode points and normals are displayed }"
    "{idle   | | Do not run KinFu, just display depth frames }"
    "{camera | | Use connected camera to run}"
    "{color  | | Show color stream}"
};

int main(int argc, char **argv){
    
    bool coarse = false;
    bool idle = false;
    bool color = false;
    
    // Parse arguement
    cv::CommandLineParser parser(argc, argv, keys);
    
    if(!parser.check()){
        parser.printErrors();
        return -1;
    }
    
    if(parser.has("help")){
        parser.printMessage();
        return 0;
    }

    if(parser.has("coarse")){
        coarse = true;
    }

    if(parser.has("idle")){
        idle = true;
    }

    if(parser.has("color")){
        color = true;
        // TODO Color stream source
    }

    // Create data source
    cv::Ptr<DepthSource> depthSource;
    // Setup parameters
    cv::Ptr<cv::kinfu::Params> params;
    
    if(coarse){
        params = cv::kinfu::Params::coarseParams();
    }else{
        params = cv::kinfu::Params::defaultParams();
    }

    if(parser.has("camera")){
        depthSource = cv::makePtr<DepthSource>();
    }else if(parser.has("depth")){
        depthSource = cv::makePtr<DepthSource>(parser.get<cv::String>("depth"));
    }
    std::cout << "Depth Source created" << std::endl;

    float fx = 597.0702;
    float fy = 595.1533;
    float cx = 317.4329;
    float cy = 240.6083;
    params->intr = cv::Matx33f(fx, 0,  cx,
                                0, fy, cy,
                                0, 0,  1);

    // params->depthFactor = 5000.f;
    params->volumeDims = cv::Vec3i::all(1024);
    float volSize = 3.f;
    params->voxelSize = volSize / 1024.f;
    params->volumePose = cv::Affine3f().translate(cv::Vec3f(-volSize/2.f, -volSize/2.f, 0));

    cv::setUseOptimized(true);

    // Create kinfu object
    cv::Ptr<cv::kinfu::KinFu> kf;
    if(!idle){
        kf = cv::kinfu::KinFu::create(params);
    }
    std::cout << "Kinfu created" << std::endl;

    // Setup visualization
    cv::viz::Viz3d window("Cloud");
    window.setViewerPose(cv::Affine3f::Identity());
    bool pause = false;
    std::cout << "Window set" << std::endl;

    // Load image sequence and reconstruct
    cv::UMat rendered;
    cv::UMat points;
    cv::UMat normals;

    cv::Point3f beginPoint = params->volumePose*cv::Point3f(0,0,0);
    // cv::Point3f endPoint;
    std::vector<cv::Point3f> cameraPose;
    cv::viz::WCameraPosition camera(params->intr, 0.25, cv::viz::Color::red());

    int64 prevTime = cv::getTickCount();

    for(cv::UMat frame = depthSource->getDepth(); !frame.empty(); frame = depthSource->getDepth()){

        cv::UMat cvt8;
        float depthFactor = params->depthFactor;
        cv::flip(frame, frame, 1);
        // cv::convertScaleAbs(frame, cvt8, 0.25*256. / depthFactor);
        cv::convertScaleAbs(frame, cvt8, 255./3000.); 
        
        // pause
        if(!pause){

            if(!idle){
                                                       
                // Update next frame in kinfu
                if(!kf->update(frame)){
                    kf->reset();
                    std::cout << "reset" << std::endl;
                }else{
                
                    kf->getCloud(points, normals);
                    cv::Affine3f pose = kf->getPose();

                    cameraPose.push_back(pose*cv::Point3f(0,0,0));

                    if(!points.empty() && !normals.empty()){

                        // Draw point cloud
                        cv::viz::WCloud cloudWidget(points, cv::viz::Color::white());
                        window.showWidget("cloud", cloudWidget);
                        
                        // Draw camera position
                        cv::viz::WPolyLine cameraTrace(cameraPose, cv::viz::Color::green());
                        window.showWidget("camer_trace", cameraTrace);
                        window.showWidget("camera", camera, kf->getPose());

                        // Draw cube
                        cv::Vec3d volSize = kf->getParams().voxelSize*kf->getParams().volumeDims;
                        window.showWidget("cube", 
                                        cv::viz::WCube(cv::Vec3d::all(0), volSize),
                                        kf->getParams().volumePose);

                        window.spinOnce(1, true);    
                    }
                }

                // Generate phong
                kf->render(rendered);

            }else{
                // Idle mode, dsiplay depth image only
                cv::applyColorMap(cvt8, rendered, cv::COLORMAP_PARULA);
            }

        }else{

            // When pause
            kf->getCloud(points, normals);
            if(!points.empty() && !normals.empty()){
                cv::viz::WCloud cloudWidget(points, cv::viz::Color::white());
                
                window.showWidget("cloud", cloudWidget);
                                
                cv::Vec3d volSize = kf->getParams().voxelSize*cv::Vec3d(kf->getParams().volumeDims);
                
                window.showWidget("cube", 
                                  cv::viz::WCube(cv::Vec3d::all(0), volSize), 
                                  kf->getParams().volumePose);
                
                PauseCallbackArgs pca(*kf);
                window.registerMouseCallback(pauseCallback, (void*)&pca);
                window.showWidget("text", cv::viz::WText(cv::String("Move camera in this window. "
                                                                    "Close the window or press Q to resume"), cv::Point()));
                window.spin();
                window.removeWidget("text");
                window.removeWidget("cloud");
                
                window.registerMouseCallback(0);
            }

            pause = false;

        }

        int64 newTime = cv::getTickCount();
        cv::putText(rendered, 
                    cv::format("FPS: %2d press R to reset, P to pause, Q to quit", (int)(cv::getTickFrequency()/(newTime-prevTime))),
                    cv::Point(0, rendered.rows-1),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.5,
                    cv::Scalar(0, 255, 255));
        prevTime = newTime;

        cv::imshow("render", rendered);

        int c = cv::waitKey(1);
        switch (c)
        {
        case 'r':
            if(!idle)
                kf->reset();
            break;
        
        case 'q':
            return 0;

        case 'p':
            if(!idle)
                pause = true;

        default:
            break;
        }

    }

    openni::OpenNI::shutdown();
    return 0;

}