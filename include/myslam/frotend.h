#pragma once
#include "myslam/common_include.h"
#include "myslam/map.h"
#include "myslam/frame.h"//里面包含了camera.h
//#include "myslam/backend.h"
#include <opencv2/features2d.hpp>

namespace myslam{

class Backend;
class Viewer;

enum class FrontendStatus { INITING, TRACKING_GOOD, TRACKING_BAD, LOST};

class Frontend{

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Frontend> Ptr;

    Frontend();

    bool AddFrame(Frame::Ptr frame);

    void SetBackend(std::shared_ptr<Backend> backend) {backend_ = backend;}

    void SetMap(Map::Ptr map) {map_ = map;}

    void SetViewer(std::shared_ptr<Viewer> viewer){viewer_ = viewer;}

    void SetCameras(Camera::Ptr left, Camera::Ptr right){
      camera_left_ = left;
      camera_right_ = right;
    }

  private:
    
    bool StereoInit();

    int DetectFeature();//左目相机提取特征点

    int FindFeaturesInRight();

    bool BuildInitMap();

    bool Track();

    int TrackLastFrame();

    int EstimateCurrentPose();

    bool InsertKeyframe();

    int TriangulationNewPoints();

    Frame::Ptr current_frame_ = nullptr;
    Frame::Ptr last_frame_ = nullptr;
    Camera::Ptr camera_left_ = nullptr;
    Camera::Ptr camera_right_ = nullptr;
    
    std::shared_ptr<Backend> backend_ = nullptr;
    //为什么不这么写Backend::Ptr backend_ = nullptr; 应该是一样的
    Map::Ptr map_ = nullptr;
    std::shared_ptr<Viewer> viewer_ = nullptr;
    //std::shared_ptr<Backend> backend_ = nullptr;

    FrontendStatus status_ = FrontendStatus::INITING;

    cv::Ptr<cv::GFTTDetector> gftt_;

    SE3 relative_motion_;

    int tracking_inliers_ = 0;

    int num_features_ = 200;
    int num_features_init_ = 100;
    int num_features_tracking_ = 50;
    int num_features_tracking_bad_ = 20;
    int num_features_needed_for_keyframe_ = 80;
    
};
}