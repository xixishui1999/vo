#pragma once

#include <pangolin/pangolin.h>

#include "myslam/common_include.h"
#include "myslam/frame.h"
#include "myslam/map.h"
#include "myslam/feature.h"

namespace myslam{
class Viewer{

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Viewer> Ptr;

    Viewer();

    void SetMap(Map::Ptr map) {map_ = map;}

    void Close();

    void AddcurrentFrame(Frame::Ptr current_frame);//对外接口函数，接受帧

    void UpdateMap();

  private:
    void ThreadLoop();

    void DrawFrame(Frame::Ptr frame, const float* color);

    void FollowCurrentFrame(pangolin::OpenGlRenderState& vis_camera);

    void DrawMapPoints();

    cv::Mat PlotFrameImage();

    Frame::Ptr current_frame_ = nullptr;
    Map::Ptr map_ = nullptr;

    std::thread viewer_thread_;
    bool viewer_running_ = true;

    std::unordered_map<unsigned long, Frame::Ptr> active_keyframes_;
    std::unordered_map<unsigned long, MapPoint::Ptr> active_landmarks_;
    bool map_updated_ = false;

    std::mutex viewer_data_mutex_;
};
}