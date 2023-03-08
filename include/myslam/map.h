#pragma once
#include "myslam/common_include.h"
#include "myslam/mappoint.h"
#include "myslam/frame.h"

namespace myslam{

class Map{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Map> Ptr;
    typedef std::unordered_map<unsigned long, MapPoint::Ptr> LandmarksType;
    typedef std::unordered_map<unsigned long, Frame::Ptr> KeyframesType;

    Map(){}

    void InsertKeyFrame(Frame::Ptr frame);

    void InsertMapPoint(MapPoint::Ptr map_point);

    //获取所有地图点
    LandmarksType GetAllMapPoints() {
      std::unique_lock<std::mutex> lck(data_mutex_);
      return landmarks_;
    }

    //获取激活地图点
    LandmarksType GetActiveMapPoints(){
      std::unique_lock<std::mutex> lck(data_mutex_);
      return active_landmarks_;
    }

    KeyframesType GetAllKeyFrame() {
      std::unique_lock<std::mutex> lck(data_mutex_);
      return keyframes_;
    }

    KeyframesType GetActiveKeyFrame() {
      std::unique_lock<std::mutex> lck(data_mutex_);
      return active_keyframes_;
    }

    void CleanMap();

  private:
    //将旧的关键帧设置为不活跃状态
    void RemoveOldKeyframe();

    std::mutex data_mutex_;
    LandmarksType landmarks_;
    LandmarksType active_landmarks_;
    KeyframesType keyframes_;
    KeyframesType active_keyframes_;

    Frame::Ptr current_frame_ = nullptr;

    int num_active_keyframe_ = 7;
};
}