#pragma once
#include "myslam/common_include.h"
#include "myslam/frame.h"
#include "myslam/map.h"

namespace myslam{
class Backend{

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Backend> Ptr;

    Backend();

    void UpdateMap();

    void SetMap(Map::Ptr map){map_ = map;}

    void SetCameras(Camera::Ptr left, Camera::Ptr right){
      camera_left_ = left;
      camera_right_ = right;
    }

    void Stop();

  private:
      void BackendLoop();

      void Optimize(Map::KeyframesType& keyframes, Map::LandmarksType& landmarks);

    std::shared_ptr<Map> map_;
    std::thread backend_thread_;
    std::mutex data_mutex_;

    std::condition_variable map_update_;
    std::atomic<bool> backend_running_;

    Camera::Ptr camera_left_ = nullptr, camera_right_ = nullptr;

};

}