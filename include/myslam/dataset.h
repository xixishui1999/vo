#pragma once
#include "common_include.h"
#include "myslam/camera.h"
#include "myslam/frame.h"

namespace myslam{

class Dataset{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Dataset> Ptr;
    Dataset(const std::string& dataset_path);
    Dataset(){}

    bool Init();

    Frame::Ptr NextFrame();

    Camera::Ptr GetCamera(int camera_id) const{
      return camera_.at(camera_id);
    }

  private:
    std::string dataset_path_;
    std::vector<Camera::Ptr> camera_;
    int current_image_index_ = 0;



};


}