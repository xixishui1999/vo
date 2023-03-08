#pragma once
#include "myslam/common_include.h"
#include "myslam/dataset.h"
#include "myslam/config.h"
#include "myslam/frotend.h"
#include "myslam/backend.h"
#include "myslam/map.h"
#include "myslam/viewer.h"

namespace myslam{

class VisualOdometry{

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<VisualOdometry> Ptr;

    VisualOdometry(std::string &config_path);

    bool Init();

    void run();

    bool step();

  private:
    Dataset::Ptr dataset_ = nullptr;
    Frontend::Ptr frontend_ = nullptr;
    Backend::Ptr backend_ = nullptr;
    Map::Ptr map_ = nullptr;
    Viewer::Ptr viewer_ = nullptr;

    std::string config_file_path_;


};
}