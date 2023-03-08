#pragma once

#include <memory>
#include <opencv2/features2d.hpp>
#include "myslam/common_include.h"
//#include "myslam/mappoint.h"引入这个会报错 得用struct MapPoint;为啥？

namespace myslam{

struct Frame;
struct MapPoint;

struct Feature{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Feature> Ptr;

    std::weak_ptr<Frame> frame_;//持有该feature的frame
    std::weak_ptr<MapPoint> map_point_; //feature关联的地图点
    cv::KeyPoint position_; //2D位置
    bool is_outlier_ = false;
    bool is_on_left_image_  = true;

  public:
    Feature(){}
    Feature(std::shared_ptr<Frame> frame, const cv::KeyPoint &kp)
        : frame_(frame), position_(kp) {}
};
}