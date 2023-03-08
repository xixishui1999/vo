#pragma once

#include "myslam/camera.h"
#include "myslam/common_include.h"


namespace myslam{

struct MapPoint;//需要前置声明，不然后面报错，不知道有Frame这个数据结构
struct Feature;

struct Frame
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Frame> Ptr;

    //每一帧应该具有的属性
    unsigned long id_ = 0;
    unsigned long keyframe_id_ = 0;
    bool is_keyframe_ = false;
    double time_stamp_;
    SE3 pose_; //使用李代数库sophus定义 sophus也是基于eigen开发的 书P86
    std::mutex pose_mutex_;
    cv::Mat left_img_, right_img_;
    std::vector<std::shared_ptr<Feature>> feature_left_;
    std::vector<std::shared_ptr<Feature>> feature_right_;

  public:
    Frame(){}
    Frame(long id, double time_stamp, const SE3 &pose, const Mat &left,
            const Mat &right);//这个函数干啥的？也是构造，在.c文件中具体写
    //Get pose
    SE3 Pose(){
        std::unique_lock<std::mutex> lck(pose_mutex_);
        return pose_;
    }
    //Set pose
    void SetPose(const SE3 &pose){
        std::unique_lock<std::mutex> lck(pose_mutex_);
        pose_ = pose;
    } 
    //设置关键帧并分配关键帧ID
    void SetKeyFrame();
    //工厂构建模式，分配ID？？？？？？？？
    static std::shared_ptr<Frame> CreateFrame();


};

}