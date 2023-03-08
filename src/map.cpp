#include "myslam/map.h"
#include "myslam/feature.h"

namespace myslam{

void Map::InsertKeyFrame(Frame::Ptr frame){
    std::cout << "InsertKeyframeing" << std::endl;
    current_frame_ = frame;
    if (keyframes_.find(frame->keyframe_id_) == keyframes_.end()){
        keyframes_.insert(make_pair(frame->keyframe_id_, frame));
    }else {
        keyframes_[frame->keyframe_id_] = frame;
        active_keyframes_[frame->keyframe_id_] = frame;
    }

    if (active_keyframes_.size() > num_active_keyframe_){
        RemoveOldKeyframe();
    }
}

void Map::RemoveOldKeyframe() {
    if (current_frame_ == nullptr) return;
    double max_dis = 0, min_dis = 9999;
    double max_kf_id = 0, min_kf_id = 0;
    auto Twc = current_frame_->Pose().inverse();
    for (auto& kf : active_keyframes_){
        if(kf.second == current_frame_) continue;
        auto dis = (kf.second->Pose() * Twc).log().norm();
        if(dis > max_dis){
            max_dis = dis;
            max_kf_id = kf.first;
        }
        if(dis < min_dis){
            min_dis = dis;
            min_kf_id = kf.first;
        }
    }

    const double min_dis_th = 0.2;
    Frame::Ptr frame_to_remove = nullptr;
    if(min_dis < min_dis_th){//首先考虑删除最近的，当小于0.2时删除最近的
        frame_to_remove = keyframes_.at(min_kf_id);
    } else {
        frame_to_remove = keyframes_.at(max_kf_id);
    }

    std::cout << "remove keyframe" << frame_to_remove->keyframe_id_;
    //删除关键帧
    active_keyframes_.erase(frame_to_remove->keyframe_id_);
    //删除该帧上特征点对地图点的观测
    for (auto feat : frame_to_remove->feature_left_){
        auto mp = feat->map_point_.lock();
        if (mp){
            mp->RemoveObservation(feat);
        }
    }

    for (auto feat : frame_to_remove->feature_right_){
        if(feat == nullptr) continue;
        auto mp = feat->map_point_.lock();
        if (mp){
            mp->RemoveObservation(feat);
        }
    }


}

void Map::InsertMapPoint(MapPoint::Ptr map_point){
    if (landmarks_.find(map_point->id_) == landmarks_.end()) {
        landmarks_.insert(make_pair(map_point->id_, map_point));
        active_landmarks_.insert(make_pair(map_point->id_, map_point));
    } else {
        landmarks_[map_point->id_] = map_point;
        active_landmarks_[map_point->id_] = map_point;
    }
}

}