#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc_c.h>//不然报CV_FILLED’ was not declared in this scope

#include "myslam/frotend.h"
#include "myslam/backend.h"
#include "myslam/feature.h"
#include "myslam/config.h"
#include "myslam/viewer.h"
#include "myslam/map.h"
#include "myslam/algorithm.h"
#include "myslam/g2o_types.h"

namespace myslam
{

Frontend::Frontend(){
    gftt_ = 
        cv::GFTTDetector::create(Config::Get<int>("num_features"), 0.01, 20);
    num_features_init_ = Config::Get<int>("num_features_init");
    num_features_ = Config::Get<int>("num_features");
}

bool Frontend::AddFrame(Frame::Ptr frame){
    current_frame_ = frame;

    switch(status_){
        case FrontendStatus::INITING:
            StereoInit();
            break;
        case FrontendStatus::TRACKING_GOOD:
        case FrontendStatus::TRACKING_BAD:
            //std::cout << "TRACKING_GOOD" << std::endl;
            Track();
            break;
        case FrontendStatus::LOST:
            //Reset();
            break;
    }

    last_frame_ = current_frame_;
    return true;
}

bool Frontend::StereoInit(){
    std::cout << "stereo init" << std::endl;
    //int num_features_left = DetectFeature();
    DetectFeature();
    int num_coor_features = FindFeaturesInRight();
    if (num_coor_features < num_features_init_){
        return false;
    }
    bool build_map_success = BuildInitMap();
    if (build_map_success) {
        status_ = FrontendStatus::TRACKING_GOOD;
        if (viewer_){
            viewer_->AddcurrentFrame(current_frame_);
        }
        return true;
    }
    return false;
}

int Frontend::DetectFeature(){
    cv::Mat mask(current_frame_->left_img_.size(), CV_8UC1, 255);
    for (auto &feat : current_frame_->feature_left_){
        cv::rectangle(mask, feat->position_.pt - cv::Point2f(10, 10),
                        feat->position_.pt + cv::Point2f(10, 10), 0, CV_FILLED);
    }

    std::vector<cv::KeyPoint> keypoints;
    gftt_->detect(current_frame_->left_img_, keypoints, mask);
    int cnt_detected = 0;
    for (auto &kp : keypoints){
        current_frame_->feature_left_.push_back(
                Feature::Ptr(new Feature(current_frame_, kp)));
        cnt_detected++;
    }
    //std::cout << current_frame_->feature_left_.size() << std::endl;
    std::cout << "Detect" << cnt_detected << "new features" << std::endl;
    return cnt_detected;
}
//对当前帧的每个左点，找右图对应的点，先根据左点位置给一个初始位置
int Frontend::FindFeaturesInRight(){
    std::vector<cv::Point2f> kps_left, kps_right;
    for (auto &kp : current_frame_->feature_left_){//std::vector<std::shared_ptr<Feature>> feature_left_;
        kps_left.push_back(kp->position_.pt);//cv::KeyPoint position_; //2D位置
        auto mp = kp->map_point_.lock();//std::weak_ptr<MapPoint> map_point_; //feature关联的地图点
        //std::cout << "FindFeaturesInRight" << std::endl;
        if (mp){//初始化的时候还没有创建地图点，所以此时不会执行，在buildmap中创建了地图点 .lock()获取weak_ptr对应的对象
            //std::cout << "--------------" << std::endl;
            auto px = camera_right_->world2pixel(mp->pos_, current_frame_->Pose());
            //std::cout << "mp->pos_" << mp->pos_ << std::endl;
            kps_right.push_back(cv::Point2f(px[0], px[1]));
        }else{
            kps_right.push_back(kp->position_.pt);
        }

    }//获取每个左点在右图的初始位置
    //std::cout << "kps_left: " << current_frame_->feature_left_.size() << std::endl;
    std::vector<uchar> status;
    Mat error;
    cv::calcOpticalFlowPyrLK(
        current_frame_->left_img_, current_frame_->right_img_, kps_left,
        kps_right, status, error, cv::Size(11, 11), 3,
        cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,
                        0.01));
    // cv::OPTFLOW_USE_INITIAL_FLOW这是最后一个可选项flag，加上会报错，不知道为啥
    // 不加的话默认以输入点的位置进行光流，加的话就是自己设置初始值，对应上面的投影位置
    // cv::calcOpticalFlowPyrLK(
    //     current_frame_->left_img_, current_frame_->right_img_, kps_left,
    //     kps_right, status, error, cv::Size(11, 11), 3);
    int num_good_pts = 0;
    for (size_t i = 0; i < status.size(); ++i){
        if (status[i]){
            cv::KeyPoint kp(kps_right[i], 7);
            Feature::Ptr feat(new Feature(current_frame_, kp));
            feat->is_on_left_image_ = false;
            current_frame_->feature_right_.push_back(feat);
            num_good_pts++;
        }else
            current_frame_->feature_right_.push_back(nullptr);
    }
    std::cout << "Find" << num_good_pts << "features in the right image" << std::endl;
    return num_good_pts;
}

bool Frontend::BuildInitMap(){
    std::vector<SE3> poses{camera_left_->pose(), camera_right_->pose()};
    //初始化时 这个时候应该还没有位姿，旋转为单位阵，平移是0
    size_t cnt_int_landmarks = 0;
    //std::cout << camera_left_->pose().matrix() << std::endl;输出SE3时候，使用.matrix()
    for (size_t i = 0; i < current_frame_->feature_left_.size(); ++i){
        if (current_frame_->feature_right_[i] == nullptr) continue;
        std::vector<Vec3> points{
            camera_left_->pixel2camera(
                Vec2(current_frame_->feature_left_[i]->position_.pt.x,
                     current_frame_->feature_left_[i]->position_.pt.y)),//这里固定了depth=1
            camera_right_->pixel2camera(
                Vec2(current_frame_->feature_right_[i]->position_.pt.x,
                     current_frame_->feature_right_[i]->position_.pt.y))};
        Vec3 pworld = Vec3::Zero(); //pworld triangulation里面使用引用直接修改
        //std::cout << points.at(1) << std::endl;
        //std::cout << "here" << std::endl;
        // std::cout << "current i = " << i << std::endl;
        // std::cout << camera_left_->pose().matrix() << std::endl;
        // std::cout << camera_right_->pose().matrix() << std::endl;
        // std::cout << points.at(0) << std::endl;
        // std::cout << points.at(1) << std::endl;
        if (triangulation(poses, points, pworld) && pworld[2] > 0){
            auto new_map_point = MapPoint::CreateNewMappoint();
            //std::cout << "pworld = " << std::endl;
            //std::cout << pworld << std::endl;
            new_map_point->SetPose(pworld);
            new_map_point->AddObservation(current_frame_->feature_left_[i]);
            new_map_point->AddObservation(current_frame_->feature_right_[i]);
            current_frame_->feature_left_[i]->map_point_ = new_map_point;
            current_frame_->feature_right_[i]->map_point_ = new_map_point;
            cnt_int_landmarks++;
            map_->InsertMapPoint(new_map_point);
        }
    }
    current_frame_->SetKeyFrame();
    std::cout << "initial map created with" << cnt_int_landmarks << "map points" << std::endl;
    std::cout << "BuildInitMap over" << std::endl;
    map_->InsertKeyFrame(current_frame_);

    return true;
}

bool Frontend::Track(){
    if (last_frame_){
        current_frame_->SetPose(relative_motion_ * last_frame_->Pose());
        //计算第二帧时候，relative_motion_还是单位阵
    }
    std::cout << "start track" << std::endl;
    //int num_track_last = TrackLastFrame();
    TrackLastFrame();
    tracking_inliers_ = EstimateCurrentPose();//G2O
    

    if (tracking_inliers_ < num_features_tracking_){
        status_ = FrontendStatus::TRACKING_GOOD;
    }else if (tracking_inliers_ > num_features_tracking_bad_){
        status_ = FrontendStatus::TRACKING_BAD;
    }else{
        status_ = FrontendStatus::LOST;
    }

    InsertKeyframe();
    relative_motion_ = current_frame_->Pose() * last_frame_->Pose().inverse();
    
    if(viewer_) viewer_->AddcurrentFrame(current_frame_);
    return true;
}

int Frontend::TrackLastFrame(){
    // use LK flow to estimate points in the right image
    std::vector<cv::Point2f> kps_last, kps_current;
    for (auto &kp : last_frame_->feature_left_) {
        if (kp->map_point_.lock()) {
            // use project point
            auto mp = kp->map_point_.lock();
            auto px =
                camera_left_->world2pixel(mp->pos_, current_frame_->Pose());
            kps_last.push_back(kp->position_.pt);
            kps_current.push_back(cv::Point2f(px[0], px[1]));
        } else {
            kps_last.push_back(kp->position_.pt);
            kps_current.push_back(kp->position_.pt);
        }
    }

    std::vector<uchar> status;
    Mat error;
    cv::calcOpticalFlowPyrLK(
        last_frame_->left_img_, current_frame_->left_img_, kps_last,
        kps_current, status, error, cv::Size(11, 11), 3,
        cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,
                         0.01),
        cv::OPTFLOW_USE_INITIAL_FLOW);

    int num_good_pts = 0;

    for (size_t i = 0; i < status.size(); ++i) {
        if (status[i]) {
            cv::KeyPoint kp(kps_current[i], 7);
            Feature::Ptr feature(new Feature(current_frame_, kp));
            feature->map_point_ = last_frame_->feature_left_[i]->map_point_;
            current_frame_->feature_left_.push_back(feature);
            num_good_pts++;
        }
    }

    std::cout << "Find " << num_good_pts << " in the last image." << std::endl;
    return num_good_pts;
}
//g2o general总体的，通用的 graph图表 optimization
int Frontend::EstimateCurrentPose(){

    typedef g2o::BlockSolver_6_3 BlockSolverType;//表示pose是6维，观测点是3维，用于3D SLAM中的BA
    typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType>
        LinearSolverType;//使用dense cholesky分解法，继承自linearsolver
    auto solver = new g2o::OptimizationAlgorithmLevenberg(
        g2o::make_unique<BlockSolverType>(
            g2o::make_unique<LinearSolverType>()));
    g2o::SparseOptimizer optimizer;//创建图的核心，稀疏优化器
    optimizer.setAlgorithm(solver);//设置求解器

/*********定义图的顶点和边，并添加到SparseOptimizer中，即optimizer**************/
    VertexPose *vertex_pose = new VertexPose();//添加顶点
    vertex_pose->setId(0);
    vertex_pose->setEstimate(current_frame_->Pose());
    std::cout << "before optimization current pose =" << std::endl;
    std::cout << current_frame_->Pose().matrix() << std::endl;
    optimizer.addVertex(vertex_pose);

    Mat33 K = camera_left_->K();

    int index = 1;
    std::vector<EdgeProjectionPoseOnly *> edges;
    std::vector<Feature::Ptr> features;
    //往图中添加边
    for (size_t i = 0; i < current_frame_->feature_left_.size(); ++i){
        auto mp = current_frame_->feature_left_[i]->map_point_.lock();
        if (mp){
            //std::cout << "xxxxxxxxxxxxxxxxxxxx" << std::endl;
            features.push_back(current_frame_->feature_left_[i]);
            EdgeProjectionPoseOnly *edge = 
                new EdgeProjectionPoseOnly(mp->pos_, K);
            edge->setId(index);
            edge->setVertex(0, vertex_pose);//设置连接的顶点
            edge->setMeasurement(
                toVec2(current_frame_->feature_left_[i]->position_.pt));//观测数值
            edge->setInformation(Eigen::Matrix2d::Identity());
            edge->setRobustKernel(new g2o::RobustKernelHuber);
            edges.push_back(edge);
            optimizer.addEdge(edge);
            index++;
        }
    }

    const double chi2_th = 5.991;
    int cnt_outlier = 0;
    for (int iteration = 0; iteration < 4; ++iteration){
        vertex_pose->setEstimate(current_frame_->Pose());
        optimizer.initializeOptimization();//初始化
        optimizer.optimize(10);//设置迭代次数
        cnt_outlier = 0;

        for (size_t i = 0; i < edges.size(); ++i){
            auto e = edges[i];
            if (features[i]->is_outlier_){
                e->computeError();
            }
            if (e->chi2() > chi2_th){
                features[i]->is_outlier_ = true;
                e->setLevel(1);
                cnt_outlier++;
            }else{
                features[i]->is_outlier_ = false;
                e->setLevel(0);
            }
            if (iteration == 2){
                e->setRobustKernel(nullptr);
            }
        }
    }

    std::cout << "outlier/inlier" << cnt_outlier << "/" << features.size() - cnt_outlier
    << std::endl;

    current_frame_->SetPose(vertex_pose->estimate());
    std::cout << "after optimization current pose =" << std::endl;
    std::cout << current_frame_->Pose().matrix() << std::endl;

    return features.size() - cnt_outlier;
}

bool Frontend::InsertKeyframe(){
    if (tracking_inliers_ >= num_features_needed_for_keyframe_){
        return false;
    }
    
    current_frame_->SetKeyFrame();
    map_->InsertKeyFrame(current_frame_);

    //SetObservationsForKeyFrame();

    DetectFeature();
    FindFeaturesInRight();
    TriangulationNewPoints();
    //backend_->

    return true;
}

int Frontend::TriangulationNewPoints(){
    std::vector<SE3> poses{camera_left_->pose(), camera_right_->pose()};
    SE3 current_pose_Twc = current_frame_->Pose().inverse();
    int cnt_triangulated_pts = 0;
    for (size_t i = 0; i < current_frame_->feature_left_.size(); ++i){
        if (current_frame_->feature_right_[i] == nullptr) continue;
        std::vector<Vec3> points{
            camera_left_->pixel2camera(
                Vec2(current_frame_->feature_left_[i]->position_.pt.x,
                     current_frame_->feature_left_[i]->position_.pt.y)),//这里固定了depth=1
            camera_right_->pixel2camera(
                Vec2(current_frame_->feature_right_[i]->position_.pt.x,
                     current_frame_->feature_right_[i]->position_.pt.y))};
        Vec3 pworld = Vec3::Zero();

        if (triangulation(poses, points, pworld) && pworld[2] > 0){
            auto new_map_point = MapPoint::CreateNewMappoint();
            pworld = current_pose_Twc * pworld;//很重要
            new_map_point->SetPose(pworld);
            new_map_point->AddObservation(current_frame_->feature_left_[i]);
            new_map_point->AddObservation(current_frame_->feature_right_[i]);
            current_frame_->feature_left_[i]->map_point_ = new_map_point;
            current_frame_->feature_right_[i]->map_point_ = new_map_point;
            map_->InsertMapPoint(new_map_point);
            cnt_triangulated_pts++;
        }
    }

    std::cout << "TriangulationNewPoints with" << cnt_triangulated_pts << "map points" << std::endl;
    map_->InsertKeyFrame(current_frame_);

    return cnt_triangulated_pts;
}


} // namespace myslam
