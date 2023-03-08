#include "myslam/visual_odometry.h"

namespace myslam{

//run.cpp config_file_path = "../config/default.yaml";
//构造函数自动执行，赋值路径值    
VisualOdometry::VisualOdometry(std::string &config_path)
    : config_file_path_(config_path){}

bool VisualOdometry::Init(){
    //读取配置文件
    if (Config::SetParameterFile(config_file_path_) == false){
        std::cout << "SetParameterFile false" << std::endl;
        return false;
    }
    //读取相机内外参
    dataset_ = Dataset::Ptr(new Dataset(Config::Get<std::string>("dataset_dir")));
    dataset_->Init();

    frontend_ = Frontend::Ptr(new Frontend);
    backend_ = Backend::Ptr(new Backend);
    map_ = Map::Ptr(new Map);
    viewer_ = Viewer::Ptr(new Viewer);//这句话开启了显示窗口的并行运行

    frontend_->SetBackend(backend_);
    frontend_->SetMap(map_);
    frontend_->SetViewer(viewer_);
    frontend_->SetCameras(dataset_->GetCamera(0), dataset_->GetCamera(1));

    backend_->SetMap(map_);
    backend_->SetCameras(dataset_->GetCamera(0), dataset_->GetCamera(1));

    viewer_->SetMap(map_);

    return true;    
}

void VisualOdometry::run(){
    std::cout << "vo is running " << std::endl;
    while(1){
        if (step() == false)
        break;
    }
    std::cout << "vo is over " << std::endl;
}

bool VisualOdometry::step(){
    Frame::Ptr new_frame = dataset_->NextFrame();
    if (new_frame == nullptr) return false;

    bool success = frontend_->AddFrame(new_frame);
    return success;
}



}