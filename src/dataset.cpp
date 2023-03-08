#include "myslam/dataset.h"
#include <fstream>
#include <boost/format.hpp>

using namespace std;

namespace myslam{

Dataset::Dataset(const std::string& dataset_path)
    : dataset_path_(dataset_path){
    std::cout << dataset_path_ << std::endl;
    }

bool Dataset::Init(){
    std::cout << "dataset init" << std::endl;
    //dataset_path_
    ifstream fin(dataset_path_ + "/calib.txt");
    if(!fin){
        cout << "can not find " << dataset_path_<< "/calib.txt" << endl;
    }

    for (int i = 0; i < 4; ++i){
        char camera_name[3];
        for (int k = 0; k < 3; ++k){
            fin >> camera_name[k];
        }
        //cout << camera_name << endl;
        double projection_data[12];
        for (int k = 0; k < 12; ++k){
            fin >> projection_data[k];
        }
        //cout << projection_data << endl;
        Mat33 K;
        K << projection_data[0], projection_data[1], projection_data[2],
            projection_data[4], projection_data[5], projection_data[6],
            projection_data[8], projection_data[9], projection_data[10];
        Vec3 t;
        t << projection_data[3], projection_data[7], projection_data[11];//t是干嘛的
        t = K.inverse() * t;
        K = K * 0.5;//这两步在算什么？
        Camera::Ptr new_camera(new Camera(K(0, 0), K(1, 1), K(0, 2), K(1, 2),
                                            t.norm(), SE3(SO3(), t)));//SE3(SO3(), t))怎么操作的
        camera_.push_back(new_camera);
        cout << i << "extrinsics:" << t.transpose() << endl;
    }
    fin.close();
    current_image_index_ = 0;
    //cout << camera_.at(0) << endl;//输出指针对应的地址
    //cout << camera_.at(1) << endl;
    return true;
}

Frame::Ptr Dataset::NextFrame(){
    boost::format fmt("%s/image_%d/%06d.png");
    cv::Mat image_left, image_right;
    image_left = 
        cv::imread((fmt % dataset_path_ % 0 % current_image_index_).str(),
                    cv::IMREAD_GRAYSCALE);
    image_right = 
        cv::imread((fmt % dataset_path_ % 1 % current_image_index_).str(),
                    cv::IMREAD_GRAYSCALE);
    if (image_left.data == nullptr || image_right.data == nullptr){
        cout << "cannot find image at index" << current_image_index_ << endl;
        return nullptr;
    }

    cv::Mat image_left_resized, image_right_resized;
    cv::resize(image_left, image_left_resized, cv::Size(), 0.5, 0.5,
                cv::INTER_NEAREST);
    cv::resize(image_right, image_right_resized, cv::Size(), 0.5, 0.5,
                cv::INTER_NEAREST);
    
    auto new_frame = Frame::CreateFrame();
    new_frame->left_img_ = image_left_resized;
    new_frame->right_img_ = image_right_resized;
    current_image_index_++;
    return new_frame;

}

}

