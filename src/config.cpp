#include "myslam/config.h"


namespace myslam{

bool Config::SetParameterFile(const std::string &filename) {
    if (config_ == nullptr)
        config_ = std::shared_ptr<Config>(new Config);
    config_->file_ = cv::FileStorage(filename, cv::FileStorage::READ);
    if (config_->file_.isOpened() == false) {
        //LOG(ERROR) << "parameter file " << filename << " does not exist.";
        std::cout << "openfalse" << std::endl;
        config_->file_.release();
        return false;
    }
    std::cout << "read done" << std::endl;
    return true;
}

Config::~Config() {
    if (file_.isOpened())
        file_.release();
}

std::shared_ptr<Config> Config::config_ = nullptr;


}