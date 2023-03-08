#pragma once
#include "myslam/common_include.h"

namespace myslam{

class Config{
  public:
    ~Config();

    template <typename T>
    static T Get(const std::string &key){
        return T(Config::config_->file_[key]);
    } 
    static bool SetParameterFile(const std::string &filename);
    //Config(){}
    
  private:
    Config(){}
    
    static std::shared_ptr<Config> config_;
    cv::FileStorage file_;
};

}