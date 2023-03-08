#include "myslam/visual_odometry.h"
#include "myslam/common_include.h"


int main(int argc, char **argv){

    std::string config_file_path = "../config/default.yaml";
    myslam::VisualOdometry::Ptr vo(
        new myslam::VisualOdometry(config_file_path));
    assert(vo->Init() == true);
    std::cout << "vo->Init() over" << std::endl;
    vo->run();
    
}
