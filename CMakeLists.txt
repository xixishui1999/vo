cmake_minimum_required(VERSION 2.8)
project(myslam)

#set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_FLAGS "-std=c++11 -Wall")
set(CMAKE_CXX_FLAGS_RELEASE  "-std=c++11 -O3 -fopenmp -pthread")

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

#eigen
include_directories("/usr/include/eigen3")

# OpenCV
#find_package(OpenCV 3 REQUIRED)
find_package(OpenCV 3 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# pangolin
find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

#Sophus
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

#G2O
find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})

#CMake Error at CMakeLists.txt:28 (find_package):
#  By not providing "FindG2O.cmake" in CMAKE_MODULE_PATH this project has
#  asked CMake to find a package configuration file provided by "G2O", but
#  CMake did not find one.
#不加入find cmake会报这个错

#csparse
find_package(CSparse REQUIRED)
include_directories(${CSPARSE_INCLUDE_DIR})#注意没有S 
#include_directories("/usr/include/suitesparse") #为啥他自己没找到，添加这句话就行了，不然报错找不到#include<cs.h>

set(THIRD_PARTY_LIBS
        ${OpenCV_LIBS}
        ${Pangolin_LIBRARIES} #GL GLU GLEW glut这个不加好像也不会报错，加的意义是什么
        ${Sophus_LIBRARIES}
        #${G2O_LIBRARIES}
        g2o_core g2o_stuff g2o_types_sba g2o_solver_csparse g2o_csparse_extension
        ${CSPARSE_LIBRARY}
        )  

include_directories(${PROJECT_SOURCE_DIR}/include)
add_subdirectory(src)
add_subdirectory(app)
#message(${THIRD_PARTY_LIBS})
