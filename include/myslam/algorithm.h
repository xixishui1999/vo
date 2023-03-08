#pragma once
#include "myslam/common_include.h"
namespace myslam{

//为什么用inline?内联函数，解决一些函数频繁调用大量消耗栈空间，inline必须与函数实现放在一起
inline bool triangulation(const std::vector<SE3> &poses,
                const std::vector<Vec3> points, Vec3 &pt_world){
    //std::cout << "triangulation pose size = " << poses.size() << std::endl;//2
    MatXX A(2 * poses.size(), 4);
    VecX b(2 * poses.size());
    b.setZero();
    //std::cout << "b.setZero() = " << std::endl;
    for (size_t i = 0; i < poses.size(); ++i) {
        Mat34 m = poses[i].matrix3x4();
        //std::cout << "poses[i].matrix3x4() = " << std::endl;
        //std::cout << m << std::endl;
        //std::cout << "points[i][0] = " << std::endl;
        //std::cout << points[i][0] << std::endl;
        //std::cout << "points[i][1] = " << std::endl;
        //std::cout << points[i][1] << std::endl;
        //std::cout << m.row(2) << std::endl;// 0 0 1 0
        A.block<1, 4>(2 * i, 0) = points[i][0] * m.row(2) - m.row(0);
        A.block<1, 4>(2 * i + 1, 0) = points[i][1] * m.row(2) - m.row(1);
    }
    auto svd = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
    pt_world = (svd.matrixV().col(3) / svd.matrixV()(3, 3)).head<3>();

    if (svd.singularValues()[3] / svd.singularValues()[2] < 1e-2) {
        // 解质量不好，放弃
        return true;
    }
    return false;
}

inline Vec2 toVec2(const cv::Point2f p) { return Vec2(p.x, p.y); }

}
/*
这段代码是一个用C++实现的三角剖分函数。三角剖分是一种找出一组覆盖给定点集的三角形的过程，
使得没有点在任何三角形的外接圆内1。三角剖分在计算机图形学、计算几何和计算机视觉中有很多应用。
这个函数有四个参数：一个姿态向量，它们是由SE3对象表示的3D刚体变换；
一个点向量，它们是由Vec3对象表示的3D坐标；以及一个叫做pt_world的Vec3对象的引用，它将存储输出。
这个函数创建了一个矩阵A和一个向量b，它们将用于求解一个线性方程组。矩阵A对于每一对姿态-点有两行，有四列。
向量b被初始化为零。函数然后遍历每一对姿态-点，并用由姿态矩阵和点坐标导出的系数填充A的相应行。
这基于一个原理，即由姿态投影的点应该落在其图像平面上。函数然后对A进行奇异值分解（SVD），以找到其最小二乘解。
解被存储在pt_world中，作为V的最后一列除以其最后一个元素的前三个元素。这相当于找到由A定义的四个平面的交点。
函数然后检查最小奇异值与第二小奇异值之比是否小于1e-2。如果是，则返回true，表示解质量不好，应该丢弃。
否则，返回false，表示解是有效的。

代码中的A矩阵是4*4的，是因为每一对姿态-点对应于A的两行，而有两对姿态-点，所以共有四行。
A的列数是由世界坐标系下的点位姿的维度决定的，即三维空间中的一个点加上一个齐次坐标，所以共有四列。

b是一个四维的向量，它被初始化为零。它表示线性方程组的右边，即Ax=b。b的每个元素对应于A的一行，
表示由姿态投影的点在其图像平面上的坐标。因为这些坐标都是零，所以b也是零。

这两行代码是用来填充A矩阵的相应行的。A.block<1, 4>(2 * i, 0)表示A矩阵的第2i行，
从第0列开始的一个1行4列的子矩阵。points[i][0]表示第i个点在左相机坐标系下的x坐标。
m是左相机投影矩阵，它有三行，分别表示x轴、y轴和z轴。m.row(2)表示m矩阵的第三行，即z轴。
m.row(0)表示m矩阵的第一行，即x轴。
这一行代码的意思是，将points[i][0]乘以m.row(2)，然后减去m.row(0)，得到一个1*4的向量，
然后赋值给A.block<1, 4>(2 * i, 0)。
这相当于将由姿态投影的点在其图像平面上的x坐标与世界坐标系下的点位姿之间建立一个线性关系。
同理，A.block<1, 4>(2 * i + 1, 0)表示A矩阵的第2i+1行，从第0列开始的一个14的子矩阵。
points[i][1]表示第i个点在左相机坐标系下的y坐标。
这一行代码是用来将由姿态投影的点在其图像平面上的y坐标与世界坐标系下的点位姿之间建立一个线性关系。

根据我从网络上搜索到的信息1，三角化是利用三角形的关系来恢复三维点的深度信息的。
如果您已经知道了两个视线方向和相对姿态，那么您可以用一个方程来表示两个视线的交点，
但是这个方程是非线性的，不容易求解。

为了简化求解过程，您可以先假设一个初始深度值（比如1），然后将这个值代入方程中，
得到一个近似的交点位置。然后您可以根据交点位置和视线方向来计算一个新的深度值，
并重复这个过程，直到收敛或达到最大迭代次数。

这样做的好处是可以避免求解非线性方程，提高计算效率和精度。😊
*/

/*
您想要看一些关于使用SVD分解的方法来进行三角化深度估计的C++代码示例。👀
#include <Eigen/Dense>
#include <Eigen/SVD>

// a function to triangulate a point from two camera poses and image coordinates
Eigen::Vector3f triangulate_point(const Eigen::Matrix4f& pose1, const Eigen::Matrix4f& pose2,
                                  const Eigen::Vector2f& point1, const Eigen::Vector2f& point2) {
    // get the camera projection matrices from the poses
    Eigen::Matrix3x4f P1 = pose1.block<3, 4>(0, 0);
    Eigen::Matrix3x4f P2 = pose2.block<3, 4>(0, 0);

    // construct a linear system of equations for the homogeneous coordinates of the point
    Eigen::Matrix<float, 4, 4> A;
    A.row(0) = point1.x() * P1.row(2) - P1.row(0);
    A.row(1) = point1.y() * P1.row(2) - P1.row(1);
    A.row(2) = point2.x() * P2.row(2) - P2.row(0);
    A.row(3) = point2.y() * P2.row(2) - P2.row(1);

    // solve the system using singular value decomposition
    Eigen::JacobiSVD<Eigen::Matrix<float, 4, 4>> svd(A, Eigen::ComputeFullV);
    Eigen::Vector4f X = svd.matrixV().col(3);

    // normalize the homogeneous coordinates
    X /= X.w();

    // return the Euclidean coordinates
    return X.head<3>();
}

Eigen::Matrix<float, 4, 4> A;
A.row(0) = point1.x() * P1.row(2) - P1.row(0);
A.row(1) = point1.y() * P1.row(2) - P1.row(1);
A.row(2) = point2.x() * P2.row(2) - P2.row(0);
A.row(3) = point2.y() * P2.row(2) - P2.row(1);
根据我从网络上搜索到的信息1，这四行是在构造一个线性方程组，用于求解点的齐次坐标。
这个线性方程组是基于三角化方法的原理，即两个相机视线的交点就是空间中点的位置。
通过将两个相机投影矩阵和图像坐标代入方程，可以得到一个四元一次方程组，然后用奇异值分解（SVD）来求解。

具体来说，这四行是在用两个相机投影矩阵和图像坐标来构造一个4×4的系数矩阵A。
每一行都是由两个相机投影矩阵的某一行和图像坐标的某一分量相乘再相减得到。
例如，第一行是由第一个相机投影矩阵的第三行和第一个图像坐标的x分量相乘再减去第一个相机投影矩阵的第一行得到。
*/