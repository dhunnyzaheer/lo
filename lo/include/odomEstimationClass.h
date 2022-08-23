// Author of FLOAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro
#ifndef _ODOM_ESTIMATION_CLASS_H_
#define _ODOM_ESTIMATION_CLASS_H_

//std lib
#include <string>
#include <math.h>
#include <vector>
#include <list>
#include <iostream>

//PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>

//ceres
#include <ceres/ceres.h>
#include <ceres/rotation.h>

//eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <eigen3/Eigen/Dense>

#include "common.h"
#include "lidarFactor.hpp"

class OdomEstimationClass
{
public:
    OdomEstimationClass();

    std::list<Eigen::Matrix4d> list_transforms;
    void lidarOdometry(std::list<Eigen::MatrixXd> &cornerSharpBuf, std::list<Eigen::MatrixXd> &cornerLessSharpBuf,
                       std::list<Eigen::MatrixXd> &surfFlatBuf, std::list<Eigen::MatrixXd> &surfLessFlatBuf);
};

/*struct LidarEdgeFactor
{
    LidarEdgeFactor(Eigen::Vector3d curr_point_, Eigen::Vector3d last_point_a_,Eigen::Vector3d last_point_b_, double s_);
    template <typename T> bool operator()(const T *q, const T *t, T *residual) const;
    static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_, const Eigen::Vector3d last_point_a_,
                                       const Eigen::Vector3d last_point_b_, const double s_){
        return (new ceres::AutoDiffCostFunction<LidarEdgeFactor, 3, 4, 3>(new LidarEdgeFactor(curr_point_, last_point_a_, last_point_b_, s_)));
    }

    Eigen::Vector3d curr_point, last_point_a, last_point_b;
    double s;
};

struct LidarPlaneFactor
{
    LidarPlaneFactor(Eigen::Vector3d curr_point_, Eigen::Vector3d last_point_j_,
                     Eigen::Vector3d last_point_l_, Eigen::Vector3d last_point_m_, double s_);
    template <typename T> bool operator()(const T *q, const T *t, T *residual) const;
    static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_, const Eigen::Vector3d last_point_j_,
                                       const Eigen::Vector3d last_point_l_, const Eigen::Vector3d last_point_m_,
                                       const double s_){
        return (new ceres::AutoDiffCostFunction<LidarPlaneFactor, 1, 4, 3>(
                new LidarPlaneFactor(curr_point_, last_point_j_, last_point_l_, last_point_m_, s_)));
    }
    Eigen::Vector3d curr_point, last_point_j, last_point_l, last_point_m;
    Eigen::Vector3d ljm_norm;
    double s;
};*/

#endif // _ODOM_ESTIMATION_CLASS_H_

