// Author of FLOAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

#ifndef _LASER_MAPPING_H_
#define _LASER_MAPPING_H_

//PCL lib
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>

//eigen  lib
#include <Eigen/Dense>
#include <Eigen/Geometry>

//c++ lib
#include <string>
#include <math.h>
#include <vector>
#include <list>
#include "common.h"
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <iostream>
#include "lidarFactor.hpp"

class LaserMappingClass 
{
    public:
    	LaserMappingClass();
        void lidarMapping(std::list<Eigen::MatrixXd> &pcCorner,
                          std::list<Eigen::MatrixXd> &pcSurf, std::list<Eigen::Matrix4d> &odom);
};
#endif // _LASER_MAPPING_H_
