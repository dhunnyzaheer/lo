// Author of FLOAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

#ifndef _SCAN_REGISTRATION_H_
#define _SCAN_REGISTRATION_H_

#include <cmath>
#include <vector>
#include <string>
#include "common.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "../../utilities/logging/include/aru/core/utilities/logging/log.h"
//#include "/home/zaheer/aru-core/src/utilities/logging/include/aru/core/utilities/logging/log.h"
#include "../../utilities/laser/include/aru/core/utilities/laser/laser.h"
//#include "/home/zaheer/aru-core/src/utilities/laser/include/aru/core/utilities/laser/laser.h"
#include <iostream>
#include <pcl/common/transforms.h>

class scanRegistrationClass 
{
    public:
    	scanRegistrationClass();
		void featureExtraction(aru::core::utilities::laser::Laser laser_new);
};

#endif // _SCAN_REGISTRATION_H_
