/*
 * ransac_impl.h
 *
 *  Created on: Apr 27, 2020
 *      Author: obd2fe
 */

#ifndef SRC_QUIZ_RANSAC_RANSAC_IMPL_H_
#define SRC_QUIZ_RANSAC_RANSAC_IMPL_H_


#include "../../render/render.h"
#include <random>
#include <cmath>
#include <ctime>
#include <cstdlib>
#include "../../processPointClouds.h"

std::vector<int> RansacPlane(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, int maxIterations, float distanceTol);


#endif /* SRC_QUIZ_RANSAC_RANSAC_IMPL_H_ */
