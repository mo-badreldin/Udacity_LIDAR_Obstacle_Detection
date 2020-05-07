/*
 * cluster_impl.h
 *
 *  Created on: May 1, 2020
 *      Author: obd2fe
 */



#ifndef SRC_QUIZ_CLUSTER_CLUSTER_IMPL_H_
#define SRC_QUIZ_CLUSTER_CLUSTER_IMPL_H_


#include <chrono>
#include <string>
#include "kdtree.h"


std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol,int min,int max);
void normal_inserter (std::vector<std::vector<float>> &points, KdTree* &tree);


#endif /* SRC_QUIZ_CLUSTER_CLUSTER_IMPL_H_ */
