/*
 * cluster_impl.cpp
 *
 *  Created on: May 1, 2020
 *      Author: obd2fe
 */

#include "cluster_impl.h"


void cluster_helper(const std::vector<std::vector<float>>& points,std::vector<float>& point,KdTree* &tree,std::vector<int>& cluster,std::vector<bool>& pts_state,float distanceTol)
{
	std::vector<int> nearest_pts_ids = tree->search(point,distanceTol);
	for(auto id : nearest_pts_ids)
	{
		if(!pts_state[id])
		{
			cluster.push_back(id);
			pts_state[id] = true;
			std::vector<float> new_pt = points[id];
			cluster_helper(points,new_pt,tree,cluster,pts_state,distanceTol);

		}
	}

}

std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol,int min,int max)
{

	// TODO: Fill out this function to return list of indices for each cluster

	std::vector<std::vector<int>> clusters;
	std::vector<bool> pts_state (points.size(),false);
	int points_index = 0;

	for(auto point : points)
	{
		if(!pts_state[points_index])
		{
			std::vector<int> cluster;
			// push first point to the cluster
			cluster.push_back(points_index);
			// mark the point as processed
			pts_state[points_index] = true;

			cluster_helper(points,point,tree,cluster,pts_state,distanceTol);
			if(cluster.size() >= min && cluster.size() <= max)
			{
				clusters.push_back(cluster);
			}

		}
		points_index++;
	}
	return clusters;

}

bool xAxisCompare (const std::vector<float> &pt1, const std::vector<float> &pt2)
{
	//	cout << student1.grade() << " " << student2.grade() << endl;
	return pt1[0] < pt2[0];
}

bool yAxisCompare (const std::vector<float> &pt1, const std::vector<float> &pt2)
{
	//	cout << student1.grade() << " " << student2.grade() << endl;
	return pt1[1] < pt2[1];
}

bool zAxisCompare (const std::vector<float> &pt1, const std::vector<float> &pt2)
{
	//	cout << student1.grade() << " " << student2.grade() << endl;
	return pt1[2] < pt2[2];
}

void median_inserter (std::vector<std::vector<float>> &points, KdTree* &tree, int axis)
{
	static int node_id = 0;
	int new_axis = (axis == 1) ? 0 : axis+1 ;
	size_t n = (points.size() % 2 == 0) ? (points.size() / 2) -1 : (points.size() / 2);
	;
	if(axis == 0)
	{
		std::nth_element(points.begin(),points.begin()+n,points.end(),xAxisCompare);
	}
	else if(axis == 1)
	{
		std::nth_element(points.begin(),points.begin()+n,points.end(),yAxisCompare);
	}
	else
	{
		std::nth_element(points.begin(),points.begin()+n,points.end(),zAxisCompare);
	}

	tree->insert(points[n],node_id);
	node_id++;
	if(points.size() > 2)
	{
		std::vector<std::vector<float>> left_vector (points.begin(),points.begin()+n);
		median_inserter(left_vector,tree,new_axis);
		std::vector<std::vector<float>> right_vector (points.begin()+n+1,points.end());
		median_inserter(right_vector,tree,new_axis);

	}
	else if (points.size() == 2)
	{
		tree->insert(points[n+1],node_id);
		node_id++;
	}
}

void normal_inserter (std::vector<std::vector<float>> &points, KdTree* &tree)
{
	 for (int i=0; i<points.size(); i++)
	    	tree->insert(points[i],i);
}
