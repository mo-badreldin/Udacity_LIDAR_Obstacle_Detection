/*
 * ransac2d_impl.cpp
 *
 *  Created on: Apr 27, 2020
 *      Author: obd2fe
 */

// using templates for processPointClouds so also include .cpp to help linker
#include "ransac_impl.h"
#include "../../processPointClouds.cpp"


void crossProduct(std::vector<float>& v_A, std::vector<float>& v_B, std::vector<float>& c_P)
{
	c_P[0] = v_A[1] * v_B[2] - v_A[2] * v_B[1];
	c_P[1] = -(v_A[0] * v_B[2] - v_A[2] * v_B[0]);
	c_P[2] = v_A[0] * v_B[1] - v_A[1] * v_B[0];
}


std::vector<int> RansacPlane(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, int maxIterations, float distanceTol)
{
	srand(time(NULL));
	std::vector<int> inliersResult;
	std::vector<int> inliersCurrent;
	float A =0;
	float B =0;
	float C =0;
	float D =0;
	uint32_t cloud_width = cloud->width;
	std::size_t points_size = cloud->points.size();

	// For max iterations
	for(int i = 0; i < maxIterations; i++)
	{
		uint32_t idx1 = 0;
		uint32_t idx2 = 0;
		uint32_t idx3 = 0;

		idx1 = std::rand()%cloud_width;
		do
		{
			idx2 = std::rand()%cloud_width;
			idx3 = std::rand()%cloud_width;
		}
		while(idx2 == idx1 || idx2 == idx3 || idx1 == idx3);

		pcl::PointXYZI pt1 = cloud->points[idx1];
		pcl::PointXYZI pt2 = cloud->points[idx2];
		pcl::PointXYZI pt3 = cloud->points[idx3];

		std::vector<float> vec_a = {pt2.x-pt1.x, pt2.y-pt1.y, pt2.z-pt1.z};
		std::vector<float> vec_b = {pt3.x-pt1.x, pt3.y-pt1.y, pt3.z-pt1.z};
		std::vector<float> corss_p = {0,0,0};
		crossProduct(vec_a,vec_b,corss_p);

		// Plane Equation Coefficients
		A = corss_p[0];
		B = corss_p[1];
		C = corss_p[2];
		D = (corss_p[0] * pt1.x + corss_p[1] * pt1.y + corss_p[2] * pt1.z) * -1;

		inliersCurrent.clear();
		for(std::size_t i = 0; i < points_size; i++)
		{
			pcl::PointXYZI pt = cloud->points[i];
			float dist_up = std::fabs (A * pt.x +  B * pt.y + C * pt.z + D);
			float dist_low = std::sqrt( A*A + B*B + C*C )  ;
			float dist = dist_up / dist_low;

			if(dist < distanceTol)
			{
				inliersCurrent.push_back(i);
			}
		}

		if(inliersCurrent.size() > inliersResult.size())
		{
			inliersResult = inliersCurrent;
		}
	}
	return inliersResult;
}
