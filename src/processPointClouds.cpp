// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include "quiz/ransac/ransac_impl.h"
#include "quiz/cluster/cluster_impl.h"


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    typename pcl::PointCloud<PointT>::Ptr voxel_cloud (new pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr cropbox_cloud (new pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr roof_cloud (new pcl::PointCloud<PointT>);

    pcl::VoxelGrid<PointT> fil_obj;
    fil_obj.setInputCloud(cloud);
    fil_obj.setLeafSize(filterRes,filterRes,filterRes);
    fil_obj.filter(*voxel_cloud);

    pcl::CropBox<PointT> crp_box_obj(true);
    crp_box_obj.setMin(minPoint);
    crp_box_obj.setMax(maxPoint);
    crp_box_obj.setInputCloud(voxel_cloud);
    crp_box_obj.filter(*cropbox_cloud);

    std::vector<int> indices;
    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f (-1.5,-1.7,-1.0,1.0));
    roof.setMax(Eigen::Vector4f (2.6,1.7,-0.4,1.0));
    roof.setInputCloud(cropbox_cloud);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    for(auto pt : indices)
    {
    	inliers->indices.push_back(pt);
    }
    pcl::ExtractIndices<PointT> extractor;
    extractor.setInputCloud(cropbox_cloud);
    extractor.setIndices(inliers);
    extractor.setNegative(true);
    extractor.filter(*roof_cloud);


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return roof_cloud;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane

	typename pcl::PointCloud<PointT>::Ptr obstable_cloud (new pcl::PointCloud<PointT>);
	typename pcl::PointCloud<PointT>::Ptr plane_cloud (new pcl::PointCloud<PointT>);
	pcl::ExtractIndices<PointT> extractor;
	extractor.setInputCloud(cloud);
	extractor.setIndices(inliers);
	extractor.setNegative(false);
	extractor.filter(*plane_cloud);
	extractor.setNegative(true);
	extractor.filter(*obstable_cloud);


    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstable_cloud, plane_cloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlanePCL(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices());
	pcl::ModelCoefficients::Ptr coef ( new pcl::ModelCoefficients());
    // TODO:: Fill in this function to find inliers for the cloud.

	pcl::SACSegmentation<PointT> seg;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(distanceThreshold);
	seg.setMaxIterations(maxIterations);
	seg.setInputCloud(cloud);
	seg.segment(*inliers,*coef);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation PCL LIb took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

	std::vector<int> inliers_vec = RansacPlane(cloud, maxIterations, distanceThreshold);


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    pcl::PointIndices::Ptr inliers (new pcl::PointIndices());
    inliers->indices = inliers_vec;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::ClusteringPCL(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    typename pcl::search::KdTree<PointT>::Ptr tree_ptr (new pcl::search::KdTree<PointT>);
    tree_ptr->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indicies;
    pcl::EuclideanClusterExtraction<PointT> euc_cluster;
    euc_cluster.setClusterTolerance(clusterTolerance);
    euc_cluster.setMinClusterSize(minSize);
    euc_cluster.setMaxClusterSize(maxSize);
    euc_cluster.setSearchMethod(tree_ptr);
    euc_cluster.setInputCloud(cloud);
    euc_cluster.extract(cluster_indicies);

    for(pcl::PointIndices clusterindexes : cluster_indicies)
    {
    	typename pcl::PointCloud<PointT>::Ptr cluster_cloud (new pcl::PointCloud<PointT>);
    	for(int idx : clusterindexes.indices)
    	{
    		cluster_cloud->points.push_back(cloud->points[idx]);
    	}
    	cluster_cloud->width = cluster_cloud->points.size();
    	cluster_cloud->height = 1;
    	cluster_cloud->is_dense = true;
    	clusters.push_back(cluster_cloud);;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering PCL took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud
																						  , float clusterTolerance, int minSize, int maxSize)
{

	// prepare data to cluster implementation format
	std::vector<std::vector<float>> points;
	for(auto pt : cloud->points)
	{
		points.push_back({pt.x,pt.y,pt.z});
	}

	auto startTime = std::chrono::steady_clock::now();

	std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
	KdTree* tree = new KdTree;

	normal_inserter(points,tree);

	std::vector<std::vector<int>> clusters_indices = euclideanCluster(points, tree, clusterTolerance,minSize,maxSize);

    for(auto clusterindexes : clusters_indices)
    {
    	typename pcl::PointCloud<PointT>::Ptr cluster_cloud (new pcl::PointCloud<PointT>);
    	for(auto idx : clusterindexes)
    	{
    		cluster_cloud->points.push_back(cloud->points[idx]);
    	}
    	cluster_cloud->width = cluster_cloud->points.size();
    	cluster_cloud->height = 1;
    	cluster_cloud->is_dense = true;
    	clusters.push_back(cluster_cloud);;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;
    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}
