// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"


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
pcl::PointIndices::Ptr ProcessPointClouds<PointT>::Ransac(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
	pcl::PointIndices::Ptr inliersResult (new pcl::PointIndices);
	srand(time(NULL));

	// TODO: Fill in this function

	int bestInliersCount = 0;

	// For max iterations
	for(int i = 0; i < maxIterations; i++)
	{
		double A, B, C, D;

		pcl::PointIndices::Ptr currInliers (new pcl::PointIndices);
		// Randomly sample subset and fit line
		int index1 = rand() % ( cloud->size() - 1 );
		int index2 = rand() % ( cloud->size() - 1 );
		int index3 = rand() % ( cloud->size() - 1 );

		PointT Pt1,Pt2,Pt3;

		Pt1 = cloud->points[index1];
		Pt2 = cloud->points[index2];
		Pt3 = cloud->points[index3];

		A = ( (Pt2.y - Pt1.y)*(Pt3.z - Pt1.z) - (Pt2.z - Pt1.z)*(Pt3.y - Pt1.y) );
		B = ( (Pt2.z - Pt1.z)*(Pt3.x - Pt1.x) - (Pt2.x - Pt1.x)*(Pt3.z - Pt1.z) );
		C = ( (Pt2.x - Pt1.x)*(Pt3.y - Pt1.y) - (Pt2.y - Pt1.y)*(Pt3.x - Pt1.x) );
		D = -( ( A * Pt1.x ) + ( B * Pt1.y ) + ( C * Pt1.z ) );


		// Measure distance between every point and fitted line
		for(int j = 0; j < cloud->size(); j++)
		{
			double Dist;
			PointT currPt;
			// If distance is smaller than threshold count it as inlier
			currPt = cloud->points[j];


			Dist = std::abs( ( A * currPt.x ) + ( B * currPt.y ) + ( C * currPt.z ) + D );
			Dist = Dist / sqrt( ( ( A*A ) + ( B*B ) + (C*C) ) );

			if(Dist < distanceTol)
			{
				currInliers->indices.push_back(j);
			}
		}

		if(currInliers->indices.size() > bestInliersCount )
		{
			bestInliersCount = currInliers->indices.size();
			inliersResult->indices = currInliers->indices;
		}

		currInliers->indices.clear();

	}

	// Return indicies of inliers from fitted line with most inliers
	return inliersResult;

}

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering

    // Voxel Grid Filtering
    typename pcl::PointCloud<PointT>::Ptr cloudVox (new pcl::PointCloud<PointT>);
    pcl::VoxelGrid<PointT> Vox;
    Vox.setInputCloud (cloud);
    Vox.setLeafSize (filterRes, filterRes, filterRes);
    Vox.filter (*cloudVox);

    // Region of Interest cropping
    typename pcl::PointCloud<PointT>::Ptr cloudROI (new pcl::PointCloud<PointT>);
    pcl::CropBox<PointT> ROI(true);
    ROI.setMin(minPoint);
    ROI.setMax(maxPoint);
    ROI.setInputCloud(cloudVox);
    ROI.filter(*cloudROI);

    std::vector<int> indices;

    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f (-1.5,-1.7,-1,1));
    roof.setMax(Eigen::Vector4f ( 2.6, 1.7,-0.4,1));
    roof.setInputCloud(cloudROI);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    for(int point : indices)
    {
    	inliers->indices.push_back(point);
    }

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloudROI);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloudROI);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudROI;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane

	typename pcl::PointCloud<PointT>::Ptr obstCloud (new pcl::PointCloud<PointT>());
	typename pcl::PointCloud<PointT>::Ptr planeCloud (new  pcl::PointCloud<PointT>());

	for(int index : inliers->indices)
	{
		planeCloud->points.push_back(cloud->points[index]);
	}

	pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*obstCloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud,planeCloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

	pcl::PointIndices::Ptr inliers;
    // TODO:: Fill in this function to find inliers for the cloud.

	/*
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
	// Create the segmentation object
	pcl::SACSegmentation<PointT> seg;
	seg.setOptimizeCoefficients (true);
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setMaxIterations (maxIterations);
	seg.setDistanceThreshold (distanceThreshold);

    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);
	*/
	inliers = Ransac(cloud, maxIterations, distanceThreshold);

    if (inliers->indices.size () == 0)
    {
    	std::cout<<"Could not estimate a planar model for the given dataset"<<std::endl;
    }
	auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}

template<typename PointT>
void ProcessPointClouds<PointT>::Proximity(int pointID,const typename pcl::PointCloud<PointT>::Ptr& cloud,std::vector<bool>& processed,std::vector<int>& cluster, KdTree<PointT>* tree, float distanceTol)
{
	processed[pointID] = true;
	cluster.push_back(pointID);
	std::vector<int> nearby = tree->search(cloud->points[pointID],distanceTol);

	for(int i = 0; i < nearby.size(); i++)
	{
		int currPtID = nearby[i];
		if(processed[currPtID] == false)
		{
			Proximity(currPtID,cloud,processed,cluster, tree,distanceTol);
		}
	}
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    KdTree<PointT>* tree = new KdTree<PointT>();

    for (int i=0; i<cloud->points.size(); i++)
    	tree->insert(cloud->points[i],i);

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles

	std::vector<bool> processed(cloud->points.size(), false);


	for (int i=0; i<cloud->points.size();i++)
	{
		std::vector<int> clusterIds;
		if(processed[i] == false)
		{
			typename pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>());
			Proximity(i,cloud,processed,clusterIds,tree,clusterTolerance);

			if(clusterIds.size() >= minSize && clusterIds.size() <= maxSize)
			{
	            for (int i = 0; i < clusterIds.size(); ++i)
	            {
	                cluster->points.push_back(cloud->points[clusterIds[i]]);
	            }

	            clusters.push_back(cluster);
			}
		}
	}

    // Creating the KdTree object for the search method of the extraction
    /*
    typename pcl::search::KdTree<PointT>::Ptr tree (new typename pcl::search::KdTree<PointT>);
    tree->setInputCloud (cloud);

    std::vector<pcl::PointIndices> cluster_indices;



    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (clusterTolerance);
    ec.setMinClusterSize (minSize);
    ec.setMaxClusterSize (maxSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);

    for (const auto& cluster : cluster_indices)
    {
    	typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new typename pcl::PointCloud<PointT>);

      for (const auto& idx : cluster.indices)
      {
        cloud_cluster->push_back((*cloud)[idx]);
      }

      cloud_cluster->width = cloud_cluster->size ();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;

      clusters.push_back(cloud_cluster);

    }
	*/

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
