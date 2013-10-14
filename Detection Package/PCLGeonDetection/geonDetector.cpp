/*
 * geonDetector.cpp
 *
 *  Created on: Jul 10, 2013
 *      Author: Shuyang Sheng
 */

#include "std_include.h"
#include "geonDetector.h"

	void geonDetector::initialize()
	{
		totalCount = 0;
		pcl::PointCloud<PointT>::Ptr cloud1                       (new pcl::PointCloud<PointT>());
		pcl::PointCloud<PointT>::Ptr cloud_filtered1           (new pcl::PointCloud<PointT>());
		pcl::PointCloud<PointT>::Ptr cloud_filtered21         (new pcl::PointCloud<PointT>());
		pcl::PointCloud<pcl::Normal>::Ptr cloud_normals1  (new pcl::PointCloud<pcl::Normal>());
		pcl::PointCloud<pcl::Normal>::Ptr cloud_normals21 (new pcl::PointCloud<pcl::Normal>());
		pcl::ModelCoefficients::Ptr coefficients1                   ( new pcl::ModelCoefficients());
		pcl::PointIndices::Ptr geonIndices1                           (new pcl::PointIndices());
		pcl::search::KdTree<PointT>::Ptr  tree2                     ( new pcl::search::KdTree<PointT>());


		this->cloud                        = cloud1;
		this->cloud_filtered            = cloud_filtered1;
		this->cloud_normals          = cloud_normals1;		
		this->cloud_filtered2          = cloud_filtered21;
		this->cloud_normals2        = cloud_normals21;
		this->coefficients              = coefficients1;
		this->geonIndices             = geonIndices1;
		this->tree                          =tree2;
		
	}
	void geonDetector::setInputCloud(pcl::PointCloud<PointT>::Ptr input_cloud)
	{
		this->cloud = input_cloud;
		  std::cerr << "PointCloud has: " << cloud->points.size () << " data points." << std::endl;
	}
	void geonDetector::setInputCloud(char *path)
	{
		  this->reader.read (path, *(this->cloud));
		  std::cerr << "PointCloud has: " << cloud->points.size () << " data points." << std::endl;
	}
	void geonDetector::preprocess()
	{
		//this->cloud_normals = new pcl::PointCloud<pcl::Normal>();
		//this->tree = new pcl::search::KdTree<PointT>();

		  // Build a pass through filter to remove spurious NaNs
		  pass.setInputCloud (cloud);
		  pass.setFilterFieldName ("z");
		  pass.setFilterLimits (0, 1.5);
		  pass.filter (*cloud_filtered);
		  std::cerr << "PointCloud after filtering has: " << cloud_filtered->points.size () << " data points." << std::endl;

	}

	void geonDetector::findNormals()
	{
		  ne.setSearchMethod (tree);
		  ne.setInputCloud (cloud_filtered);
		  ne.setKSearch (50);
		  ne.compute (*cloud_normals);
	}
	
std::string findTypeName( pcl::SacModel geonType)
{
	std::string str;
		switch(geonType)
		{
			case pcl::SACMODEL_PLANE:
				str = "SACMODEL_PLANE";
				break;

			case pcl::SACMODEL_LINE:
			str = "SACMODEL_LINE";
				break;
			case pcl::SACMODEL_CIRCLE2D:
			str = "SACMODEL_CIRCLE2D";
				break;
			case pcl::SACMODEL_CIRCLE3D:
			str = "SACMODEL_CIRCLE3D";
				break;
			case pcl::SACMODEL_SPHERE:
			str = "SACMODEL_SPHERE";
				break;
			case pcl::SACMODEL_CYLINDER:
			str = "SACMODEL_CYLINDER";
				break;
			case pcl::SACMODEL_CONE:
			str = "SACMODEL_CONE";
				break;
			case pcl::SACMODEL_TORUS:
			str = "SACMODEL_TORUS";
				break;
			case pcl::SACMODEL_PARALLEL_LINE:
			str = "SACMODEL_PARALLEL_LINE";
				break;
			case pcl::SACMODEL_PERPENDICULAR_PLANE:
			str = "SACMODEL_PERPENDICULAR_PLANE";
				break;
			case pcl::SACMODEL_PARALLEL_LINES:
			str = "SACMODEL_PARALLEL_LINES";
				break;
			case pcl::SACMODEL_NORMAL_PLANE:
			str = "SACMODEL_NORMAL_PLANE";
				break;
			case pcl::SACMODEL_NORMAL_SPHERE:
			str = "SACMODEL_NORMAL_SPHERE";
				break;
			case pcl::SACMODEL_REGISTRATION:
			str = "SACMODEL_REGISTRATION";
				break;
			case pcl::SACMODEL_REGISTRATION_2D:
			str = "SACMODEL_REGISTRATION_2D";
				break;
			case pcl::SACMODEL_PARALLEL_PLANE:
			str = "SACMODEL_PARALLEL_PLANE";
				break;
			case pcl::SACMODEL_NORMAL_PARALLEL_PLANE:
			str = "SACMODEL_NORMAL_PARALLEL_PLANE";
				break;
			case pcl::SACMODEL_STICK:
			str = "SACMODEL_STICK";
				break;
		}
		
		return str;
}
	
	
	
	
	
	
	void geonDetector::detect(pcl::SacModel geonType)
	{

		  seg.setOptimizeCoefficients (true);
		  seg.setNormalDistanceWeight (0.1);
		  seg.setMethodType (pcl::SAC_RANSAC);
		  seg.setMaxIterations (100);
		  seg.setDistanceThreshold (0.03);
		  seg.setInputCloud (cloud_filtered);
		  seg.setInputNormals (cloud_normals);
		  seg.setModelType (geonType);

		  //perform the segmentation, aka detection
		  seg.segment (*geonIndices, *coefficients);
		  std::cerr << geonType << "coefficients: " << *coefficients << std::endl;

		  //if no geon is found, return directly.
		  if(geonIndices->indices.size()==0)
		  {
			  return;
		  }

		  //increase total number of found geons.
		  this->totalCount++;

		  // Remove the inlier, extract the rest
		  extract.setInputCloud(cloud_filtered);
		  extract.setIndices(geonIndices);
		  extract.setNegative (true);
		  extract.filter (*cloud_filtered2);

		  extract_normals.setInputCloud(cloud_normals);
		  extract_normals.setIndices(geonIndices);
		  extract_normals.setNegative (true);
		  extract_normals.filter (*cloud_normals2);


		  // Write the inliers to disk
		  pcl::PointCloud<PointT>::Ptr cloud_geon(new pcl::PointCloud<PointT> ());
		  extract.setNegative(false);
		  extract.filter (*cloud_geon);
		  std::cerr << "PointCloud representing the planar component: " << cloud_geon->points.size () << " data points." << std::endl;

		  std::stringstream geonPath;
		  geonPath << findTypeName(geonType) << "_" << totalCount  << ".pcd";

		  writer.write (geonPath.str().c_str(), *cloud_geon, false);
		 // writer.write ("geon.pcd", *cloud_geon, false);

		  //copy cloud_filtered2 to cloud_filtered
		  pcl::copyPointCloud(*cloud_filtered2,*cloud_filtered);
		  pcl::copyPointCloud(*cloud_normals2,*cloud_normals);



	}



