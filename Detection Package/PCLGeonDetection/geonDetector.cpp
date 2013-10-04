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
		coefficients = new pcl::ModelCoefficients::Ptr;
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
		this->cloud_normals = new pcl::PointCloud<pcl::Normal>();
		this->tree = new pcl::search::KdTree<PointT>();

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

		  std::string geonPath = geonType + (char)totalCount +".pcd";

		  writer.write (geonPath.c_str(), *cloud_geon, false);


		  //copy cloud_filtered2 to cloud_filtered
		  pcl::copyPointCloud(*cloud_filtered2,*cloud_filtered);
		  pcl::copyPointCloud(*cloud_normals2,*cloud_normals);


		/*
		switch(geonType)
		{
			case pcl::SACMODEL_PLANE:
				seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
				break;

			case pcl::SACMODEL_LINE:
				break;
			case pcl::SACMODEL_CIRCLE2D:
				break;
			case pcl::SACMODEL_CIRCLE3D:
				break;
			case pcl::SACMODEL_SPHERE:
				break;
			case pcl::SACMODEL_CYLINDER:
				break;
			case pcl::SACMODEL_CONE:
				break;
			case pcl::SACMODEL_TORUS:
				break;
			case pcl::SACMODEL_PARALLEL_LINE:
				break;
			case pcl::SACMODEL_PERPENDICULAR_PLANE:
				break;
			case pcl::SACMODEL_PARALLEL_LINES:
				break;
			case pcl::SACMODEL_NORMAL_PLANE:
				break;
			case pcl::SACMODEL_NORMAL_SPHERE:
				break;
			case pcl::SACMODEL_REGISTRATION:
				break;
			case pcl::SACMODEL_REGISTRATION_2D:
				break;
			case pcl::SACMODEL_PARALLEL_PLANE:
				break;
			case pcl::SACMODEL_NORMAL_PARALLEL_PLANE:
				break;
			case pcl::SACMODEL_STICK:
				break;
		}
		*/
	}



