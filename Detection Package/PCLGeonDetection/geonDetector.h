/*
 * geonDetector.h
 *
 *  Created on: Jul 10, 2013
 *      Author: Shuyang Sheng
 */

#include "std_include.h"

typedef pcl::PointXYZ PointT;

	class geonDetector
	{

	public:
		void initialize();

		pcl::PointCloud<PointT>::Ptr getInputCloud(){ return cloud;};
		void setInputCloud(pcl::PointCloud<PointT>::Ptr input_cloud);
		void setInputCloud(char *path);
		void preprocess();
		void findNormals();
		void detect(pcl::SacModel geonType);

		//int *geonCount = new int[20];
		int totalCount;

	private:
		pcl::PointCloud<PointT>::Ptr cloud;
		pcl::PointCloud<PointT>::Ptr cloud_filtered;
		pcl::PointCloud<PointT>::Ptr cloud_filtered2;
		pcl::PointCloud<pcl::Normal>::Ptr cloud_normals;
		pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2;
		pcl::ModelCoefficients::Ptr coefficients;

		pcl::PointIndices::Ptr geonIndices;

		pcl::PCDReader reader;
		pcl::PCDWriter writer;

		pcl::PassThrough<PointT> pass;
		pcl::NormalEstimation<PointT, pcl::Normal> ne;
		pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;

		pcl::ExtractIndices<PointT> extract;
		pcl::ExtractIndices<pcl::Normal> extract_normals;
		pcl::search::KdTree<PointT>::Ptr tree;

	};
