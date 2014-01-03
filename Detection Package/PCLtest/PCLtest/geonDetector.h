/*
 * geonDetector.h
 *
 *  Created on: Jul 10, 2013
 *      Author: Shuyang Sheng
 */

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl\io\pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/pcl_visualizer.h>

typedef pcl::PointXYZRGBA PointT;

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
		void rotate_to_horizon(pcl::PointCloud<PointT>::Ptr output);
		//int *geonCount = new int[20];
		int totalCount;

	public:
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
