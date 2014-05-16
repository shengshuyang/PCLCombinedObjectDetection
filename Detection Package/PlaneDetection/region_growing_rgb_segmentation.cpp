#include <iostream>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <string>
#include <pcl/features/normal_3d.h>
#include <pcl/point_cloud.h>
int
region_growing_rgb_segmentation(std::string filename)
{
	pcl::search::Search <pcl::PointXYZRGB>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGB> > (new pcl::search::KdTree<pcl::PointXYZRGB>);

	pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud <pcl::PointXYZRGB>);
	pcl::PointCloud <pcl::Normal>::Ptr normal(new pcl::PointCloud <pcl::Normal>);
	if ( pcl::io::loadPCDFile <pcl::PointXYZRGB> (filename, *cloud) == -1 )
	{
		std::cout << "Cloud reading failed." << std::endl;
		return (-1);
	}

	pcl::IndicesPtr indices (new std::vector <int>);
	pcl::PassThrough<pcl::PointXYZRGB> pass;
	pass.setInputCloud (cloud);
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (0.0, 1.0);
	pass.filter (*indices);

	//pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
	//ne.setInputCloud (cloud);
	//ne.setSearchMethod(tree);
	//ne.setRadiusSearch (0.03);
	//ne.compute(*normal);

	std::cout <<"Finished normal estimation"<<std::endl;

	pcl::RegionGrowingRGB<pcl::PointXYZRGB,pcl::Normal> reg;
	reg.setInputCloud (cloud);
	reg.setInputNormals(normal);
	reg.setIndices (indices);
	reg.setSearchMethod (tree);
	reg.setDistanceThreshold (10);
	reg.setPointColorThreshold (5);
	reg.setRegionColorThreshold (3);
	reg.setMinClusterSize (200);
	reg.setMaxClusterSize(500);
	reg.setNormalTestFlag(false);

	std::vector <pcl::PointIndices> clusters;
	reg.extract (clusters);

	pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();

	pcl::PCDWriter writer;
	std::stringstream os;
	os << "segmented_" << filename;
	writer.write(os.str(),*colored_cloud);

	pcl::visualization::CloudViewer viewer ("Cluster viewer");
	viewer.showCloud (colored_cloud);
	while (!viewer.wasStopped ())
	{
		boost::this_thread::sleep (boost::posix_time::microseconds (100));
	}

	return (0);
}