#include "region_growing_hsv.h"

#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <boost/smart_ptr/shared_ptr.hpp>
#include <vector>
#include <pcl/PointIndices.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <istream>
//#include <svm.h>

int main(int argc, char *argv[])
{


	pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZHSV>);
	pcl::PointCloud<pcl::Normal>::Ptr normal(new pcl::PointCloud<pcl::Normal>);
	pcl::search::Search<pcl::PointXYZHSV>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZHSV> >(new pcl::search::KdTree<pcl::PointXYZHSV>);

	//read plane indices
	std::ifstream input;
	input.open("inlier_indices.txt");
	if(!input.is_open())
	{
		std::cout<< "cannot open inlier indices"<<std::endl;
	}
	std::string str;

	//note that plane indices is the same as inlier indices, which meanas all points inside the detected plane
	pcl::PointIndices::Ptr plane_indices(new pcl::PointIndices);

	while(!input.eof())
	{
		std::getline(input,str);
		//std::cout << str<<std::endl;
		if( !str.empty())
		{
			int x = std::stoi(str);
			plane_indices->indices.push_back(x);
		}
	}

	std::vector<int> plane_labels;

	//1 for plane and 0 for not in plane
	plane_labels.push_back(1);
	for (int i=1; i < plane_indices->indices.size(); i++)
	{
		int a = plane_indices->indices.at(i-1)+1;
		int b = plane_indices->indices.at(i);
		for( int j = a; j<b;j++)
		{
			plane_labels.push_back(0);
		}
		plane_labels.push_back(1);
	}


	std::cout << "File Reader started" << std::endl;
	pcl::PCDReader reader;
 
	std::string filename = argv[1];
	reader.read(filename, *cloud);


	pcl::IndicesPtr indices(new std::vector <int>);
	pcl::PassThrough<pcl::PointXYZHSV> pass;
	pass.setInputCloud(cloud);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(-5.0, 5.0);
	pass.filter(*indices);

	pcl::RegionGrowingHSV<pcl::PointXYZHSV, pcl::Normal> reg;
	reg.setInputCloud(cloud);
	reg.setSearchMethod(tree);
	reg.setIndices(indices);
	reg.setDistanceThreshold(10);

	reg.setPointColorThreshold (-10, -0.3, -0.3, 10, 0.3, 0.3);
	reg.setRegionColorThreshold(-10, -0.3, -0.3, 10, 0.3, 0.3);


	reg.setMinClusterSize(50);
	//reg.setMaxClusterSize(1000);
	reg.setNumberOfRegionNeighbours(5);
	reg.setNumberOfNeighbours(20);

	//reg.setInputNormals(normal);
	reg.setNormalTestFlag(false);
	reg.setCurvatureTestFlag(false);
	reg.setResidualTestFlag(false);
	

	std::cout << "Start Extract Clusters!" << std::endl;
	std::vector<pcl::PointIndices> clusters;// (new std::vector<pcl::PointIndices>());
	reg.extract(clusters);


	//pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();
	//pcl::PCDWriter writer;
	//writer.write(filename.insert(0,"segmented_"), *colored_cloud);

	//reg.writeClustersToFile("clusters_statistics.txt", plane_labels);

	reg.readShadowLabelsFromFile("labels.txt");
	pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud2 = reg.getShadowCloud();
	pcl::PCDWriter writer;
	std::stringstream os2;
	os2 << "shadow_detected_" << filename;
	writer.write(os2.str(), *colored_cloud2);


	return(0);
}