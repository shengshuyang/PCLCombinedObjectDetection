#include <pcl\common\eigen.h>
#include <pcl/point_types.h>
#include <pcl\io\io.h>
#include "extremeFinder.h"
#include <ostream>
#include <math.h>
typedef pcl::PointXYZRGBA PointT;

int intersection_test(Eigen::Vector3f *start, Eigen::Vector3f *direction, pcl::PointIndices::Ptr output, pcl::PointCloud<PointT>::Ptr cloud, pcl::PointIndices::Ptr mask, double radius)
{
	direction->normalize();

	int max_index = cloud->size()-1;
	for( int j = 0; j<mask->indices.size();j++)
	{
		int ii = mask->indices.at(j);
		int i = std::min( ii ,max_index);
		Eigen::Vector3f *test_point = new Eigen::Vector3f(cloud->at(i).x,cloud->at(i).y,cloud->at(i).z);
		Eigen::Vector3f test_diff = *test_point - *start;
		if( test_diff.dot(*direction) < 0)//test if the dot product is on the positive side of main axis
		{
			continue;
		}
		double dist_squared = test_diff.dot(test_diff) - (test_diff.dot(*direction))*(test_diff.dot(*direction));
		if( dist_squared < radius*radius)
		{
			output->indices.push_back(i);
		}

	}
	if( output->indices.size() == 0) return(0);

	//pcl::PointCloud<PointT>::Ptr output(new pcl::PointCloud<PointT>);
	//pcl::copyPointCloud<PointT> (*cloud,output->indices,*output);

	//pcl::io::savePCDFile("output.pcd",*output);

	//std::cout<< "intersection test finished" <<std::endl;

	return (1);
}