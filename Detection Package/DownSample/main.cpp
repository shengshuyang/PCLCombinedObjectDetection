#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/io/pcd_io.h>
int main(int argc, char *argv[])
{
	typedef pcl::PointXYZRGB PointT;
	pcl::ApproximateVoxelGrid<PointT>::Ptr avg(new pcl::ApproximateVoxelGrid<PointT>);
	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr output(new pcl::PointCloud<PointT>);
	pcl::PCDReader reader;
	std::string filename = argv[1];
	reader.read(filename, *cloud);
	
	if(cloud->empty())
	{
		std::cout << "Cannot read point cloud file" << std::endl;
		return -1;
	}

	avg->setInputCloud(cloud);
	avg->setLeafSize(0.01,0.01,0.01);
	avg->filter(*output);

	pcl::PCDWriter writer;

	std::stringstream ss;
	ss<<"downsampled_"<<filename;
	writer.write(ss.str(),*output);

	return 0;

}