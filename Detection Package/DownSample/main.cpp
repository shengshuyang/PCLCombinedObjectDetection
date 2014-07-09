/* Copyright(C) 2014, Shuyang Sheng */
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "extremeFinder.h"
#include <pcl/io/pcd_io.h>
int main(int argc, char *argv[])
{
  if (argc < 4)
  {
  	std::cout << "Usage: DownSample cloud_name.pcd leaf_size_x leaf_size_y leaf_size_z";
  	return -1;
  }
  typedef pcl::PointXYZRGB PointT;
  pcl::ApproximateVoxelGrid<PointT>::Ptr avg(new pcl::ApproximateVoxelGrid<PointT>);
  pcl::CropBox<PointT>::Ptr crop(new pcl::CropBox<PointT>);
  pcl::PassThrough<PointT>::Ptr pass( new pcl::PassThrough<PointT>);
  pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr middle(new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr output(new pcl::PointCloud<PointT>);
  //pcl::visualization::PCLVisualizer::Ptr vis(new pcl::visualization::PCLVisualizer());
  pcl::PCDReader reader;
  std::string filename = argv[1];
  reader.read(filename, *cloud);

  if (cloud->empty())
  {
    std::cout << "Cannot read point cloud file" << std::endl;
    return -1;
  }

  // get the leaf size in each direction
  double size_x = atof(argv[2]);
  double size_y = atof(argv[3]);
  double size_z = atof(argv[4]);

  avg->setInputCloud(cloud);
  avg->setLeafSize(size_x, size_y, size_z);
  avg->filter(*middle);

  //extremeFinder* ext_finder = new extremeFinder();
  //ext_finder->find(middle);
  //ext_finder->print();

  Eigen::Vector4f min_point(-0.5, -1, -1, 1);
  Eigen::Vector4f max_point(0.4, 2, 1, 1);
  //Eigen::Affine3f transform;

  //transform.matrix() <<  -0.99899,    0, 0.0449239,                0,
  //                       -0.0322377,  0.696449, -0.716882,         0,
  //                       -0.0312872, -0.717606, -0.695746,         0,
  //                        0,         0,         0,         1;

  //crop->setInputCloud(middle);
  ////crop->setTransform(transform);
  //crop->setMin(min_point);
  //crop->setMax(max_point);
  //crop->filter(*output);

  pass->setInputCloud(middle);
  pass->setFilterFieldName ("x");
  pass->setFilterLimits (-0.5, 0.4);
  pass->filter(*output);

  int pos = filename.find_last_of(".pcd");
  std::stringstream ss;
  ss << filename.substr(0,pos-3) << "_downsampled.pcd";

  pcl::PCDWriter writer;
  writer.write(ss.str(), *output);

  //vis->addPointCloud(output, "cloud", 0);
  //vis->addCoordinateSystem(1.0, 0, 0, 0, 0);
  //vis->initCameraParameters();
  //while (!vis->wasStopped ())
  //{
	 // vis->spinOnce (100);
	 // boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  //}
  return 0;
}
