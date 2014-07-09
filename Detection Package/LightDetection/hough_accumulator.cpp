/* Copyright(C) 2014, Shuyang Sheng */
#include "std_include.h"

int hough_accumulation(std::string filename)
{
  pcl::PointCloud<PointT>::Ptr cloud_tilted(new pcl::PointCloud<PointT>());
  pcl::PointIndices::Ptr outliers(new pcl::PointIndices());
  pcl::PointIndices::Ptr shadows(new pcl::PointIndices());

  std::cout << "Start Hough accumulation" << std::endl;

  if (pcl::io::loadPCDFile<PointT>(filename, *cloud_tilted) == -1)
  {
    PCL_ERROR("couldn't read file");
    return -1;
  }

  for (int i = 0; i < cloud_tilted->size() ; i++)
  {
	PointT point = cloud_tilted->at(i);
  	if (point.r == 255) // shadow
  	{
	  shadows->indices.push_back(i);
  	}
	else if (point.b == 255) // outliers i.e. objects
	{
		outliers->indices.push_back(i);
	}
  }

  int x = 0;

  double accumulator[100][100] = { 0 };

  std::vector<Eigen::Vector3f> lights;
  std::vector<double> distances;
  extremeFinder ef;
  ef.find(cloud_tilted);
  double xmin = ef.x_min, xmax = ef.x_max, zmin = ef.z_min, zmax = ef.z_max;
  double ylevel = ef.y_max - 0.2;

  std::cout << "Start Accumulation Process" << std::endl;
  for (int i = 0; i < shadows->indices.size(); i++)
  {
    int idx_shadow = shadows->indices.at(i);
    PointT ps = cloud_tilted->at(idx_shadow);  // point in shadow
    for (int j = 0; j < outliers->indices.size(); j += 1)
    {
      int idx_object = outliers->indices.at(j);
      PointT po = cloud_tilted->at(idx_object);  // point in object

      Eigen::Vector3f vs(ps.x, ps.y, ps.z);
      Eigen::Vector3f vo(po.x, po.y, po.z);

      double alpha = (ylevel - ps.y) / (po.y - ps.y);  // set the z-plane position of light sources as 0.5
      Eigen::Vector3f light = vs + alpha*(vo - vs);  // position of the light source

      double dist = pow(ps.x - po.x, 2) + pow(ps.y - po.y, 2) + pow(ps.z - po.z, 2);

      lights.push_back(light);
      distances.push_back(exp(-100*dist));
    }
  }
  std::cout << "write to an array" << std::endl;
  for (int i = 0; i < lights.size(); i++)
  {
    Eigen::Vector3f light = lights.at(i);
    int xpos = (light[0] - xmin) * 100 / (xmax - xmin);
    int zpos = (light[2] - zmin) * 100 / (zmax - zmin);
    accumulator[std::min(std::max(xpos, 0), 99)][std::min(std::max(zpos, 0), 99)] += distances.at(i);
  }

  double count_min = INTMAX_MAX;
  double count_max = 0;
  for (int i = 1; i < 99 ; i++)
  {
  	for (int j = 1; j < 99 ; j++)
  	{
  		if (accumulator[i][j] > count_max)	count_max = accumulator[i][j];
		if (accumulator[i][j] < count_min)	count_min = accumulator[i][j];
  	}
  }

  for (int i = 1; i < 99; i++)
  {
    for (int j = 1; j < 99; j++)
    {
      double ipos = (i - 0) * (xmax - xmin) / 100 + xmin;
      double jpos = (j - 0) * (zmax - zmin) / 100 + zmin;
      pcl::PointXYZRGBA temp_point;
      temp_point.x = ipos;
      temp_point.y = ylevel;
      temp_point.z = jpos;
      double color = (accumulator[i][j] - count_min)*255*255*255/(count_max - count_min);
	  //temp_point.rgb = ((int)r) << 16 | ((int)0) << 8 | ((int)50);
	  temp_point.rgb = color;
      cloud_tilted->push_back(temp_point);
    }
  }

  pcl::PCDWriter pcd_writer;
  //pcd_writer.write("light_map.pcd", *cloud_tilted);
  int pos = filename.find_last_of(".pcd");
  std::stringstream os;
  os << filename.substr(0,pos-3) << "_hit_map.pcd";
  pcd_writer.write(os.str(),*cloud_tilted);

  std::ofstream writer("accumulator.txt");
  writer << "a = [";
  for (int i = 0; i < 100; i++)
  {
    for (int j = 0; j < 100; j++)
    {
      writer << (accumulator[i][j] - count_min)*255/(count_max - count_min) << " ";
    }
    if (i != 99)
    {
      writer << ";" << std::endl;
    }
  }
  writer << "];";
  writer.close();

  std::cout << "Finished!" << std::endl;
  return 0;
}
