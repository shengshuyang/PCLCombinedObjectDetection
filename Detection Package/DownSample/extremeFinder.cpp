/* Copyright(C) 2014, Shuyang Sheng */

#include "extremeFinder.h"

int extremeFinder::find(pcl::PointCloud<PointT>::Ptr cloud)
{
  if (cloud->empty() || cloud->size() == 0)
  {
    std::cout << "Cloud empty!" << std::endl;
    return -1;
  }

  for (int i = 0; i < cloud->size(); i++)
  {
    try {
      PointT p = cloud->at(i);
      if (p.x > x_max) x_max = p.x;
      if (p.x < x_min) x_min = p.x;

      if (p.y > y_max) y_max = p.y;
      if (p.y < y_min) y_min = p.y;

      if (p.z > z_max) z_max = p.z;
      if (p.z < z_min) z_min = p.z;

      x_mean += p.x;
      y_mean += p.y;
      z_mean += p.z;
    }
    catch (const std::out_of_range& err) {
      std::cerr << "Out of Range error: " << err.what() 
                << ". Iteration number: " << i << std::endl;
    }
  }

  x_mean /= cloud->size();
  y_mean /= cloud->size();
  z_mean /= cloud->size();
  return 0;
}

int extremeFinder::print()
{
  std::cout << "X from " << x_min << " to " << x_max << std::endl;
  std::cout << "Y from " << y_min << " to " << y_max << std::endl;
  std::cout << "Z from " << z_min << " to " << z_max << std::endl;
  return 0;
}
