/* Copyright(C) 2014, Shuyang Sheng */
#include <math.h>
#include "includes.h"

// 1. Decompose the input surface into a smooth base and a height vector field

int pcDenoise::pcDecompose()
{
  if (cloud->empty())
  {
      std::cout << "The input point cloud seems to be empty.";
      return -1;
  }

  pcType temp_cloud(new pcl::PointCloud<PT>);

    int K = 100;
    pcl::KdTreeFLANN<PT> kdtree;
    kdtree.setInputCloud(cloud);

    std::vector<int> Idx(K);
    std::vector<float> Dist(K);
    double mx = 0;
    double my = 0;
    double mz = 0;

    for (int i = 0; i < cloud->width; i++ )
    {
      mx += (cloud->points.at(i).x);
      my += (cloud->points.at(i).y);
      mz += (cloud->points.at(i).z);
    }
    mx /= cloud->width;
    my /= cloud->width;
    mz /= cloud->width;

    for (int i = 0; i < cloud->width; i++ )
    {
      kdtree.nearestKSearch(i, K, Idx, Dist);
      int neighbourIdx = 0;
      double xx = 0;
      double yy = 0;
      double zz = 0;
      for (int j = 0; j < K; j++)
      {
        neighbourIdx = Idx[j];
        xx += (cloud->points.at(neighbourIdx).x);
        yy += (cloud->points.at(neighbourIdx).y);
        zz += (cloud->points.at(neighbourIdx).z);
      }
      xx /= K;
      yy /= K;
      zz /= K;

      xx -= mx;
      yy -= my;
      zz -= mz;
      PT* temp = new PT();
      temp->x = xx;
      temp->y = yy;
      temp->z = zz;
      temp->r = cloud->points.at(i).r;
      temp->g = cloud->points.at(i).g;
      temp->b = cloud->points.at(i).b;
      temp_cloud->push_back(*temp);
    }
    cloud->clear();
    pcl::copyPointCloud(*temp_cloud, *cloud);

  return 0;
}

// 2. For each point p compute a local descriptor P (p) encoding the variation of the height vector field around the point (Section 5)
int pcDenoise::findDescriptor()
{
  localDescriptor* des = new localDescriptor();
  return 0;
}

// 3. Compute the denoised height vector for each point based on the similarity between the descriptors (Section 6)

int pcDenoise::denoise()
{
  // manipulate the height vector
  return 0;
}

// 4. Update all point positions by adding the denoised height field to the smooth surface.

int pcDenoise::combine()
{
  return 0;
}
