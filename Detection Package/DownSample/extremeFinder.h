/* Copyright(C) 2014, Shuyang Sheng */

#ifndef PROJECTS_SHADOW_DETECTION_PLANEDETECTION_EXTREMEFINDER_H_
#define PROJECTS_SHADOW_DETECTION_PLANEDETECTION_EXTREMEFINDER_H_

#include <pcl\point_types.h>
#include <pcl\point_cloud.h>

typedef pcl::PointXYZRGB PointT;

class extremeFinder
{
 public:
    double x_min;
    double y_min;
    double z_min;
    double x_max;
    double y_max;
    double z_max;
    double x_mean;
    double y_mean;
    double z_mean;

    extremeFinder(){x_min = 0; y_min = 0; z_min = 0;
                    x_max = 0; y_max = 0; z_max = 0;
                    x_mean = 0; y_mean = 0; z_mean =0;};

    int find(pcl::PointCloud<PointT>::Ptr cloud);
    int print();
};
#endif  // PROJECTS_SHADOW_DETECTION_PLANEDETECTION_EXTREMEFINDER_H_
