/*
 * extremeFinder.h
 *
 *  Created on: Nov 14, 2013
 *      Author: joey
 */

#ifndef EXTREMEFINDER_H_
#define EXTREMEFINDER_H_

#include <pcl\point_types.h>
#include <pcl\point_cloud.h>

typedef pcl::PointXYZRGBA PointT;

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

                extremeFinder(){x_min=0;y_min=0;z_min=0;x_max=0;y_max=0;z_max=0; x_mean = 0; y_mean=0; z_mean =0;};

				int find(pcl::PointCloud<PointT>::Ptr cloud);
                int print();


};




#endif /* EXTREMEFINDER_H_ */