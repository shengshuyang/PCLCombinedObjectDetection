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

                extremeFinder(){x_min=0;y_min=0;z_min=0;x_max=0;y_max=0;z_max=0;};

				int find(pcl::PointCloud<PointT>::Ptr cloud);
                int print();


};


int extremeFinder::find(pcl::PointCloud<PointT>::Ptr cloud)
{
        if(cloud->empty() || cloud->size()==0)
        {
                std::cout<<"Cloud empty!"<<std::endl;
                return -1;
        }

        for( int i=0; i<cloud->size();i++)
        {
                try{

                        PointT p = cloud->at(i);
                        if( p.x > x_max) x_max = p.x;
                        if( p.x < x_min) x_min = p.x;

                        if( p.y > y_max) y_max = p.y;
                        if( p.y < y_min) y_min = p.y;

                        if( p.z > z_max) z_max = p.z;
                        if( p.z < z_min) z_min = p.z;

                }catch(const std::out_of_range& err){

                        std::cerr<< "Out of Range error: " << err.what()<<". Iteration number: "<<i << std::endl;

                }
        }
        return 0;
}

int extremeFinder::print()
{
        std::cout<< "X from "<< x_min << " to " << x_max <<std::endl;
        std::cout<< "Y from "<< y_min << " to " << y_max <<std::endl;
        std::cout<< "Z from "<< z_min << " to " << z_max <<std::endl;
        return 0;
}

#endif /* EXTREMEFINDER_H_ */