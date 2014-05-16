
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/features/normal_3d.h>
#include <boost/thread/thread.hpp>
//#include <pcl/range_image/range_image.h>
//#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/ModelCoefficients.h>
//#include <pcl/visualization/point_cloud_handlers.h>
#include <pcl/console/parse.h>
#include <iostream> 
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <array>
#include <hash_set>
#include <pcl/filters/radius_outlier_removal.h>
#include <string>
#include "extremeFinder.h"
#include "geonDetector.h"

typedef pcl::PointXYZRGBA PointT;

#define PI 3.1415926
#define HITMAP_SIZE 32400

extern void shadow_detection(std::string);
extern int hough_accumulation(void);
extern int region_growing_rgb_segmentation(std::string);
extern int is_in_indices(pcl::PointIndices &input, int idx);
extern void cumulate_hits( std::array<pcl::PointIndices,HITMAP_SIZE> &hitmap, pcl::PointCloud<PointT>::Ptr cloud, pcl::PointIndices::Ptr mask, Eigen::Vector3f light);

extern void save_hitmap(std::array<pcl::PointIndices,HITMAP_SIZE> &hitmap, char *path);

extern void reverse_indices(pcl::PointIndices::Ptr input, pcl::PointIndices::Ptr output, int max_index);