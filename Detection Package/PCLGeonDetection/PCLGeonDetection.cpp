#include "std_include.h"
#include "geonDetector.h"
//typedef pcl::PointXYZ PointT;

int
main (int argc, char** argv)
{
	//parse file name
	std::vector<int> filenames;
	filenames = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");
	if (filenames.size () != 1)
	{
	  std::cout << "Filenames missing.\n";
	  //showHelp (argv[0]);
	  exit (-1);
	}
	//create an geon detector object
	geonDetector det;
	det.initialize();

  // Read in the cloud data
  det.setInputCloud(argv[filenames[0]]);
  std::cerr << "PointCloud has: " << det.getInputCloud()->size()<< " data points." << std::endl;

  // Build a pass through filter to remove spurious NaNs
  det.preprocess();

  //// Save the filtered cloud data into a file for debugging purpose
  //std::string prefix = "filtered_";
  //prefix.append(argv[filenames[0]]);
  //writer.write(prefix.c_str(),*cloud_filtered);

  // Estimate point normals
  det.findNormals();

  // Create the segmentation object for the planar model and set all the parameters

  det.detect(pcl::SACMODEL_CYLINDER);





/****
pcl::visualization::PCLVisualizer viewer;

pcl::visualization::PointCloudColorHandlerCustom<PointT> scene_color (cloud, 255, 255, 255);
viewer.addPointCloud (cloud, scene_color, "scene");
viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "scene");

pcl::visualization::PointCloudColorHandlerCustom<PointT> cylinder_color (cloud_cylinder, 0, 255, 255);
viewer.addPointCloud (cloud_cylinder, cylinder_color, "cylinder");
viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cylinder");

while(!viewer.wasStopped())
{
	viewer.spinOnce();
}
****/

  return (0);
}
