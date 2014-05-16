#include "std_include.h"

int hough_accumulation()
{
	pcl::PointCloud<PointT>::Ptr cloud_tilted(new pcl::PointCloud<PointT>());
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
	pcl::PointIndices::Ptr outliers(new pcl::PointIndices());
	pcl::PointIndices::Ptr shadows(new pcl::PointIndices());

	std::cout<<"Start Hough accumulation"<<std::endl;

	if (pcl::io::loadPCDFile<PointT>("tilted_cloud.pcd", *cloud_tilted) == -1)
	{
		PCL_ERROR("couldn't read file");
		return -1;

	}

	int idx;
	std::ifstream idx_reader("in_plane.idx");
	while (!idx_reader.eof())
	{
		idx_reader >> idx;
		inliers->indices.push_back(idx);
	}
	idx_reader.close();

	idx_reader.open("not_in_plane.idx");
	while (!idx_reader.eof())
	{
		idx_reader >> idx;
		outliers->indices.push_back(idx);
	}
	idx_reader.close();
	
	idx_reader.open("shadow.idx");
	while (!idx_reader.eof())
	{
		idx_reader >> idx;
		shadows->indices.push_back(idx);
	}
	idx_reader.close();

	int x = 0;

	double accumulator[100][100] = { 0 };

	std::vector<Eigen::Vector3f> lights;
	std::vector<double> distances;
	extremeFinder ef;
	ef.find(cloud_tilted);
	double xmin = ef.x_min, xmax = ef.x_max, zmin = ef.z_min, zmax = ef.z_max;
	double ylevel = ef.y_max - 0.4;



	std::cout << "Start Accumulation Process"<<std::endl;
	for (int i = 0; i < shadows->indices.size(); i++)
	{
		int idx_shadow = shadows->indices.at(i);
		PointT ps = cloud_tilted->at(idx_shadow);//point in shadow
		for (int j = 0; j < outliers->indices.size(); j += 100)
		{
			int idx_object = outliers->indices.at(j);
			PointT po = cloud_tilted->at(idx_object);//point in object

			Eigen::Vector3f vs(ps.x,ps.y,ps.z);
			Eigen::Vector3f vo(po.x,po.y,po.z);

			double alpha = (ylevel - ps.y) / (po.y - ps.y);//set the z-plane position of light sources as 0.5
			Eigen::Vector3f light = vs + alpha*(vo - vs);//position of the light source

			double dist = pow(ps.x - po.x, 2) + pow(ps.y - po.y, 2) + pow(ps.z - po.z, 2);

			lights.push_back(light);
			distances.push_back(exp(-100*dist));


		}
	}
	std::cout<<"write to an array"<<std::endl;
	for (int i = 0; i < lights.size();i++)
	{
		Eigen::Vector3f light = lights.at(i);
		int xpos = (light[0] - xmin) * 100 / (xmax - xmin);
		int zpos = (light[2] - zmin) * 100 / (zmax - zmin);
		accumulator[std::min(std::max(xpos, 0), 99)][std::min(std::max(zpos, 0), 99)]+= distances.at(i);

	}
	for (int i = 0; i < 100; i++)
	{
		for (int j = 0; j < 100; j++)
		{
			double ipos = (i - 0) * (xmax - xmin) / 100 + xmin;
			double jpos = (j - 0) * (zmax - zmin) / 100 + zmin;
			pcl::PointXYZRGBA temp_point;
			temp_point.x = ipos;
			temp_point.y = ylevel;
			temp_point.z = jpos;
			temp_point.rgb = accumulator[i][j];
			cloud_tilted->push_back(temp_point);
		}
	}

	pcl::PCDWriter pcd_writer;
	pcd_writer.write("light_map.pcd",*cloud_tilted);

	std::ofstream writer("accumulator.txt");
	writer << "a = [";
	for (int i = 0; i < 100; i++)
	{
		for (int j = 0; j < 100; j++)
		{

			writer << accumulator[i][j] << " ";
		}
		if (i != 99)
		{
			writer << ";"<<std::endl;
		}
		
	}
	writer << "];";
	writer.close();
	
	std::cout<<"Finished!"<<std::endl;

	return 0;
}