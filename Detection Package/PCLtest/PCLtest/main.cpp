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
#include <pcl/point_types_conversion.h>
#include <pcl/console/parse.h>
#include <iostream> 
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include "intersection_test.h"
#include "extremeFinder.h"
#include "geonDetector.h"
#include <array>
#define PI 3.1415926
#define HITMAP_SIZE 32400
int is_in_indices(pcl::PointIndices &input, int idx)
{
	for( int i=0;i<input.indices.size();i++)
	{
		if( input.indices.at(i) == idx) return(1);
	}
	return(0);
}
void cumulate_hits( std::array<pcl::PointIndices,HITMAP_SIZE> &hitmap, pcl::PointCloud<PointT>::Ptr cloud, pcl::PointIndices::Ptr mask, Eigen::Vector3f light)
{
	double alpha, theta;
	//for( int i=0; i<HITMAP_SIZE; i++)
	//{
	//	pcl::PointIndices::Ptr temp(new pcl::PointIndices());

	//}

	Eigen::Vector3f x_axis(1,0,0);
	for( int j = 0; j<mask->indices.size();j++)//loop over the mask indices
	{
		int i = mask->indices.at(j);
		Eigen::Vector3f v(cloud->at(i).x,cloud->at(i).y,cloud->at(i).z);//initialize v with point position
		
		v = v - light;//get the vector from light by subtraction

		if( v[1] >=0 ) continue;//if it's going up instead of going down, don't count it.

		Eigen::Vector3f v_xz = v.cwiseProduct(Eigen::Vector3f(1.0,0.0,1.0));//project the point to xz plane
		v.normalize(); v_xz.normalize();

		alpha = acos(v_xz.dot(x_axis));//the direction inside xz plane, from 0 to 360 degrees
		theta = acos(v_xz.dot(v));//the direction to negative y axis, from 0 to 90 degrees

		int alpha_i = (int) (alpha*180/PI);
		int theta_i = (int) (theta*180/PI);

		if( v[2]<0 ) alpha_i = 359 - alpha_i; 
		theta_i = abs(theta_i);

		//if( ! hitmap.at( alpha_i*90 + theta_i)) 
		//{
		//	pcl::PointIndices::Ptr temp(new pcl::PointIndices());
		//	hitmap.at( alpha_i*90 + theta_i) = temp;
		//}
		hitmap[ alpha_i*90 + theta_i].indices.push_back(i);
	}

		std::cout<<std::endl<<"Done Hitmap"<<std::endl;

}
void save_hitmap(std::array<pcl::PointIndices,HITMAP_SIZE> &hitmap, char *path)
{
	pcl::PointCloud<pcl::PointXYZ> temp;
	std::string str = path;
	std::string txt_path = str.append(".txt");
	str = path;
	std::string pcd_path = str.append(".pcd");

	std::ofstream of(txt_path.c_str());
	std::cout<<std::endl;
	for( int i=0; i< 360; i++)//alpha
	{
		for( int j = 0; j < 90; j++)//theta
		{
			of << hitmap[i*90 + j].indices.size();
			double aa = i * PI/180;
			double tt = j * PI/180;
			double xx = cos(tt)*cos(aa);
			double zz = cos(tt)*sin(aa);
			temp.push_back( pcl::PointXYZ( xx, (double)hitmap[i*90 + j].indices.size()/200, zz));
			//if( hitmap[i*90 + j].indices.size() >2)
			//{
			//	temp.push_back( pcl::PointXYZ( xx, 0.5, zz));
			//}
			//else
			//{
			//	temp.push_back( pcl::PointXYZ( xx, 0.0, zz));
			//}
			if( (j+1)%90 == 0)
			{
				of <<";"<<std::endl;
				std::cout<<".";
			}
			else
			{
				of << ",";
			}
		}
	}
	of.close();

	pcl::PCDWriter writer;
	writer.write(pcd_path.c_str(),temp);
}

void reverse_indices(pcl::PointIndices::Ptr input, pcl::PointIndices::Ptr output, int max_index)
{
	//filling leading indices before the smallest in input
	for( int i=0;i<input->indices.at(0);i++)
	{
		output->indices.push_back(i);
	}
	//filling indices in between
	for( int i=0; i< input->indices.size()-1;i++)
	{
		int a = input->indices.at(i);
		int b = input->indices.at(i+1);
		if( b==a+1) continue;
		for( int j = a+1; j< b; j++)
		{
			output->indices.push_back(j);
		}
	}
	//filing trailing indices
	for( int i= input->indices.at(input->indices.size()-1)+1; i<max_index;i++)
	{
		output->indices.push_back(i);
	}
}

int main(int argc, char *argv[])
{

	//std::vector<int> filenames;
	//filenames = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");
	//if (filenames.size () != 1)
	//{
	//std::cout << "Filenames missing.\n";
	//exit (-1);
	//}
	//std::string filename;
	//filename = argv[filenames[0]];

	//pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
	//pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);

	//if(pcl::io::loadPCDFile<PointT>(filename,*cloud) == -1)
	//{
	//	PCL_ERROR (	"couldn't read file");
	//	return -1;
	//	
	//}
	////pass through filter
	//pcl::PassThrough<PointT> pass;
	//pass.setInputCloud (cloud);
	//pass.setFilterFieldName ("z");
	//pass.setFilterLimits (-3, 3);
	//pass.filter (*cloud_filtered);
	////pcl::copyPointCloud(*cloud_filtered,*cloud_copy);
	////std::cerr << "PointCloud after filtering has: " << cloud_filtered->points.size () << " data points." << std::endl;

	//geonDetector det;
	//det.initialize();
	//det.setInputCloud(cloud_filtered);
	//det.preprocess();
	//det.findNormals();
	//det.detect(pcl::SACMODEL_PLANE);
	//std::cout<<"Done!"<<std::endl;

	//pcl::PointCloud<PointT>::Ptr cloud_tilted(new pcl::PointCloud<PointT>());
	//det.rotate_to_horizon(cloud_tilted);

	//pcl::PCDWriter writer;
	//writer.write("tilted_box2.pcd",*cloud_tilted);

	//pcl::PointIndices::Ptr in_plane = det.geonIndices;
	//pcl::PointIndices::Ptr not_in_plane (new pcl::PointIndices());
	//int max_index = cloud_tilted->size();
	//reverse_indices(in_plane,not_in_plane,max_index);
	//std::cout<<"Number of indices not in plane:"<< not_in_plane->indices.size()<<std::endl;

////=============================Part 1, Geometry based initial guess=============================
	
	////extreme finder
	//std::cout <<"before ef"<<std::endl;
 //   extremeFinder *ef = new extremeFinder();
 //   std::cout <<"after ef"<<std::endl;
 //   ef->find(cloud_tilted);
 //   ef->print();
	//
	////set artificial light position
	//Eigen::Vector3f light_pos = Eigen::Vector3f(0.8*ef->x_max+0.2*ef->x_min,ef->y_max+ 0.2 , 0.8* ef->z_max+0.2*ef->z_min);
 //   std::cout << "Light Position: x= "<< light_pos(0) << ", y= "<< light_pos(1) << ", z= "<< light_pos(2) << ". " <<std::endl;
	//
	////find hit map between rays and plane
	//std::array<pcl::PointIndices,HITMAP_SIZE> *hitmap_in_plane = new std::array<pcl::PointIndices,HITMAP_SIZE>();
	//cumulate_hits(*hitmap_in_plane,cloud_tilted,in_plane,light_pos);
	//save_hitmap(*hitmap_in_plane,"in_plane");

	////find hit map between rays and objects on the plane
	//std::array<pcl::PointIndices,HITMAP_SIZE> *hitmap_not_in_plane = new std::array<pcl::PointIndices,HITMAP_SIZE>();
	//cumulate_hits(*hitmap_not_in_plane,cloud_tilted,not_in_plane,light_pos);
	//save_hitmap(*hitmap_not_in_plane,"not_in_plane");

	////do an AND operation to find rays that cross both object and plane. these rays are potential shadows
	//std::array<pcl::PointIndices,HITMAP_SIZE> *hitmap_both = new std::array<pcl::PointIndices,HITMAP_SIZE>();
	//for( int i=0;i<HITMAP_SIZE;i++)
	//{
	//	int sn = hitmap_not_in_plane->at(i).indices.size();//not in plane
	//	int si = hitmap_in_plane->at(i).indices.size();//in plane
	//	if(sn > 2 && si > 2)
	//	{
	//		hitmap_both->at(i) = hitmap_in_plane->at(i);
	//	}
	//}
	//save_hitmap(*hitmap_both,"both");

	////backward mapping the final hitmap to original point cloud, the result is gonna be shadow position
	//pcl::PointIndices::Ptr shadow_candidates( new pcl::PointIndices());
	//for( int i=0; i< hitmap_both->size();i++)// i th direction
	//{
	//	for( int j = 0; j<hitmap_both->at(i).indices.size();j++)// j th point index in that direction
	//	{
	//		shadow_candidates->indices.push_back(hitmap_both->at(i).indices.at(j));
	//	}
	//}
	//pcl::PointCloud<PointT>::Ptr final_shadow(new pcl::PointCloud<PointT>());
	//pcl::copyPointCloud(*cloud_tilted,*shadow_candidates,*final_shadow);
	//writer.write("final_shadow.pcd",*final_shadow);

//=============================convert from GRBA to HSV=============================

	//pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_tilted(new pcl::PointCloud<pcl::PointXYZRGBA>());
	//pcl::PCDReader reader;
	//reader.read("tilted_box2.pcd",*cloud_tilted);

	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_tilted_rgb(new pcl::PointCloud<pcl::PointXYZRGB>());
	//pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud_tilted_hsv(new pcl::PointCloud<pcl::PointXYZHSV>());

	//pcl::copyPointCloud(*cloud_tilted,*cloud_tilted_rgb);
	////for( int i=0; i<cloud_tilted->size();i++)
	////{
	////	cloud_tilted_rgb->push_back( pcl::PointXYZRGB());
	////	cloud_tilted_rgb->at(i).r = cloud_tilted->at(i).r;
	////	cloud_tilted_rgb->at(i).g = cloud_tilted->at(i).g;
	////	cloud_tilted_rgb->at(i).b = cloud_tilted->at(i).b;
	////	cloud_tilted_rgb->at(i).x = cloud_tilted->at(i).x;
	////	cloud_tilted_rgb->at(i).y = cloud_tilted->at(i).y;
	////	cloud_tilted_rgb->at(i).z = cloud_tilted->at(i).z;
	////}

	//pcl::PointCloudXYZRGBtoXYZHSV(*cloud_tilted_rgb,*cloud_tilted_hsv);

	//pcl::PCDWriter writer;
	//writer.write("tilted_box2_hsv.pcd",*cloud_tilted_hsv);
	//std::cout<<"Done RGB to HSV conversion"<<std::endl;


//=============================Part 2, Color Consistancy shadow detection=============================

	//save and load index sets


	//std::ofstream index_writer("in_plane.idx");
	//for(int i=0;i<in_plane->indices.size();i++)
	//{
	//	index_writer << in_plane->indices.at(i) << std::endl;
	//}
	//index_writer.close();

	//std::ofstream index_writer2("not_in_plane.idx");
	//for(int i=0;i<not_in_plane->indices.size();i++)
	//{
	//	index_writer2 << not_in_plane->indices.at(i) << std::endl;
	//}
	//index_writer2.close();


	pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud_hsv(new pcl::PointCloud<pcl::PointXYZHSV>());
	pcl::PCDReader reader;
	pcl::PCDWriter writer;
	reader.read("tilted_box2_hsv.pcd",*cloud_hsv);

	pcl::PointIndices::Ptr not_in_plane (new pcl::PointIndices());
	pcl::PointIndices::Ptr in_plane (new pcl::PointIndices());


	int idx;
	std::ifstream in_plane_reader("in_plane.idx");
	while(!in_plane_reader.eof())
	{
		in_plane_reader >> idx;
		in_plane->indices.push_back(idx);
	}

	std::ifstream not_in_plane_reader("not_in_plane.idx");
	while(!not_in_plane_reader.eof())
	{
		not_in_plane_reader >> idx;
		not_in_plane->indices.push_back(idx);
	}

	pcl::KdTreeFLANN<pcl::PointXYZHSV> kdtree;
	kdtree.setInputCloud(cloud_hsv);


	int K = 3;
	std::vector<int> pointIdxNKNSearch(K);
	std::vector<float> pointNKNSquaredDistance(K);

	pcl::PointIndices::Ptr seed_idx( new pcl::PointIndices());
	double percent = 0;
	std::cout<<"Start searching for seeds"<<std::endl;
	for( int i=0; i< in_plane->indices.size();i++)
	{
		percent = ((double)i)/in_plane->indices.size();
		//std::cout<<(int)(percent*100)<<"%"<<std::endl;
		int idx = in_plane->indices.at(i);
		//pointIdxNKNSearch will store all indices that are close to the i'th point in plane
		int out = kdtree.nearestKSearch(idx, K, pointIdxNKNSearch, pointNKNSquaredDistance);
		if( out <= 0) continue;
		for( int j=0;j<K;j++)
		{
			if( std::find(not_in_plane->indices.begin(),not_in_plane->indices.end(),pointIdxNKNSearch.at(j)) != not_in_plane->indices.end())
			{
				seed_idx->indices.push_back(idx);
				break;
			}
		}
	}
	std::cout<<"Done!"<<std::endl;
	pcl::PointCloud<pcl::PointXYZHSV>::Ptr seed_points(new pcl::PointCloud<pcl::PointXYZHSV>());
	pcl::copyPointCloud(*cloud_hsv,*seed_idx,*seed_points);

	writer.write("seed_points.pcd",*seed_points);


	for( int i=0; i<seed_idx->indices.size();i++)
	{
		int idx = seed_idx->indices.at(i);
		cloud_hsv->at(idx).y = cloud_hsv->at(idx).y+0.5;
	}

	writer.write("seed_points2.pcd",*cloud_hsv);
	K=2;
	pcl::PointIndices::Ptr seed_idx_temp( new pcl::PointIndices());
	for( int itr = 0; itr <10;itr++)
	{
		for( int i=0;i<seed_idx->indices.size();i++)
		{
			int idx = seed_idx->indices.at(i);
			int out = kdtree.nearestKSearch(idx, K, pointIdxNKNSearch, pointNKNSquaredDistance);
			int idx_neighbour = pointIdxNKNSearch.at(1);
			if( std::find(in_plane->indices.begin(),in_plane->indices.end(),idx_neighbour) != in_plane->indices.end() && // find if the point idx_neighbour is in the set of in_plane
				std::find(seed_idx->indices.begin(),seed_idx->indices.end(),idx_neighbour) == in_plane->indices.end())//in plane but not in seed yet
			{
				pcl::PointXYZHSV p1(cloud_hsv->at(idx));
				pcl::PointXYZHSV p2(cloud_hsv->at(idx_neighbour));
				if( abs(p2.h -p1.h) < 0.05 && (p2.v/p1.v) >=1.0)
				{
					seed_idx_temp->indices.push_back(idx_neighbour);
				}
			}

		}
		for( int i=0;i<seed_idx_temp->indices.size();i++)
		{
			seed_idx->indices.push_back(seed_idx_temp->indices.at(i));
		}
		//seed_idx_temp->indices.clear();
	}
	pcl::PointCloud<pcl::PointXYZHSV>::Ptr seed_points_ext(new pcl::PointCloud<pcl::PointXYZHSV>());
	pcl::copyPointCloud(*cloud_hsv,*seed_idx,*seed_points_ext);

	writer.write("seed_points_ext.pcd",*seed_points);

	//Eigen::Vector3f *d = new Eigen::Vector3f();
	//Eigen::Vector3f *hit = new Eigen::Vector3f(1,0,0);
	////std::vector<std::vector<pcl::PointIndices::Ptr>> hit_map;
	//pcl::PointIndices::Ptr hit_map[89][89];
	//int hit_test[89][89];

	//std::ofstream file("hit_test.txt");
	//for( int i = 0; i < 1; i++)
	//{
	//	//hit_map.push_back(new std::vector<pcl::PointIndices::Ptr>);
	//	for(int j = 0; j< 90;j++)
	//	{
	//		*d = Eigen::Vector3f(0,cos(j*PI/180),-sin(j*PI/180));
	//		pcl::PointIndices::Ptr temp(new pcl::PointIndices());
	//		//hit_map.at(i+45).at(j+45) = temp;
	//		hit_map[i+45][j+45] = temp;
	//		*d = Eigen::Vector3f(i,-45,j); 
	//		hit_test[i][j] = intersection_test(&light_pos, d, hit_map[i+45][j+45], det.cloud_filtered,not_in_plane,0.1);
	//		std::cout<<hit_test[i][j]<<",";
	//		file<<hit_test[i][j]<<",";
	//	}
	//	std::cout<<";"<<std::endl;
	//	file<<";"<<std::endl;
	//}
	



//#pragma region Normal Estimation

	////create a normal distance feature estimator
	//pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal> ne;

	////set 'cloud' as input cloud.
	//ne.setInputCloud(cloud);
	////create a kdtree and set it as the search method of ne.
	//pcl::search::KdTree<pcl::PointXYZ>::Ptr tree( new pcl::search::KdTree<pcl::PointXYZ>());
	//ne.setSearchMethod(tree);

	////create the output dataset.
	//pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

	////set parameter
	//ne.setKSearch(9);

	////compute features
	//ne.compute(*cloud_normals);

//#pragma endregion



	//pcDenoise den;
	//den.setCloud(cloud_filtered);
	//den.pcDecompose();



//#pragma region visualize point cloud and its normals
//
//	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer("My Viewer"));
//	viewer->setBackgroundColor(0,0,0);
//	viewer->initCameraParameters();
//	pcl::visualization::PointCloudColorHandlerRGBField<PointT> single_color(cloud);
//	viewer->addPointCloud<pcl::PointXYZRGBA>(cloud,single_color,"smooth");
//	
//	//viewer->addPointCloud<pcl::PointXYZRGBA >(cloud_copy,single_color,"original");
//	//viewer->addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud,cloud_normals,10,0.02,"normal");
//	//viewer->addCoordinateSystem(1.0);
//	
//	
//
//	while(!viewer->wasStopped())
//	{
//		viewer->spinOnce(100);
//		boost::this_thread::sleep (boost::posix_time::microseconds (10000));
//	}
//
//#pragma endregion


return (0);
	
}


