
#include "std_include.h"
#include "intersection_test.h"

#include <pcl/common/transforms.h>
#include <pcl/point_types_conversion.h>
#include <string>
using namespace std;
int  is_in_indices(pcl::PointIndices &input, int idx)
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

void point_cloud_rgba_to_hsv(pcl::PointCloud<pcl::PointXYZRGBA> &in, pcl::PointCloud<pcl::PointXYZHSV> &out, std::string filename)
{

	pcl::PointCloudXYZRGBAtoXYZHSV(in,out);
	for( int i=0; i<in.size();i++)
	{
		out.at(i).x = in.at(i).x;
		out.at(i).y = in.at(i).y;
		out.at(i).z = in.at(i).z;
	}
	pcl::PCDWriter writer;
	writer.write(filename.c_str(),out);
	std::cout<<"Done RGB to HSV conversion"<<std::endl;

}

void shadow_detection(std::string filename)
{
	pcl::PCDReader reader;
	pcl::PCDWriter writer;

	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr cloud_tilted(new pcl::PointCloud<PointT>());
	pcl::PointCloud<PointT>::Ptr cloud_temp(new pcl::PointCloud<PointT>());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_tilted_rgb(new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud_tilted_hsv(new pcl::PointCloud<pcl::PointXYZHSV>());
	pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud_seed_hsv(new pcl::PointCloud<pcl::PointXYZHSV>());
	//pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud_hsv(new pcl::PointCloud<pcl::PointXYZHSV>());

	pcl::PointIndices::Ptr in_plane (new pcl::PointIndices());
	pcl::PointIndices::Ptr not_in_plane (new pcl::PointIndices());
	pcl::PointIndices::Ptr seed_idx (new pcl::PointIndices());
	pcl::PointIndices::Ptr seed_idx_original (new pcl::PointIndices());//seed indices before singular point removal

	if(pcl::io::loadPCDFile<PointT>(filename,*cloud) == -1)
	{
		PCL_ERROR (	"couldn't read file");
		return;
		
	}
	////pass through filter
	//pcl::PassThrough<PointT> pass;
	//pass.setInputCloud (cloud);
	//pass.setFilterFieldName ("z");
	//pass.setFilterLimits (-3, 3);
	//pass.filter (*cloud_filtered);
	//pcl::copyPointCloud(*cloud_filtered,*cloud_copy);
	//std::cerr << "PointCloud after filtering has: " << cloud_filtered->points.size () << " data points." << std::endl;

	geonDetector det;
	det.initialize();
	det.setInputCloud(cloud);
	det.preprocess();
	det.findNormals();
	det.detect(pcl::SACMODEL_PLANE);
	det.rotate_to_horizon(cloud_tilted);


	//write geon indices into a file
	std::ofstream of("inlier_indices.txt");
	for (int i=0; i < det.geonIndices->indices.size(); i++)
	{
		of << det.geonIndices->indices.at(i);
		if(i != det.geonIndices->indices.size()-1)
		{
			of << std::endl;
		}
	}
	of.close();
	std::cout<<"Done!"<<std::endl;

	point_cloud_rgba_to_hsv(*cloud_tilted,*cloud_tilted_hsv, filename.insert(0,"hsv_"));

#pragma region deprecated codes
	////extreme finder
	//std::cout <<"before ef"<<std::endl;
	//extremeFinder *ef = new extremeFinder();
	//std::cout <<"after ef"<<std::endl;
	//ef->find(cloud_tilted);
	//ef->print();
	//Eigen::Affine3f centering(Eigen::Translation3f(ef->x_mean,ef->y_mean,ef->z_mean));
	//pcl::copyPointCloud(*cloud_tilted, *cloud_temp);
	//pcl::transformPointCloud(*cloud_temp,*cloud_tilted,centering);

	//writer.write("tilted_cloud.pcd",*cloud_tilted);

	//int max_index = cloud_tilted->size();
	//in_plane = det.geonIndices;
	//reverse_indices(in_plane,not_in_plane,max_index);
	//std::cout<<"Number of indices not in plane:"<< not_in_plane->indices.size()<<std::endl;

	//pcl::PointCloud<pcl::PointXYZRGBA>::Ptr in_plane_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
	//pcl::copyPointCloud(*cloud_tilted,*in_plane,*in_plane_cloud);
	//writer.write("in_plane.pcd",*in_plane_cloud);

	//pcl::PointCloud<pcl::PointXYZRGBA>::Ptr not_in_plane_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
	//pcl::copyPointCloud(*cloud_tilted,*not_in_plane,*not_in_plane_cloud);
	//writer.write("not_in_plane.pcd",*not_in_plane_cloud);
//////=============================Part 1, Geometry based initial guess=============================
//	
	////extreme finder
	//std::cout <<"before ef"<<std::endl;
 //   extremeFinder *ef = new extremeFinder();
 //   std::cout <<"after ef"<<std::endl;
 //   ef->find(cloud_tilted);
 //   ef->print();
//	
//	//set artificial light position
//	Eigen::Vector3f light_pos = Eigen::Vector3f(0.4*ef->x_max+0.6*ef->x_min,ef->y_max+ 0.3 , 0.4* ef->z_max+0.6*ef->z_min);
//    std::cout << "Light Position: x= "<< light_pos(0) << ", y= "<< light_pos(1) << ", z= "<< light_pos(2) << ". " <<std::endl;
//	
//	//find hit map between rays and plane
//	std::array<pcl::PointIndices,HITMAP_SIZE> *hitmap_in_plane = new std::array<pcl::PointIndices,HITMAP_SIZE>();
//	cumulate_hits(*hitmap_in_plane,cloud_tilted,in_plane,light_pos);
//	save_hitmap(*hitmap_in_plane,"in_plane");
//
//	//find hit map between rays and objects on the plane
//	std::array<pcl::PointIndices,HITMAP_SIZE> *hitmap_not_in_plane = new std::array<pcl::PointIndices,HITMAP_SIZE>();
//	cumulate_hits(*hitmap_not_in_plane,cloud_tilted,not_in_plane,light_pos);
//	save_hitmap(*hitmap_not_in_plane,"not_in_plane");
//
//	//do an AND operation to find rays that cross both object and plane. these rays are potential shadows
//	std::array<pcl::PointIndices,HITMAP_SIZE> *hitmap_both = new std::array<pcl::PointIndices,HITMAP_SIZE>();
//	for( int i=0;i<HITMAP_SIZE;i++)
//	{
//		int sn = hitmap_not_in_plane->at(i).indices.size();//not in plane
//		int si = hitmap_in_plane->at(i).indices.size();//in plane
//		if(sn > 2 && si > 2)
//		{
//			hitmap_both->at(i) = hitmap_in_plane->at(i);
//		}
//	}
//	save_hitmap(*hitmap_both,"both");
//
//	//backward mapping the final hitmap to original point cloud, the result is gonna be shadow position
//	pcl::PointIndices::Ptr shadow_candidates( new pcl::PointIndices());
//	for( int i=0; i< hitmap_both->size();i++)// i th direction
//	{
//		for( int j = 0; j<hitmap_both->at(i).indices.size();j++)// j th point index in that direction
//		{
//			shadow_candidates->indices.push_back(hitmap_both->at(i).indices.at(j));
//		}
//	}
//	for( int i=0;i<shadow_candidates->indices.size();i++)
//	{
//		int idx = shadow_candidates->indices.at(i);
//		cloud_tilted->at(idx).r = 0;
//		cloud_tilted->at(idx).g = 0;
//		cloud_tilted->at(idx).b = 255;
//	}
//	//pcl::PointCloud<PointT>::Ptr final_shadow(new pcl::PointCloud<PointT>());
//	//pcl::copyPointCloud(*cloud_tilted,*shadow_candidates,*final_shadow);
//	writer.write("final_shadow.pcd",*cloud_tilted);
//
////=============================convert from GRBA to HSV=============================

//    point_cloud_rgba_to_hsv(*cloud_tilted,*cloud_tilted_hsv, filename.insert(0,"hsv_"));
//
//////=============================Part 2, Color Consistancy shadow detection=============================
//
//	//searching for the seeds
//
//	//reader.read("tilted_tape5_hsv.pcd",*cloud_hsv);
//
//	pcl::KdTreeFLANN<pcl::PointXYZHSV> kdtree;
//	kdtree.setInputCloud(cloud_tilted_hsv);
//
//	int K = 5;	double percent = 0;
//	std::vector<int> pointIdxNKNSearch(K);
//	std::vector<float> pointNKNSquaredDistance(K);
//
//	std::cout<<"Start searching for seeds"<<std::endl;
//
//	std::hash_set<int, hash_compare<int, greater<int>>> not_in_plane_hash;
//	for( int i=0;i<not_in_plane->indices.size();i++)
//	{
//		not_in_plane_hash.insert(not_in_plane->indices.at(i));
//	}
//	std::cout<<"Hashing finished!"<<std::endl;
//
//	for( int i=0; i< in_plane->indices.size();i++)
//	{
//		//percent = ((double)i)/in_plane->indices.size();
//		//std::cout<<(int)(percent*100)<<"%"<<std::endl;
//		int idx = in_plane->indices.at(i);
//		//pointIdxNKNSearch will store all indices that are close to the i'th point in plane
//		int out = kdtree.nearestKSearch(idx, K, pointIdxNKNSearch, pointNKNSquaredDistance);
//		if( out <= 0) continue;
//		for( int j=0;j<K;j++)
//		{
//			int idx_neighbour = pointIdxNKNSearch.at(j);
//			if( not_in_plane_hash.find(idx_neighbour) != not_in_plane_hash.end())
//			{
//				seed_idx_original->indices.push_back(idx);
//				break;
//			}
//		}
//	}
//	std::cout<<"Done!"<<std::endl;
//
//	////save index sets
//	//std::ofstream index_writer3("seed_idx.idx");
//	//for(int i=0;i<seed_idx->indices.size();i++)
//	//{
//	//	index_writer3 << seed_idx->indices.at(i) << std::endl;
//	//}
//	//index_writer3.close();
//
//
//	std::ofstream index_writer("in_plane.idx");
//	for (int i = 0; i < in_plane->indices.size(); i++)
//	{
//		index_writer << in_plane->indices.at(i) << std::endl;
//	}
//	index_writer.close();
//
//	//std::ofstream index_writer2("not_in_plane.idx");
//	index_writer.open("not_in_plane.idx");
//	for (int i = 0; i < not_in_plane->indices.size(); i++)
//	{
//		index_writer << not_in_plane->indices.at(i) << std::endl;
//	}
//	index_writer.close();
//
//
//	////laod index sets
//	//int idx;
//	//std::ifstream in_plane_reader("in_plane.idx");
//	//while(!in_plane_reader.eof())
//	//{
//	//	in_plane_reader >> idx;
//	//	in_plane->indices.push_back(idx);
//	//}
//
//	//std::ifstream not_in_plane_reader("not_in_plane.idx");
//	//while(!not_in_plane_reader.eof())
//	//{
//	//	not_in_plane_reader >> idx;
//	//	not_in_plane->indices.push_back(idx);
//	//}
//
//	//std::ifstream seed_idx_reader("seed_idx.idx");
//	//while(!seed_idx_reader.eof())
//	//{
//	//	seed_idx_reader >> idx;
//	//	seed_idx->indices.push_back(idx);
//	//}
//
//
//	//===========================some visualizations===============================================
//	//pcl::PointCloud<pcl::PointXYZHSV>::Ptr seed_points(new pcl::PointCloud<pcl::PointXYZHSV>());
//	//pcl::copyPointCloud(*cloud_tilted,*seed_idx,*seed_points);
//	//writer.write("seed_points.pcd",*seed_points);
//
//
//	//for( int i=0; i<seed_idx->indices.size();i++)
//	//{
//	//	int idx = seed_idx->indices.at(i);
//	//	cloud_tilted_hsv->at(idx).y = cloud_tilted_hsv->at(idx).y+0.5;
//	//}
//	//writer.write("seed_points2.pcd",*cloud_tilted_hsv);
//
//	//===================find average color of the shadow============
//	//double avg_h = 0,avg_s = 0,avg_v = 0;
//	//for( int i=0;i < seed_idx->indices.size();i++)
//	//{
//	//	avg_h += cloud_tilted_hsv->at(seed_idx->indices.at(i)).h;
//	//	avg_s += cloud_tilted_hsv->at(seed_idx->indices.at(i)).s;
//	//	avg_v += cloud_tilted_hsv->at(seed_idx->indices.at(i)).v;
//	//}
//	//avg_h = avg_h/seed_idx->indices.size();
//	//avg_s = avg_s/seed_idx->indices.size();
//	//avg_v = avg_v/seed_idx->indices.size();
//
//	pcl::RadiusOutlierRemoval<pcl::PointXYZHSV> rorfilter (true); // Initializing with true will allow us to extract the removed indices
//	rorfilter.setInputCloud (cloud_tilted_hsv);
//	rorfilter.setIndices(seed_idx_original);
//	rorfilter.setRadiusSearch (0.005);
//	rorfilter.setMinNeighborsInRadius (15);
//	rorfilter.setNegative (false);
//	rorfilter.filter (seed_idx->indices);
//
//	//pcl::PointCloud<pcl::PointXYZRGBA>::Ptr clean_seeds(new pcl::PointCloud<pcl::PointXYZRGBA>());
//	//pcl::copyPointCloud(*cloud_tilted,*seed_idx,*clean_seeds);
//	//writer.write("clean_seeds.pcd",*clean_seeds);
//	//rorfilter.applyFilterIndices(seed_idx->indices);
//	// The resulting cloud_out contains all points of cloud_in that have 4 or less neighbors within the 0.1 search radius
//	//indices_rem = rorfilter.getRemovedIndices ();
//	// The indices_rem array indexes all points of cloud_in that have 5 or more neighbors within the 0.1 search radius
//
//
//	K=15;
//
//	std::cout<<"Start hashing indices to speed up searching later on"<<std::endl;
//	std::hash_set<int,hash_compare<int, greater<int>>> seed_idx_hash;
//	for( int i=0;i<seed_idx->indices.size();i++)
//	{
//		seed_idx_hash.insert(seed_idx->indices.at(i));
//	}
//
//	std::hash_set<int,hash_compare<int, greater<int>>> in_plane_hash;
//	for( int i=0;i<in_plane->indices.size();i++)
//	{
//		in_plane_hash.insert(in_plane->indices.at(i));
//	}
//	std::cout<<"Hashing finished!"<<std::endl;
//	pcl::PointIndices::Ptr seed_idx_temp( new pcl::PointIndices());
//	for( int n = 0; n <100;n++)
//	{
//		std::cout<<n<<std::endl;
//		std::hash_set<int>::iterator itr = seed_idx_hash.begin();
//		for( ; itr != seed_idx_hash.end(); itr++)
//		{
//			int idx = *itr;
//			int out = kdtree.nearestKSearch(idx, K, pointIdxNKNSearch, pointNKNSquaredDistance);
//			for(int j=0;j<K;j++)
//			{
//				int idx_neighbour = pointIdxNKNSearch.at(j);
//				//if( std::find(in_plane->indices.begin(),in_plane->indices.end(),idx_neighbour) != in_plane->indices.end() && // find if the point idx_neighbour is in the set of in_plane
//				//	std::find(seed_idx->indices.begin(),seed_idx->indices.end(),idx_neighbour) == seed_idx->indices.end())//in plane but not in seed yet
//				if( seed_idx_hash.find(idx_neighbour) == seed_idx_hash.end() && 
//					in_plane_hash.find(idx_neighbour) != in_plane_hash.end())
//				{
//					pcl::PointXYZHSV p1(cloud_tilted_hsv->at(idx));
//					pcl::PointXYZHSV p2(cloud_tilted_hsv->at(idx_neighbour));
//					if( abs(p2.h - p1.h) < 30 && (p2.v/p1.v) <=1.5)
//					{
//						seed_idx_temp->indices.push_back(idx_neighbour);
//					}
//					break;//if found a neighbour that's not in seed yet but is part of the plane, we skip other neighbours of this seed, instead jump to the next seed.
//				}
//			}
//
//		}
//		//if(seed_idx_temp->indices.size() <100 )
//		//{
//		//	break;
//		//}
//		for( int i=0;i<seed_idx_temp->indices.size();i++)
//		{
//			seed_idx->indices.push_back(seed_idx_temp->indices.at(i));
//			seed_idx_hash.insert(seed_idx_temp->indices.at(i));
//		}
//		seed_idx_temp->indices.clear();
//
//	}
//
//	index_writer.open("shadow.idx");
//	for(int i=0;i<seed_idx->indices.size();i++)
//	{
//		index_writer << seed_idx->indices.at(i) << std::endl;
//	}
//	index_writer.close();
//
//	//pcl::PointCloud<pcl::PointXYZHSV>::Ptr seed_points_ext(new pcl::PointCloud<pcl::PointXYZHSV>());
//	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr seed_points_ext(new pcl::PointCloud<pcl::PointXYZRGBA>());
//	pcl::copyPointCloud(*cloud_tilted,*seed_idx,*seed_points_ext);
//
//	writer.write("seed_points_ext_100.pcd",*seed_points_ext);
//
//	for( int i=0;i<seed_idx->indices.size();i++)
//	{
//		int idx = seed_idx->indices.at(i);
//		cloud_tilted->at(idx).r = 0;
//		cloud_tilted->at(idx).g = 0;
//		cloud_tilted->at(idx).b = 255;
//	}
//	writer.write("cloud_with_shadow_blue.pcd",*cloud_tilted);
//	//int x = 0;
//	//Eigen::Vector3f *d = new Eigen::Vector3f();
//	//Eigen::Vector3f *hit = new Eigen::Vector3f(1,0,0);
//	////std::vector<std::vector<pcl::PointIndices::Ptr>> hit_map;
//	//pcl::PointIndices::Ptr hit_map[89][89];
//	//int hit_test[89][89];
//
//	//std::ofstream file("hit_test.txt");
//	//for( int i = 0; i < 1; i++)
//	//{
//	//	//hit_map.push_back(new std::vector<pcl::PointIndices::Ptr>);
//	//	for(int j = 0; j< 90;j++)
//	//	{
//	//		*d = Eigen::Vector3f(0,cos(j*PI/180),-sin(j*PI/180));
//	//		pcl::PointIndices::Ptr temp(new pcl::PointIndices());
//	//		//hit_map.at(i+45).at(j+45) = temp;
//	//		hit_map[i+45][j+45] = temp;
//	//		*d = Eigen::Vector3f(i,-45,j); 
//	//		hit_test[i][j] = intersection_test(&light_pos, d, hit_map[i+45][j+45], det.cloud_filtered,not_in_plane,0.1);
//	//		std::cout<<hit_test[i][j]<<",";
//	//		file<<hit_test[i][j]<<",";
//	//	}
//	//	std::cout<<";"<<std::endl;
//	//	file<<";"<<std::endl;
//	//}
//	
//
//
//
////#pragma region Normal Estimation
//
//	////create a normal distance feature estimator
//	//pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal> ne;
//
//	////set 'cloud' as input cloud.
//	//ne.setInputCloud(cloud);
//	////create a kdtree and set it as the search method of ne.
//	//pcl::search::KdTree<pcl::PointXYZ>::Ptr tree( new pcl::search::KdTree<pcl::PointXYZ>());
//	//ne.setSearchMethod(tree);
//
//	////create the output dataset.
//	//pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
//
//	////set parameter
//	//ne.setKSearch(9);
//
//	////compute features
//	//ne.compute(*cloud_normals);
//
////#pragma endregion
//
//
//
//	//pcDenoise den;
//	//den.setCloud(cloud_filtered);
//	//den.pcDecompose();
//
//
//
////#pragma region visualize point cloud and its normals
////
////	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer("My Viewer"));
////	viewer->setBackgroundColor(0,0,0);
////	viewer->initCameraParameters();
////	pcl::visualization::PointCloudColorHandlerRGBField<PointT> single_color(cloud);
////	viewer->addPointCloud<pcl::PointXYZRGBA>(cloud,single_color,"smooth");
////	
////	//viewer->addPointCloud<pcl::PointXYZRGBA >(cloud_copy,single_color,"original");
////	//viewer->addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud,cloud_normals,10,0.02,"normal");
////	//viewer->addCoordinateSystem(1.0);
////	
////	
////
////	while(!viewer->wasStopped())
////	{
////		viewer->spinOnce(100);
////		boost::this_thread::sleep (boost::posix_time::microseconds (10000));
////	}
////
////#pragma endregion
#pragma endregion

	
}


