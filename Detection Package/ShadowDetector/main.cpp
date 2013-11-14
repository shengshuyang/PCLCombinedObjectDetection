#include <pcl/point_types.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/organized_edge_detection.h>
#include <Eigen/StdVector>
typedef pcl::PointXYZRGBA PointT;
typedef pcl::Normal NormalT;

int main(int argc, char** argv)
{
	if(argc != 2)
	{
		printf("%d Format: PCLShadowDetector path\n\n",argc);
		return -1;
	}
	printf("finished parsing cloud\n");
	pcl::PointCloud<PointT>::Ptr cloud  (new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr cloud2 (new pcl::PointCloud<PointT>);
	pcl::io::loadPCDFile<PointT>(argv[1],*cloud);

	printf("finished reading input point cloud\n");
	pcl::NormalEstimation<PointT,NormalT> ne;//(new pcl::NormalEstimation<PointT,PointT>);
	pcl::search::KdTree<PointT>::Ptr kdtree (new pcl::search::KdTree<PointT>());
	pcl::PointCloud<NormalT>::Ptr normal (new pcl::PointCloud<NormalT>());
	ne.setInputCloud(cloud);
	ne.setSearchMethod(kdtree);
	ne.setRadiusSearch(0.02);
	ne.compute(*normal);

	pcl::OrganizedEdgeFromRGBNormals<pcl::PointXYZRGBA, pcl::Normal, pcl::Label> oed;
	oed.setInputNormals (normal);
	oed.setInputCloud (cloud);
	oed.setDepthDisconThreshold (0.02); // 2cm
	oed.setMaxSearchNeighbors (50);
	pcl::PointCloud<pcl::Label> labels;
	std::vector<pcl::PointIndices> label_indices;
	oed.compute (labels, label_indices);

	pcl::PointCloud<PointT>::Ptr occluding_edges (new pcl::PointCloud<PointT>),
			occluded_edges       (new pcl::PointCloud<PointT>),
			boundary_edges       (new pcl::PointCloud<PointT>),
			high_curvature_edges (new pcl::PointCloud<PointT>),
			rgb_edges            (new pcl::PointCloud<PointT>);

	pcl::copyPointCloud (*cloud, label_indices[0].indices, *boundary_edges);
	pcl::copyPointCloud (*cloud, label_indices[1].indices, *occluding_edges);
	pcl::copyPointCloud (*cloud, label_indices[2].indices, *occluded_edges);
	pcl::copyPointCloud (*cloud, label_indices[3].indices, *high_curvature_edges);
	pcl::copyPointCloud (*cloud, label_indices[4].indices, *rgb_edges);
	
	pcl::io::savePCDFile<PointT>("./output/boundary_edge.pcd",*boundary_edges);
	pcl::io::savePCDFile<PointT>("./output/occluding_edges.pcd",*occluding_edges);
	pcl::io::savePCDFile<PointT>("./output/occluded_edges.pcd",*occluded_edges);
	pcl::io::savePCDFile<PointT>("./output/high_curvature_edges.pcd",*high_curvature_edges);
	pcl::io::savePCDFile<PointT>("./output/rgb_edges2.pcd",*rgb_edges);
	
	std::vector<int> curvature_edge_indices = label_indices[0].indices;
	std::vector<int> rgb_edge_indices = label_indices[4].indices;//temporary code
	std::vector<int> seed_indices ;

	printf("start finding seeds\n");
	for(int k=1;k<4;k++)//iterate over 3 types of edges
	{
		for( int i=0; i < label_indices[k].indices.size();i++)//iterate of the k'th type edge
		{
			for( int j=0; j< rgb_edge_indices.size();j++)//compare the j'th point in rgb_edge to the i'th
			{
				if(label_indices[k].indices.at(i) == rgb_edge_indices.at(j) )
				{
					seed_indices.push_back(label_indices[k].indices.at(i));
				}
			}
		}
	}

	printf("start expanding\n");
	std::vector<int> final_indices = seed_indices;
	std::cout << "total size: "<<rgb_edge_indices.size()<<std::endl;

	for(int itr = 0;itr<1;itr++)//repeat the whole process. currently set to one.
	{
		for( int i=0;i<rgb_edge_indices.size();i++)
		{
			std::cout << "Current size: "<<final_indices.size()<<std::endl;
			for(int j=0;j<seed_indices.size();j++)
			{
				Eigen::Vector3i p1 = cloud->at(rgb_edge_indices.at(i),1).getRGBVector3i();
				Eigen::Vector3i p2 = cloud->at(seed_indices.at(j),1).getRGBVector3i();
				Eigen::Vector3i pd = p1-p2;
				int d = pd.squaredNorm();

				if(d<50)
				{
					std::cout<<"Distance is: "<<d<<std::endl;
					final_indices.push_back(rgb_edge_indices.at(i));
					break;
				}
			}
		}
		seed_indices = final_indices;
	}


	printf("finished expanding\n");

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr final_edges (new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::copyPointCloud(*cloud, final_indices, *final_edges);
	pcl::io::savePCDFile<PointT>("./output/final_edges.pcd",*final_edges);

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr seed_edges (new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::copyPointCloud(*cloud, seed_indices, *seed_edges);
	pcl::io::savePCDFile<PointT>("./output/seed_edges.pcd",*seed_edges);


	//==========================visualization=========================================
	pcl::visualization::PCLVisualizer viewer("viewer1",true);
	pcl::visualization::PointCloudColorHandlerRGBField<PointT> color(cloud);
	viewer.addPointCloud<PointT>(cloud,color,"cloud1");
	//viewer.addPointCloudNormals<PointT,NormalT>(cloud,normal,10,0.02,"normal");
	pcl::visualization::PointCloudColorHandlerCustom< PointT > color_edge(255,0,0);
	viewer.addPointCloud<PointT>(final_edges,color_edge,"edges1");
	viewer.setCameraPosition(1,1,0,0,0,0,0);
	while(!viewer.wasStopped())
	{
		viewer.spinOnce();
	}
	
	return(0);


}
