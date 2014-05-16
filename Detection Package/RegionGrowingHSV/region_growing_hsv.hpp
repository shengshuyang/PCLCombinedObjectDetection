#ifndef PCL_REGION_GROWING_HSV_HPP_
#define PCL_REGION_GROWING_HSV_HPP_


#include "region_growing_hsv.h"
#include <fstream>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <queue>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT>
void pcl::RegionGrowingHSV<PointT, NormalT>::setPointColorThreshold(float hue_min, float sat_min, float value_min, float hue_max, float sat_max, float value_max)
{
	color_p2p_threshold_hue_min_ = hue_min;
	color_p2p_threshold_hue_max_ = hue_max;

	color_p2p_threshold_saturation_min_ = sat_min;
	color_p2p_threshold_saturation_max_ = sat_max;

	color_p2p_threshold_value_min_ = value_min;
	color_p2p_threshold_value_max_ = value_max;

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT>
void pcl::RegionGrowingHSV<PointT, NormalT>::setRegionColorThreshold(float hue_min, float sat_min, float value_min, float hue_max, float sat_max, float value_max)
{
	color_r2r_threshold_hue_min_ = hue_min;
	color_r2r_threshold_hue_max_ = hue_max;

	color_r2r_threshold_saturation_min_ = sat_min;
	color_r2r_threshold_saturation_max_ = sat_max;

	color_r2r_threshold_value_min_ = value_min;
	color_r2r_threshold_value_max_ = value_max;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT>
bool pcl::RegionGrowingHSV<PointT, NormalT>::calculateColorimetricalDifference(std::vector<float>& first_color, std::vector<float>& second_color, bool is_checking_region) const
{
	float h_max = is_checking_region ? color_r2r_threshold_hue_max_ : color_p2p_threshold_hue_max_;
	float h_min = is_checking_region ? color_r2r_threshold_hue_min_ : color_p2p_threshold_hue_min_;
	float s_max = is_checking_region ? color_r2r_threshold_saturation_max_ : color_p2p_threshold_saturation_max_;
	float s_min = is_checking_region ? color_r2r_threshold_saturation_min_ : color_p2p_threshold_saturation_min_;
	float v_max = is_checking_region ? color_r2r_threshold_value_max_ : color_p2p_threshold_value_max_;
	float v_min = is_checking_region ? color_r2r_threshold_value_min_ : color_p2p_threshold_value_min_;


	if ((first_color[0] - second_color[0]) > h_max ||
		(first_color[0] - second_color[0]) < h_min ||
		(first_color[1] - second_color[1]) > s_max ||
		(first_color[1] - second_color[1]) < s_min ||
		(first_color[2] - second_color[2]) > v_max ||
		(first_color[2] - second_color[2]) < v_min)
	{
		return false;
	}
	return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT>
bool pcl::RegionGrowingHSV<PointT, NormalT>::validatePoint(int initial_seed, int point, int nghbr, bool& is_a_seed) const
{
	is_a_seed = true;

	// check the color difference
	std::vector<float> point_color;
	point_color.resize(3, 0);
	std::vector<float> nghbr_color;
	nghbr_color.resize(3, 0);
	point_color[0] = input_->points[point].h;
	point_color[1] = input_->points[point].s;
	point_color[2] = input_->points[point].v;
	nghbr_color[0] = input_->points[nghbr].h;
	nghbr_color[1] = input_->points[nghbr].s;
	nghbr_color[2] = input_->points[nghbr].v;
	bool similar_enough = calculateColorimetricalDifference(point_color, nghbr_color, false);
	if (!similar_enough)
		return (false);


	float cosine_threshold = cosf(theta_threshold_);

	// check the angle between normals if needed
	if (normal_flag_)
	{
		float data[4];
		data[0] = input_->points[point].data[0];
		data[1] = input_->points[point].data[1];
		data[2] = input_->points[point].data[2];
		data[3] = input_->points[point].data[3];

		Eigen::Map<Eigen::Vector3f> initial_point(static_cast<float*> (data));
		Eigen::Map<Eigen::Vector3f> initial_normal(static_cast<float*> (normals_->points[point].normal));
		if (smooth_mode_flag_ == true)
		{
			Eigen::Map<Eigen::Vector3f> nghbr_normal(static_cast<float*> (normals_->points[nghbr].normal));
			float dot_product = fabsf(nghbr_normal.dot(initial_normal));
			if (dot_product < cosine_threshold)
				return (false);
		}
		else
		{
			Eigen::Map<Eigen::Vector3f> nghbr_normal(static_cast<float*> (normals_->points[nghbr].normal));
			Eigen::Map<Eigen::Vector3f> initial_seed_normal(static_cast<float*> (normals_->points[initial_seed].normal));
			float dot_product = fabsf(nghbr_normal.dot(initial_seed_normal));
			if (dot_product < cosine_threshold)
				return (false);
		}
	}

	// check the curvature if needed
	if (curvature_flag_ && normals_->points[nghbr].curvature > curvature_threshold_)
		is_a_seed = false;

	// check the residual if needed
	if (residual_flag_)
	{
		float data_p[4];
		data_p[0] = input_->points[point].data[0];
		data_p[1] = input_->points[point].data[1];
		data_p[2] = input_->points[point].data[2];
		data_p[3] = input_->points[point].data[3];
		float data_n[4];
		data_n[0] = input_->points[nghbr].data[0];
		data_n[1] = input_->points[nghbr].data[1];
		data_n[2] = input_->points[nghbr].data[2];
		data_n[3] = input_->points[nghbr].data[3];
		Eigen::Map<Eigen::Vector3f> nghbr_point(static_cast<float*> (data_n));
		Eigen::Map<Eigen::Vector3f> initial_point(static_cast<float*> (data_p));
		Eigen::Map<Eigen::Vector3f> initial_normal(static_cast<float*> (normals_->points[point].normal));
		float residual = fabsf(initial_normal.dot(initial_point - nghbr_point));
		if (residual > residual_threshold_)
			is_a_seed = false;
	}

	return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT>
void pcl::RegionGrowingHSV<PointT, NormalT>::applyRegionMergingAlgorithm()
{
	int number_of_points = static_cast<int> (indices_->size());

	// calculate color of each segment
	std::vector< std::vector<float> > segment_color;
	std::vector<float> color;
	color.resize(3, 0);
	segment_color.resize(number_of_segments_, color);

	for (int i_point = 0; i_point < number_of_points; i_point++)
	{
		int point_index = (*indices_)[i_point];
		int segment_index = point_labels_[point_index];
		segment_color[segment_index][0] += input_->points[point_index].h;
		segment_color[segment_index][1] += input_->points[point_index].s;
		segment_color[segment_index][2] += input_->points[point_index].v;
	}
	for (int i_seg = 0; i_seg < number_of_segments_; i_seg++)
	{
		segment_color[i_seg][0] = static_cast<unsigned int> (static_cast<float> (segment_color[i_seg][0]) / static_cast<float> (num_pts_in_segment_[i_seg]));
		segment_color[i_seg][1] = static_cast<unsigned int> (static_cast<float> (segment_color[i_seg][1]) / static_cast<float> (num_pts_in_segment_[i_seg]));
		segment_color[i_seg][2] = static_cast<unsigned int> (static_cast<float> (segment_color[i_seg][2]) / static_cast<float> (num_pts_in_segment_[i_seg]));
	}

	// now it is time to find out if there are segments with a similar color
	// and merge them together
	std::vector<unsigned int> num_pts_in_homogeneous_region;
	std::vector<int> num_seg_in_homogeneous_region;

	segment_labels_.resize(number_of_segments_, -1);

	float dist_thresh = distance_threshold_;
	int homogeneous_region_number = 0;
	int curr_homogeneous_region = 0;
	for (int i_seg = 0; i_seg < number_of_segments_; i_seg++)
	{
		curr_homogeneous_region = 0;
		if (segment_labels_[i_seg] == -1)
		{
			segment_labels_[i_seg] = homogeneous_region_number;
			curr_homogeneous_region = homogeneous_region_number;
			num_pts_in_homogeneous_region.push_back(num_pts_in_segment_[i_seg]);
			num_seg_in_homogeneous_region.push_back(1);
			homogeneous_region_number++;
		}
		else
			curr_homogeneous_region = segment_labels_[i_seg];

		unsigned int i_nghbr = 0;
		while (i_nghbr < region_neighbour_number_ && i_nghbr < segment_neighbours_[i_seg].size())
		{
			int index = segment_neighbours_[i_seg][i_nghbr];
			if (segment_distances_[i_seg][i_nghbr] > dist_thresh)
			{
				i_nghbr++;
				continue;
			}
			if (segment_labels_[index] == -1)
			{
				bool similar_enough = calculateColorimetricalDifference(segment_color[i_seg], segment_color[index], true);
				if (similar_enough)
				{
					segment_labels_[index] = curr_homogeneous_region;
					num_pts_in_homogeneous_region[curr_homogeneous_region] += num_pts_in_segment_[index];
					num_seg_in_homogeneous_region[curr_homogeneous_region] += 1;
				}
			}
			i_nghbr++;
		}// next neighbour
	}// next segment

	segment_color.clear();
	color.clear();

	std::vector< std::vector<int> > final_segments;
	std::vector<int> region;
	final_segments.resize(homogeneous_region_number, region);
	for (int i_reg = 0; i_reg < homogeneous_region_number; i_reg++)
	{
		final_segments[i_reg].resize(num_seg_in_homogeneous_region[i_reg], 0);
	}

	std::vector<int> counter;
	counter.resize(homogeneous_region_number, 0);
	for (int i_seg = 0; i_seg < number_of_segments_; i_seg++)
	{
		int index = segment_labels_[i_seg];
		final_segments[index][counter[index]] = i_seg;
		counter[index] += 1;
	}

	std::vector< std::vector< std::pair<float, int> > > region_neighbours;
	findRegionNeighbours(region_neighbours, final_segments);

	int final_segment_number = homogeneous_region_number;
	for (int i_reg = 0; i_reg < homogeneous_region_number; i_reg++)
	{
		if (num_pts_in_homogeneous_region[i_reg] < min_pts_per_cluster_)
		{
			if (region_neighbours[i_reg].empty())
				continue;
			int nearest_neighbour = region_neighbours[i_reg][0].second;
			if (region_neighbours[i_reg][0].first == std::numeric_limits<float>::max())
				continue;
			int reg_index = segment_labels_[nearest_neighbour];
			int num_seg_in_reg = num_seg_in_homogeneous_region[i_reg];
			for (int i_seg = 0; i_seg < num_seg_in_reg; i_seg++)
			{
				int segment_index = final_segments[i_reg][i_seg];
				final_segments[reg_index].push_back(segment_index);
				segment_labels_[segment_index] = reg_index;
			}
			final_segments[i_reg].clear();
			num_pts_in_homogeneous_region[reg_index] += num_pts_in_homogeneous_region[i_reg];
			num_pts_in_homogeneous_region[i_reg] = 0;
			num_seg_in_homogeneous_region[reg_index] += num_seg_in_homogeneous_region[i_reg];
			num_seg_in_homogeneous_region[i_reg] = 0;
			final_segment_number -= 1;

			int nghbr_number = static_cast<int> (region_neighbours[reg_index].size());
			for (int i_nghbr = 0; i_nghbr < nghbr_number; i_nghbr++)
			{
				if (segment_labels_[region_neighbours[reg_index][i_nghbr].second] == reg_index)
				{
					region_neighbours[reg_index][i_nghbr].first = std::numeric_limits<float>::max();
					region_neighbours[reg_index][i_nghbr].second = 0;
				}
			}
			nghbr_number = static_cast<int> (region_neighbours[i_reg].size());
			for (int i_nghbr = 0; i_nghbr < nghbr_number; i_nghbr++)
			{
				if (segment_labels_[region_neighbours[i_reg][i_nghbr].second] != reg_index)
				{
					std::pair<float, int> pair;
					pair.first = region_neighbours[i_reg][i_nghbr].first;
					pair.second = region_neighbours[i_reg][i_nghbr].second;
					region_neighbours[reg_index].push_back(pair);
				}
			}
			region_neighbours[i_reg].clear();
			std::sort(region_neighbours[reg_index].begin(), region_neighbours[reg_index].end(), comparePair);
		}
	}

	assembleRegions(num_pts_in_homogeneous_region, static_cast<int> (num_pts_in_homogeneous_region.size()));

	number_of_segments_ = final_segment_number;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT>
pcl::RegionGrowingHSV<PointT, NormalT>::RegionGrowingHSV() :
color_p2p_threshold_(1225.0f),
color_r2r_threshold_(10.0f),
distance_threshold_(0.05f),
region_neighbour_number_(100),
point_distances_(0),
segment_neighbours_(0),
segment_distances_(0),
segment_labels_(0),
color_r2r_threshold_hue_min_(-1),
color_r2r_threshold_hue_max_(1),
color_r2r_threshold_saturation_max_(-1),
color_r2r_threshold_saturation_min_(1),
color_r2r_threshold_value_min_(-1),
color_r2r_threshold_value_max_(1),
color_p2p_threshold_hue_min_(-1),
color_p2p_threshold_hue_max_(1),
color_p2p_threshold_saturation_min_(-1),
color_p2p_threshold_saturation_max_(1),
color_p2p_threshold_value_min_(-1),
color_p2p_threshold_value_max_(1)
{
	normal_flag_ = false;
	curvature_flag_ = false;
	residual_flag_ = false;
	min_pts_per_cluster_ = 10;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT>
pcl::RegionGrowingHSV<PointT, NormalT>::~RegionGrowingHSV()
{
	point_distances_.clear();
	segment_neighbours_.clear();
	segment_distances_.clear();
	segment_labels_.clear();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> float
pcl::RegionGrowingHSV<PointT, NormalT>::getPointColorThreshold() const
{
	return (powf(color_p2p_threshold_, 0.5f));
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> float
pcl::RegionGrowingHSV<PointT, NormalT>::getRegionColorThreshold() const
{
	return (powf(color_r2r_threshold_, 0.5f));
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> float
pcl::RegionGrowingHSV<PointT, NormalT>::getDistanceThreshold() const
{
	return (powf(distance_threshold_, 0.5f));
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> void
pcl::RegionGrowingHSV<PointT, NormalT>::setDistanceThreshold(float thresh)
{
	distance_threshold_ = thresh * thresh;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> unsigned int
pcl::RegionGrowingHSV<PointT, NormalT>::getNumberOfRegionNeighbours() const
{
	return (region_neighbour_number_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> void
pcl::RegionGrowingHSV<PointT, NormalT>::setNumberOfRegionNeighbours(unsigned int nghbr_number)
{
	region_neighbour_number_ = nghbr_number;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> bool
pcl::RegionGrowingHSV<PointT, NormalT>::getNormalTestFlag() const
{
	return (normal_flag_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> void
pcl::RegionGrowingHSV<PointT, NormalT>::setNormalTestFlag(bool value)
{
	normal_flag_ = value;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> void
pcl::RegionGrowingHSV<PointT, NormalT>::setCurvatureTestFlag(bool value)
{
	curvature_flag_ = value;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> void
pcl::RegionGrowingHSV<PointT, NormalT>::setResidualTestFlag(bool value)
{
	residual_flag_ = value;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> void
pcl::RegionGrowingHSV<PointT, NormalT>::extract(std::vector <pcl::PointIndices>& clusters)
{
	clusters_.clear();
	clusters.clear();
	point_neighbours_.clear();
	point_labels_.clear();
	num_pts_in_segment_.clear();
	point_distances_.clear();
	segment_neighbours_.clear();
	segment_distances_.clear();
	segment_labels_.clear();
	number_of_segments_ = 0;

	bool segmentation_is_possible = initCompute();
	if (!segmentation_is_possible)
	{
		deinitCompute();
		return;
	}

	segmentation_is_possible = prepareForSegmentation();
	if (!segmentation_is_possible)
	{
		deinitCompute();
		return;
	}
	std::cout<< "Start find point neighbours"<<std::endl;
	findPointNeighbours();

	std::cout<< "Start region growing"<<std::endl;
	applySmoothRegionGrowingAlgorithm();

	std::cout<< "assemble regions"<<std::endl;
	RegionGrowing<PointT, NormalT>::assembleRegions();

	std::cout<< "Start find region neighbours"<<std::endl;
	findSegmentNeighbours();

	std::cout<< "Start merging region"<<std::endl;
	applyRegionMergingAlgorithm();

	std::vector<pcl::PointIndices>::iterator cluster_iter = clusters_.begin();
	std::cout << "Num of clusters is: " << clusters_.size() <<std::endl;
	while (cluster_iter != clusters_.end())
	{
		if (cluster_iter->indices.size() < min_pts_per_cluster_ || cluster_iter->indices.size() > max_pts_per_cluster_)
		{
			cluster_iter = clusters_.erase(cluster_iter);
		}
		else
			cluster_iter++;
	}

	clusters.reserve(clusters_.size());
	std::copy(clusters_.begin(), clusters_.end(), std::back_inserter(clusters));
	findSegmentNeighbours();
	deinitCompute();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> bool
pcl::RegionGrowingHSV<PointT, NormalT>::prepareForSegmentation()
{
	// if user forgot to pass point cloud or if it is empty
	if (input_->points.size() == 0)
		return (false);

	// if normal/smoothness test is on then we need to check if all needed variables and parameters
	// were correctly initialized
	if (normal_flag_)
	{
		// if user forgot to pass normals or the sizes of point and normal cloud are different
		if (normals_ == 0 || input_->points.size() != normals_->points.size())
			return (false);
	}

	// if residual test is on then we need to check if all needed parameters were correctly initialized
	if (residual_flag_)
	{
		if (residual_threshold_ <= 0.0f)
			return (false);
	}

	// if curvature test is on ...
	// if (curvature_flag_)
	// {
	//   in this case we do not need to check anything that related to it
	//   so we simply commented it
	// }

	// here we check the parameters related to color-based segmentation
	if (region_neighbour_number_ == 0 || color_p2p_threshold_ < 0.0f || color_r2r_threshold_ < 0.0f || distance_threshold_ < 0.0f)
		return (false);

	// from here we check those parameters that are always valuable
	if (neighbour_number_ == 0)
		return (false);

	// if user didn't set search method
	if (!search_)
		search_.reset(new pcl::search::KdTree<PointT>);

	if (indices_)
	{
		if (indices_->empty())
			PCL_ERROR("[pcl::RegionGrowingHSV::prepareForSegmentation] Empty given indices!\n");
		search_->setInputCloud(input_, indices_);
	}
	else
		search_->setInputCloud(input_);

	return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> void
pcl::RegionGrowingHSV<PointT, NormalT>::findPointNeighbours()
{
	int point_number = static_cast<int> (indices_->size());
	std::vector<int> neighbours;
	std::vector<float> distances;

	point_neighbours_.resize(input_->points.size(), neighbours);
	point_distances_.resize(input_->points.size(), distances);

	for (int i_point = 0; i_point < point_number; i_point++)
	{
		int point_index = (*indices_)[i_point];
		neighbours.clear();
		distances.clear();
		search_->nearestKSearch(i_point, region_neighbour_number_, neighbours, distances);
		point_neighbours_[point_index].swap(neighbours);
		point_distances_[point_index].swap(distances);
	}
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> void
pcl::RegionGrowingHSV<PointT, NormalT>::findSegmentNeighbours()
{
	std::vector<int> neighbours;
	std::vector<float> distances;
	segment_neighbours_.clear();
	segment_distances_.clear();
	num_pts_in_segment_.clear();
	number_of_segments_ = clusters_.size();
	for (int i=0;i<number_of_segments_;i++)
	{
		num_pts_in_segment_.push_back(clusters_[i].indices.size());
	}

	segment_neighbours_.resize(number_of_segments_, neighbours);
	segment_distances_.resize(number_of_segments_, distances);
	std::vector<pcl::PointIndices>::iterator seg_itr;
	int count = 0;
	for (int i=0; i< point_labels_.size();i++)
	{
		point_labels_[i] = 0;

	}
	for (seg_itr = clusters_.begin();seg_itr < clusters_.end(); ++seg_itr)
	{
		for (int j = 0; j < seg_itr->indices.size(); j++ )
		{
			int idx = seg_itr->indices.at(j);
			point_labels_[idx] = count;
		}
		count++;
	}

	for (int i_seg = 0; i_seg < number_of_segments_; i_seg++)
	{
		if (i_seg == 33)
		{
			std::cout <<"arriving 33"<<std::endl;
		}
		std::vector<int> nghbrs;
		std::vector<float> dist;
		findRegionsKNN(i_seg, region_neighbour_number_, nghbrs, dist);
		segment_neighbours_[i_seg].swap(nghbrs);
		segment_distances_[i_seg].swap(dist);
	}
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> void
pcl::RegionGrowingHSV<PointT, NormalT>::findRegionsKNN(int index, int nghbr_number, std::vector<int>& nghbrs, std::vector<float>& dist)
{
	std::vector<float> distances;
	float max_dist = std::numeric_limits<float>::max();
	distances.resize(clusters_.size(), max_dist);

	int number_of_points = num_pts_in_segment_[index];
	//loop throug every point in this segment and check neighbours
	for (int i_point = 0; i_point < number_of_points; i_point++)
	{
		int point_index = clusters_[index].indices[i_point];
		int number_of_neighbours = static_cast<int> (point_neighbours_[point_index].size());
		//loop throug every neighbour of the current point, find out to which segment it belongs
		//and if it belongs to neighbouring segment and is close enough then remember segment and its distance
		for (int i_nghbr = 0; i_nghbr < number_of_neighbours; i_nghbr++)
		{
			// find segment
			int segment_index = -1;
			segment_index = point_labels_[point_neighbours_[point_index][i_nghbr]];

			if (segment_index != index)
			{
				// try to push it to the queue
				if (distances[segment_index] > point_distances_[point_index][i_nghbr])
					distances[segment_index] = point_distances_[point_index][i_nghbr];
			}
		}
	}// next point

	std::priority_queue<std::pair<float, int> > segment_neighbours;
	for (int i_seg = 0; i_seg < number_of_segments_; i_seg++)
	{
		if (distances[i_seg] < max_dist)
		{
			segment_neighbours.push(std::make_pair(distances[i_seg], i_seg));
			if (int(segment_neighbours.size()) > nghbr_number)
				segment_neighbours.pop();
		}
	}

	int size = std::min<int>(static_cast<int> (segment_neighbours.size()), nghbr_number);
	nghbrs.resize(size, 0);
	dist.resize(size, 0);
	int counter = 0;
	while (!segment_neighbours.empty() && counter < nghbr_number)
	{
		dist[counter] = segment_neighbours.top().first;
		nghbrs[counter] = segment_neighbours.top().second;
		segment_neighbours.pop();
		counter++;
	}
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> void
pcl::RegionGrowingHSV<PointT, NormalT>::findRegionNeighbours(std::vector< std::vector< std::pair<float, int> > >& neighbours_out, std::vector< std::vector<int> >& regions_in)
{
	int region_number = static_cast<int> (regions_in.size());
	neighbours_out.clear();
	neighbours_out.resize(region_number);

	for (int i_reg = 0; i_reg < region_number; i_reg++)
	{
		int segment_num = static_cast<int> (regions_in[i_reg].size());
		neighbours_out[i_reg].reserve(segment_num * region_neighbour_number_);
		for (int i_seg = 0; i_seg < segment_num; i_seg++)
		{
			int curr_segment = regions_in[i_reg][i_seg];
			int nghbr_number = static_cast<int> (segment_neighbours_[curr_segment].size());
			std::pair<float, int> pair;
			for (int i_nghbr = 0; i_nghbr < nghbr_number; i_nghbr++)
			{
				int segment_index = segment_neighbours_[curr_segment][i_nghbr];
				if (segment_distances_[curr_segment][i_nghbr] == std::numeric_limits<float>::max())
					continue;
				if (segment_labels_[segment_index] != i_reg)
				{
					pair.first = segment_distances_[curr_segment][i_nghbr];
					pair.second = segment_index;
					neighbours_out[i_reg].push_back(pair);
				}
			}// next neighbour
		}// next segment
		std::sort(neighbours_out[i_reg].begin(), neighbours_out[i_reg].end(), comparePair);
	}// next homogeneous region
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> void
pcl::RegionGrowingHSV<PointT, NormalT>::assembleRegions(std::vector<unsigned int>& num_pts_in_region, int num_regions)
{
	clusters_.clear();
	pcl::PointIndices segment;
	clusters_.resize(num_regions, segment);
	for (int i_seg = 0; i_seg < num_regions; i_seg++)
	{
		clusters_[i_seg].indices.resize(num_pts_in_region[i_seg]);
	}

	std::vector<int> counter;
	counter.resize(num_regions, 0);
	int point_number = static_cast<int> (indices_->size());
	for (int i_point = 0; i_point < point_number; i_point++)
	{
		int point_index = (*indices_)[i_point];
		int index = point_labels_[point_index];
		index = segment_labels_[index];
		clusters_[index].indices[counter[index]] = point_index;
		counter[index] += 1;
	}

	// now we need to erase empty regions
	if (clusters_.empty()) return;

	std::vector<pcl::PointIndices>::iterator itr1, itr2;
	itr1 = clusters_.begin();
	itr2 = clusters_.end() - 1;

	while (itr1 < itr2)
	{
		while (!(itr1->indices.empty()) && itr1 < itr2) itr1++;
		while (itr2->indices.empty() && itr1 < itr2)    itr2--;
		if (itr1 != itr2) itr1->indices.swap(itr2->indices);
	}

	if (itr2->indices.empty())  clusters_.erase(itr2, clusters_.end());

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> void
pcl::RegionGrowingHSV<PointT, NormalT>::getSegmentFromPoint(int index, pcl::PointIndices& cluster)
{
	cluster.indices.clear();

	bool segmentation_is_possible = initCompute();
	if (!segmentation_is_possible)
	{
		deinitCompute();
		return;
	}

	// first of all we need to find out if this point belongs to cloud
	bool point_was_found = false;
	int number_of_points = static_cast <int> (indices_->size());
	for (size_t point = 0; point < number_of_points; point++)
	if ((*indices_)[point] == index)
	{
		point_was_found = true;
		break;
	}

	if (point_was_found)
	{
		if (clusters_.empty())
		{
			clusters_.clear();
			point_neighbours_.clear();
			point_labels_.clear();
			num_pts_in_segment_.clear();
			point_distances_.clear();
			segment_neighbours_.clear();
			segment_distances_.clear();
			segment_labels_.clear();
			number_of_segments_ = 0;

			segmentation_is_possible = prepareForSegmentation();
			if (!segmentation_is_possible)
			{
				deinitCompute();
				return;
			}

			findPointNeighbours();
			applySmoothRegionGrowingAlgorithm();
			RegionGrowing<PointT, NormalT>::assembleRegions();

			findSegmentNeighbours();
			applyRegionMergingAlgorithm();
		}
		// if we have already made the segmentation, then find the segment
		// to which this point belongs
		std::vector <pcl::PointIndices>::iterator i_segment;
		for (i_segment = clusters_.begin(); i_segment != clusters_.end(); i_segment++)
		{
			bool segment_was_found = false;
			for (size_t i_point = 0; i_point < i_segment->indices.size(); i_point++)
			{
				if (i_segment->indices[i_point] == index)
				{
					segment_was_found = true;
					cluster.indices.clear();
					cluster.indices.reserve(i_segment->indices.size());
					std::copy(i_segment->indices.begin(), i_segment->indices.end(), std::back_inserter(cluster.indices));
					break;
				}
			}
			if (segment_was_found)
			{
				break;
			}
		}// next segment
	}// end if point was found

	deinitCompute();
}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> int
pcl::RegionGrowingHSV<PointT, NormalT>::writeClustersToFile(std::string path, std::vector<int> plane_labels)
{

	std::ofstream of (path);

	std::vector<pcl::PointIndices>::iterator itr;
	int i = 0;

	for(itr = clusters_.begin(); itr < clusters_.end();itr++)
	{
		std::vector<int>::iterator idx;
		Eigen::Vector3f center(0,0,0);
		Eigen::Vector3f color(0,0,0);
		double avg_label = 0;
  		for (idx = itr->indices.begin(); idx < itr->indices.end();idx++)
		{
			PointT p = input_->at(*idx);
			center[0] += p.x;
			center[1] += p.y;
			center[2] += p.z;

			color[0] += p.h;
			color[1] += p.s;
			color[2] += p.v;

			avg_label += plane_labels.at(*idx);
		}
		center /= itr->indices.size();
		color  /= itr->indices.size();
		avg_label  /= itr->indices.size();
		of << center[0] << " " << center[1] << " " << center[2] <<" " 
		   << color[0]  << " " << color[1]  << " " << color[2] <<" ";

		of << (int)avg_label <<" ";
		of << itr->indices.size() << " ";

		for(int j=0; j < segment_neighbours_[i].size();j++)
		{
			of << segment_neighbours_[i][j] << " ";
		}
		for( int j=segment_neighbours_[i].size(); j< 10; j++)
		{

			of << "-1 ";
		}
		of <<";" <<std::endl;

		cluster_centers_.push_back(center);
		cluster_colors_.push_back(color);

		i++;
	}

	of.close();
	return (0);

}
template <typename PointT, typename NormalT> pcl::PointCloud<pcl::PointXYZRGB>::Ptr
pcl::RegionGrowingHSV<PointT, NormalT>::getShadowCloud ()
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	//colored_cloud = (new pcl::PointCloud<pcl::PointXYZRGB>)->makeShared ();
	//colored_cloud->width = input_->width;
	//colored_cloud->height = input_->height;
	//colored_cloud->is_dense = input_->is_dense;

	std::vector< pcl::PointIndices >::iterator i_segment;
	int i=0;
	for (i_segment = clusters_.begin (); i_segment != clusters_.end (); i_segment++)
	{
		std::vector<int>::iterator i_point;
		for (i_point = i_segment->indices.begin (); i_point != i_segment->indices.end (); i_point++)
		{
			int index;
			index = *i_point;
			//colored_cloud->points[index].r = segment_shadow_labels_.at(i) == 1 ? 255:0;
			//colored_cloud->points[index].g = segment_shadow_labels_.at(i) == 1 ? 0:255;
			//colored_cloud->points[index].b = 0;

			pcl::PointXYZRGB point;
			point.x = *(input_->points[index].data);
			point.y = *(input_->points[index].data + 1);
			point.z = *(input_->points[index].data + 2);
			point.r = 100;
			point.g = 100;
			point.b = 100;
			if (segment_shadow_labels_.at(i)==0)//shadow
			{
				point.r = 255;
			}
			if (segment_shadow_labels_.at(i)==1)//not shadow
			{
				point.g = 255;
			}
			if (segment_shadow_labels_.at(i)==2)//object
			{
				point.b = 255;
			}
			colored_cloud->points.push_back (point);
		}
		i++;
	}
	colored_cloud->height = colored_cloud->points.size();
	colored_cloud->width = 1;


	return (colored_cloud);
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> int
pcl::RegionGrowingHSV<PointT, NormalT>::readShadowLabelsFromFile(std::string path)
{

	std::cout <<"haven't implemented yet."<<std::endl;

	std::ifstream ss(path);
	char label;
	while( !ss.eof())
	{
		label = ss.get();
		if( label == '0')
		{
			this->segment_shadow_labels_.push_back(0);
		}
		else if(label == '1')
		{

			this->segment_shadow_labels_.push_back(1);
		}
		else if(label == '2')
		{

			this->segment_shadow_labels_.push_back(2);
		}
		
	}
	return (0);
}


#endif