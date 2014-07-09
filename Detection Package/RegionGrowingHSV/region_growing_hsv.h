#ifndef PCL_REGION_GROWING_HSV_H_
#define PCL_REGION_GROWING_HSV_H_

#include <pcl/segmentation/region_growing.h>
#include <pcl/point_types.h>
namespace pcl
{


  template <typename PointT, typename NormalT = pcl::Normal>
  class PCL_EXPORTS RegionGrowingHSV : public RegionGrowing<PointT, NormalT>
  {
   public:
    using RegionGrowing<PointT, NormalT>::input_;
    using RegionGrowing<PointT, NormalT>::indices_;
    using RegionGrowing<PointT, NormalT>::initCompute;
    using RegionGrowing<PointT, NormalT>::deinitCompute;
    using RegionGrowing<PointT, NormalT>::normals_;
    using RegionGrowing<PointT, NormalT>::normal_flag_;
    using RegionGrowing<PointT, NormalT>::curvature_flag_;
    using RegionGrowing<PointT, NormalT>::residual_flag_;
    using RegionGrowing<PointT, NormalT>::residual_threshold_;
    using RegionGrowing<PointT, NormalT>::neighbour_number_;
    using RegionGrowing<PointT, NormalT>::search_;
    using RegionGrowing<PointT, NormalT>::min_pts_per_cluster_;
    using RegionGrowing<PointT, NormalT>::max_pts_per_cluster_;
    using RegionGrowing<PointT, NormalT>::smooth_mode_flag_;
    using RegionGrowing<PointT, NormalT>::theta_threshold_;
    using RegionGrowing<PointT, NormalT>::curvature_threshold_;
    using RegionGrowing<PointT, NormalT>::point_neighbours_;
    using RegionGrowing<PointT, NormalT>::point_labels_;
    using RegionGrowing<PointT, NormalT>::num_pts_in_segment_;
    using RegionGrowing<PointT, NormalT>::clusters_;
    using RegionGrowing<PointT, NormalT>::number_of_segments_;
    using RegionGrowing<PointT, NormalT>::applySmoothRegionGrowingAlgorithm;
    using RegionGrowing<PointT, NormalT>::assembleRegions;

  public:
    /** \brief Default constructor */
    RegionGrowingHSV();

    /** \brief Default destructor */
    ~RegionGrowingHSV();

    /** \brief Returns the color threshold value used for testing if points belong to the same region. */
    float
      getPointColorThreshold() const;

    /** \brief This method specifies the threshold value for color test between the points.
    * This kind of testing is made at the first stage of the algorithm(region growing).
    * If the difference between points color is less than threshold value, then they are considered
    * to be in the same region.
    * \param[in] thresh new threshold value for color test
    */
    void setPointColorThreshold(float hue_min, float sat_min, float value_min, float hue_max, float sat_max, float value_max);

    /** \brief Returns the color threshold value used for testing if regions can be merged. */
    float
      getRegionColorThreshold() const;

    /** \brief This method specifies the threshold value for color test between the regions.
    * This kind of testing is made at the second stage of the algorithm(region merging).
    * If the difference between segments color is less than threshold value, then they are merged together.
    * \param[in] thresh new threshold value for color test
    */
    void setRegionColorThreshold(float hue_min, float sat_min, float value_min, float hue_max, float sat_max, float value_max);

    /** \brief Returns the distance threshold. If the distance between two points is less or equal to
    * distance threshold value, then those points assumed to be neighbouring points.
    */
    float
      getDistanceThreshold() const;

    /** \brief Allows to set distance threshold.
    * \param[in] thresh new threshold value for neighbour test
    */
    void
      setDistanceThreshold(float thresh);

    /** \brief Returns the number of nearest neighbours used for searching K nearest segments.
    * Note that here it refers to the segments(not the points).
    */
    unsigned int
      getNumberOfRegionNeighbours() const;

    /** \brief This method allows to set the number of neighbours that is used for finding
    * neighbouring segments. Neighbouring segments are needed for the merging process.
    * \param[in] nghbr_number the number of neighbouring segments to find
    */
    void
      setNumberOfRegionNeighbours(unsigned int nghbr_number);

    /** \brief Returns the flag that signalize if the smoothness test is turned on/off. */
    bool
      getNormalTestFlag() const;

    /** \brief
    * Allows to turn on/off the smoothness test.
    * \param[in] value new value for normal/smoothness test. If set to true then the test will be turned on
    */
    void
      setNormalTestFlag(bool value);

    /** \brief Allows to turn on/off the curvature test.
    * \param[in] value new value for curvature test. If set to true then the test will be turned on
    */
    virtual void
      setCurvatureTestFlag(bool value);

    /** \brief
    * Allows to turn on/off the residual test.
    * \param[in] value new value for residual test. If set to true then the test will be turned on
    */
    virtual void
      setResidualTestFlag(bool value);

    /** \brief This method launches the segmentation algorithm and returns the clusters that were
    * obtained during the segmentation.
    * \param[out] clusters clusters that were obtained. Each cluster is an array of point indices.
    */
    virtual void
      extract(std::vector <pcl::PointIndices>& clusters);

    /** \brief For a given point this function builds a segment to which it belongs and returns this segment.
    * \param[in] index index of the initial point which will be the seed for growing a segment.
    */
    virtual void
      getSegmentFromPoint(int index, pcl::PointIndices& cluster);

    /** \brief Write cluster centers, number of points and average color to a file
    * \param[in] path location of desired output file.
    */
    int
      writeClustersToFile(std::string path, std::vector<int> plane_labels);

    /** \brief Read cluster labels (0 or 1) from a file
    * \param[in] path location of label file.
    */
    int
      readShadowLabelsFromFile(std::string path);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr 
      getShadowCloud ();

    std::vector<Eigen::Vector3f> cluster_centers_;
    std::vector<Eigen::Vector3f> cluster_colors_;

  protected:

    /** \brief This method simply checks if it is possible to execute the segmentation algorithm with
    * the current settings. If it is possible then it returns true.
    */
    virtual bool
      prepareForSegmentation();

    /** \brief This method finds KNN for each point and saves them to the array
    * because the algorithm needs to find KNN a few times.
    */
    virtual void
      findPointNeighbours();

    /** \brief This method simply calls the findRegionsKNN for each segment and
    * saves the results for later use.
    */
    void
      findSegmentNeighbours();

    /** \brief This method finds K nearest neighbours of the given segment.
    * \param[in] index index of the segment for which neighbours will be found
    * \param[in] nghbr_number the number of neighbours to find
    * \param[out] nghbrs the array of indices of the neighbours that were found
    * \param[out] dist the array of distances to the corresponding neighbours
    */
    void
      findRegionsKNN(int index, int nghbr_number, std::vector<int>& nghbrs, std::vector<float>& dist);

    /** \brief This function implements the merging algorithm described in the article
    * "Color-based segmentation of point clouds"
    * by Qingming Zhan, Yubin Liang, Yinghui Xiao
    */
    void
      applyRegionMergingAlgorithm();

    /** \brief check color similarity between two points or two regions. If is_checking_region is set to true, thresholds for regions will be used
    * otherwise threshold for points will be used.
    **/
    bool calculateColorimetricalDifference(std::vector<float>& first_color, std::vector<float>& second_color, bool is_checking_region) const;

    /** \brief This method assembles the array containing neighbours of each homogeneous region.
    * Homogeneous region is the union of some segments. This array is used when the regions
    * with a few points need to be merged with the neighbouring region.
    * \param[out] neighbours_out vector of lists of neighbours for every homogeneous region
    * \param[in] regions_in vector of lists, each list contains indices of segments that belong
    * to the corresponding homogeneous region.
    */
    void
      findRegionNeighbours(std::vector< std::vector< std::pair<float, int> > >& neighbours_out, std::vector< std::vector<int> >& regions_in);

    /** \brief This function simply assembles the regions from list of point labels.
    * \param[in] num_pts_in_region for each final region it stores the corresponding number of points in it
    * \param[in] num_regions number of regions to assemble
    */
    void
      assembleRegions(std::vector<unsigned int>& num_pts_in_region, int num_regions);

    /** \brief This function is checking if the point with index 'nghbr' belongs to the segment.
    * If so, then it returns true. It also checks if this point can serve as the seed.
    * \param[in] initial_seed index of the initial point that was passed to the growRegion() function
    * \param[in] point index of the current seed point
    * \param[in] nghbr index of the point that is neighbour of the current seed
    * \param[out] is_a_seed this value is set to true if the point with index 'nghbr' can serve as the seed
    */
    virtual bool validatePoint(int initial_seed, int point, int nghbr, bool& is_a_seed)  const;


  protected:

      float color_r2r_threshold_hue_min_;
      float color_r2r_threshold_hue_max_;

      float color_r2r_threshold_saturation_max_;
      float color_r2r_threshold_saturation_min_;

      float color_r2r_threshold_value_min_;
      float color_r2r_threshold_value_max_;

      float color_p2p_threshold_hue_min_;
      float color_p2p_threshold_hue_max_;

      float color_p2p_threshold_saturation_min_;
      float color_p2p_threshold_saturation_max_;

      float color_p2p_threshold_value_min_;
      float color_p2p_threshold_value_max_;

    /** \brief Thershold used in color test for points. */
    float color_p2p_threshold_;

    /** \brief Thershold used in color test for regions. */
    float color_r2r_threshold_;

    /** \brief Threshold that tells which points we need to assume neighbouring. */
    float distance_threshold_;

    /** \brief Number of neighbouring segments to find. */
    unsigned int region_neighbour_number_;

    /** \brief Stores distances for the point neighbours from point_neighbours_ */
    std::vector< std::vector<float> > point_distances_;

    /** \brief Stores the neighboures for the corresponding segments. */
    std::vector< std::vector<int> > segment_neighbours_;

    /** \brief Stores distances for the segment neighbours from segment_neighbours_ */
    std::vector< std::vector<float> > segment_distances_;

    /** \brief Stores new indices for segments that were obtained at the region growing stage. */
    std::vector<int> segment_labels_;

    /** \brief Stores new indices for segments that were obtained at the region growing stage. */
    std::vector<int> segment_shadow_labels_;
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  };




}

#endif