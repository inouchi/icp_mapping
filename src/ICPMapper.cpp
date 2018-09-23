#include "ICPMapper.hpp"

ICPMapper::ICPMapper(ros::NodeHandle* nodeHandlePtr, ros::NodeHandle* localNodeHandlePtr)
  : nodeHandlePtr_(nodeHandlePtr),
    localNodeHandlePtr_(localNodeHandlePtr),
    originalCloud_(new pcl::PointCloud<pcl::PointXYZRGB>),
    previousIcpResult_(new pcl::PointCloud<pcl::PointXYZRGB>),
    currentModel_(new pcl::PointCloud<pcl::PointXYZRGB>),
    previousModel_(new pcl::PointCloud<pcl::PointXYZRGB>)
{
  // Initialize Sibscriber and Publishers
  cloudSub_ = nodeHandlePtr_->subscribe("/kinect2/sd/points", 1, &ICPMapper::cloudCb, this);
  modelPub_ = nodeHandlePtr->advertise<sensor_msgs::PointCloud2>(ros::this_node::getName() + "/model", 1);

  generateModelSrv_ = localNodeHandlePtr_->advertiseService("generate_model", &ICPMapper::generateModel, this);
  restorePreviousModelSrv_ = localNodeHandlePtr_->advertiseService("restore_previous_model", &ICPMapper::restorePreviousModel, this);

  //Set ICP parameters
  localNodeHandlePtr_->getParam("max_iterations", icpParam_.maxIterations);
  localNodeHandlePtr_->getParam("max_correspondence_distance", icpParam_.maxCorrespondenceDistance);
  localNodeHandlePtr_->getParam("transformation_epsilon", icpParam_.transformationEpsilon);
  localNodeHandlePtr_->getParam("euclidean_fitness_epsilon", icpParam_.euclideanFitnessEpsilon);

  // Set filter parameters
  localNodeHandlePtr_->getParam("filter_leaf_size", filter_.leafSizes);
  localNodeHandlePtr_->getParam("filter_min_axis", filter_.minAxis);
  localNodeHandlePtr_->getParam("filter_max_axis", filter_.maxAxis);
  localNodeHandlePtr_->getParam("stddev_mul_thresh" , filter_.stddevMulThresh);

  // Set distance value for using in the function of rotatePointCloud
  if (!localNodeHandlePtr->getParam("distance", distance_))  distance_ = 1.0f;
}
 


ICPMapper::~ICPMapper()
{
  delete nodeHandlePtr_;
  delete localNodeHandlePtr_;
}


void ICPMapper::cloudCb(const sensor_msgs::PointCloud2ConstPtr& input)
{
  // Convert sensor_msgs::PointCloud2 into pcl::PointCloud<T>
  // Set up-to-date a cloud data from Kinect
  pcl::fromROSMsg(*input, *originalCloud_);
}


void ICPMapper::matching(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr target, 
                         pcl::PointCloud<pcl::PointXYZRGB>::Ptr output)
{
  if (input == nullptr || output == target)
  {
    output = nullptr;
    return;
  }

  // Down sampling
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr filteredTarget(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::ApproximateVoxelGrid<pcl::PointXYZRGB> sor;  
  sor.setInputCloud(target);
  sor.setLeafSize(filter_.leafSizes[0], filter_.leafSizes[1], filter_.leafSizes[2]);
  sor.filter(*filteredTarget);

  pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
  icp.setInputCloud(input);
  icp.setInputTarget(filteredTarget);
 
  // Set the max correspondence distance (e.g., correspondences with higher distances will be ignored)
  icp.setMaxCorrespondenceDistance(icpParam_.maxCorrespondenceDistance);
  // Set the maximum number of iterations (criterion 1)
  icp.setMaximumIterations(icpParam_.maxIterations);
  // Set the transformation epsilon (criterion 2)
  icp.setTransformationEpsilon(icpParam_.transformationEpsilon);
  // Set the euclidean distance difference epsilon (criterion 3)
  icp.setEuclideanFitnessEpsilon(icpParam_.euclideanFitnessEpsilon);

  // Set initial alignment estimate found using robot odometry
  Eigen::Matrix4f initGuess = Eigen::Matrix4f::Identity();

  // Matching
  icp.align(*output, initGuess);

  std::cout << "Has converged:" << icp.hasConverged() << " Score: " << icp.getFitnessScore() << std::endl;
  print4x4Matrix(icp.getFinalTransformation());
}


void ICPMapper::publishPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, const std::string& frameID)
{
  // Convert pcl::PointCloud<T> into sensor_msgs::PointCloud
  sensor_msgs::PointCloud2 output; 
  pcl::toROSMsg(*cloud, output);
  output.header.frame_id = frameID;

  // Publish the result
  modelPub_.publish(output);
}


void ICPMapper::filtering(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output, const Filter& filter)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud(input);
  pass.setFilterLimitsNegative(false);

  // Set range of x axis
  pass.setFilterFieldName("x");
  pass.setFilterLimits(filter.minAxis[0], filter.maxAxis[0]);
  pass.filter(*tmp);
  pass.setInputCloud(tmp);

  //Set range of y axis
  pass.setFilterFieldName("y");
  pass.setFilterLimits(filter.minAxis[1], filter.maxAxis[1]);
  pass.filter(*tmp);
  pass.setInputCloud(tmp);

  //Set range of z axis
  pass.setFilterFieldName("z");
  pass.setFilterLimits(filter.minAxis[2], filter.maxAxis[2]);
  pass.filter(*tmp);

  // Create the filtering object
  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
  sor.setInputCloud(tmp);
  sor.setMeanK(50);
  sor.setStddevMulThresh(filter_.stddevMulThresh);
  sor.filter(*output);
}


void ICPMapper::rotatePointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output, const float degree)
{
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

  pcl::PointCloud<pcl::PointXYZRGB> tmp;
  transform(2, 3) = -distance_;
  pcl::transformPointCloud(*input, tmp, transform);

  transform = Eigen::Matrix4f::Identity();
  float theta = pcl::deg2rad(degree);  // The y-angle of rotation in radians on kinect system coordination
  transform(0, 0) =  cos(theta);
  transform(0, 2) =  sin(theta);
  transform(2, 0) = -sin(theta);
  transform(2, 2) =  cos(theta);

  pcl::transformPointCloud(tmp, tmp, transform);

  transform = Eigen::Matrix4f::Identity();
  transform(2, 3) = distance_;
  pcl::transformPointCloud(tmp, *output, transform);
}


void ICPMapper::print4x4Matrix(const Eigen::Matrix4f& matrix)
{
  printf("Rotation matrix :\n");
  printf("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
  printf("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
  printf("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
  printf("Translation vector :\n");
  printf("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
}


bool ICPMapper::generateModel(ssh_icp_mapping::GenerateModel::Request& req, ssh_icp_mapping::GenerateModel::Response& res)
{
  std::vector<int> indices;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr removedNaNCloud (new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::removeNaNFromPointCloud(*originalCloud_, *removedNaNCloud, indices);
 
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  filtering(removedNaNCloud, filteredCloud, filter_);

  if (req.isFirstBoot)
  {
    // Store a previous model
    *previousModel_ = *currentModel_;

    std::cout << "--- Initial start-up for generating a model ---" << std::endl;
    *currentModel_ = *filteredCloud;
    *previousIcpResult_ = *currentModel_;
    publishPointCloud(currentModel_, "kinect");
    res.isSuccess = true;

    return true;
  }

  // Store a previous model
  *previousModel_ = *currentModel_;

  std::cout << "--- Generating a model using ICP ---" << std::endl;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr rotatedCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  rotatePointCloud(filteredCloud, rotatedCloud, req.degree);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGB>());
  matching(rotatedCloud, previousIcpResult_, output);
  
  if (output == nullptr)
  {
    ROS_ERROR("Matching by ICP failed");
    res.isSuccess = false;

    return false;
  }

  *previousIcpResult_ = *output; 
  *currentModel_ += *output;
  publishPointCloud(currentModel_, "kinect");
  res.isSuccess = true;

  return true;
}


bool ICPMapper::restorePreviousModel(ssh_icp_mapping::RestorePreviousModel::Request& req, ssh_icp_mapping::RestorePreviousModel::Response& res)
{
  if (currentModel_ == nullptr)
  {
    res.isSuccess = false;
    return false;
  }

  std::cout << "--- Restoring a previous model ---" << std::endl;

  *currentModel_ = *previousModel_;
  publishPointCloud(currentModel_, "kinect");
  res.isSuccess = true;

  return true;
}
