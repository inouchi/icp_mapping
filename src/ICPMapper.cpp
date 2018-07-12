#include "ICPMapper.hpp"

ICPMapper::ICPMapper(ros::NodeHandle* nodeHandlePtr, ros::NodeHandle* localNodeHandlePtr)
  : nodeHandlePtr_(nodeHandlePtr),
    localNodeHandlePtr_(localNodeHandlePtr),
    originalCloud_(new pcl::PointCloud<pcl::PointXYZRGB>),
    model_(new pcl::PointCloud<pcl::PointXYZRGB>)
{
  // Initialize Sibscriber and Publishers
  cloudSub_ = nodeHandlePtr_->subscribe("/kinect2/sd/points", 1, &ICPMapper::cloudCb, this);
  modelPub_ = nodeHandlePtr->advertise<sensor_msgs::PointCloud2>(ros::this_node::getName() + "/model", 1);

  generateModelSrv_ = localNodeHandlePtr_->advertiseService("generate_model", &ICPMapper::generateModel, this);
 
  if (!(localNodeHandlePtr_->getParam("filter_min_axis", filter_.minAxis)))
  {
    filter_.minAxis[0] = -0.2; 
    filter_.minAxis[1] = -0.3;
    filter_.minAxis[2] =  0.8;
  }
  
  if (!(nodeHandlePtr->getParam("filter_max_axis", filter_.maxAxis)))
  {
    filter_.maxAxis[0] = 0.2;
    filter_.maxAxis[1] = 0.17;
    filter_.maxAxis[2] = 1.1;
  }
}
 


ICPMapper::~ICPMapper()
{

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

  pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
  icp.setInputCloud(input);
  icp.setInputTarget(target);
 
  // Set the max correspondence distance to 1cm (e.g., correspondences with higher distances will be ignored)
  //icp.setMaxCorrespondenceDistance(1.0);
  // Set the maximum number of iterations (criterion 1)
  icp.setMaximumIterations(800);
  // Set the transformation epsilon (criterion 2)
  //icp.setTransformationEpsilon(1e-9);
  // Set the euclidean distance difference epsilon (criterion 3)
  //icp.setEuclideanFitnessEpsilon(1);

  Eigen::Matrix4f initGuess = Eigen::Matrix4f::Identity();
  icp.align(*output, initGuess);

  *output += *target;
  std::cout << "Has converged:" << icp.hasConverged() << " Score: " << icp.getFitnessScore() << std::endl;
  std::cout << icp.getFinalTransformation() << std::endl;
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
  sor.setStddevMulThresh(1.0);
  sor.filter(*output);
}


void ICPMapper::rotatePointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output, const float degree, const float distance)
{
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

  pcl::PointCloud<pcl::PointXYZRGB> tmp;
  transform(2, 3) = -distance;
  pcl::transformPointCloud(*input, tmp, transform);

  transform = Eigen::Matrix4f::Identity();
  float theta = pcl::deg2rad(degree);  // The y-angle of rotation in radians on kinect system coordination
  transform(0, 0) =  cos(theta);
  transform(0, 2) =  sin(theta);
  transform(2, 0) = -sin(theta);
  transform(2, 2) =  cos(theta);

  pcl::transformPointCloud(tmp, tmp, transform);

  transform = Eigen::Matrix4f::Identity();
  transform(2, 3) = distance;
  pcl::transformPointCloud(tmp, *output, transform);
}


void ICPMapper::print4x4Matrix(const Eigen::Matrix4f& matrix)
{
  printf ("Rotation matrix :\n");
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
  printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
  printf ("Translation vector :\n");
  printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
}


bool ICPMapper::generateModel(ssh_icp_mapping::GenerateModelRequest& req, ssh_icp_mapping::GenerateModelResponse& res)
{
  std::vector<int> indices;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr removedNaNCloud (new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::removeNaNFromPointCloud(*originalCloud_, *removedNaNCloud, indices);
 
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  filtering(removedNaNCloud, filteredCloud, filter_);

  if (req.isFirstBoot)
  {
    std::cout << "--- Initial start-up for generating a model ---" << std::endl;
    *model_ = *filteredCloud;
    publishPointCloud(model_, "kinect");
    res.isSuccess = true;

    return true;
  }

  std::cout << "--- Generating a model using ICP ---" << std::endl;
 
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr rotatedCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  rotatePointCloud(filteredCloud, rotatedCloud, req.degree, req.distance);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGB>());
  matching(rotatedCloud, model_, output);
  
  if (output == nullptr)
  {
    ROS_ERROR("Matching by ICP failed");
    res.isSuccess = false;

    return false;
  }

  *model_ += *output;
  publishPointCloud(model_, "kinect");
  res.isSuccess = true;

  return true;
}
