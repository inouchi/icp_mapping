#include "ICPMapper.hpp"

ICPMapper::ICPMapper(ros::NodeHandle* nodeHandlePtr, ros::NodeHandle* localNodeHandlePtr)
  : nodeHandlePtr_(nodeHandlePtr),
    localNodeHandlePtr_(localNodeHandlePtr),
    originalCloud_(new pcl::PointCloud<pcl::PointXYZRGB>),
    model_(new pcl::PointCloud<pcl::PointXYZRGB>)
{
  // Initialize Sibscriber and Publishers
  cloudSub_ = nodeHandlePtr_->subscribe("/camera/depth_registered/points", 1, &ICPMapper::cloudCb, this);
  modelPub_ = nodeHandlePtr->advertise<sensor_msgs::PointCloud2>(ros::this_node::getName() + "/mode", 1);

  generateModelSrv_ = localNodeHandlePtr_->advertiseService("generate_model", &ICPMapper::generateModel, this);
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

  // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
  icp.setMaxCorrespondenceDistance(0.05);
  // Set the maximum number of iterations (criterion 1)
  icp.setMaximumIterations(100);
  // Set the transformation epsilon (criterion 2)
  icp.setTransformationEpsilon(1e-8);
  // Set the euclidean distance difference epsilon (criterion 3)
  icp.setEuclideanFitnessEpsilon(1);

  icp.align(*output);
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


bool ICPMapper::generateModel(ssh_icp_mapping::GenerateModelRequest& req, ssh_icp_mapping::GenerateModelResponse& res)
{
  std::mutex mutex;
  std::lock_guard<std::mutex> lock(mutex);

  if (req.isFirstBoot)
  {
    std::cout << "--- Initial start-up for generating a model ---" << std::endl;

    *model_ = *originalCloud_;
    publishPointCloud(model_, "kinect");
    res.isSuccess = true;

    return true;
  }

  std::cout << "--- Generating a model using ICP ---" << std::endl;

  std::vector<int> indices;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr input(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr target(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::removeNaNFromPointCloud(*originalCloud_, *input, indices);
  pcl::removeNaNFromPointCloud(*model_, *target, indices);
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGB>());
  matching(input, target, output);

  if (output == nullptr)
  {
    ROS_ERROR("Matching by ICP failed");
    res.isSuccess = false;

    return false;
  }

  model_ = output;
  publishPointCloud(model_, "kinect");
  res.isSuccess = true;

  return true;
}