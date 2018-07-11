#pragma once

// C++ specific includes
#include <iostream>
#include <vector>
#include <string>
#include <mutex>

// ROS specific includes
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "ssh_icp_mapping/GenerateModel.h"

// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h> 
#include <pcl/filters/extract_indices.h>
#include <pcl/registration/icp.h>
#include <pcl/common/angles.h>
#include <pcl/filters/passthrough.h>

struct Filter
{
  Filter()
  {
    minAxis.resize(3);
    maxAxis.resize(3);
  }

  std::vector<float> minAxis;
  std::vector<float> maxAxis;
};

class ICPMapper
{

  public:
    ICPMapper(ros::NodeHandle* nodeHandlePtr, ros::NodeHandle* localNodeHandlePtr);
    ~ICPMapper();


  protected:
    void cloudCb(const sensor_msgs::PointCloud2ConstPtr& input);
    void matching(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr target,
                  pcl::PointCloud<pcl::PointXYZRGB>::Ptr output);
    void publishPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, const std::string& frameID);
    void filtering(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output, const Filter& filter);
    void rotatePointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output, const float degree, const float distance);
    void print4x4Matrix(const Eigen::Matrix4f& matrix);
    bool generateModel(ssh_icp_mapping::GenerateModelRequest& req, ssh_icp_mapping::GenerateModelResponse& res);


  private:  
    ros::NodeHandle* nodeHandlePtr_;
    ros::NodeHandle* localNodeHandlePtr_;

    ros::Subscriber cloudSub_;  // For subscribing a cloud data from Kinect
    ros::Publisher modelPub_;   // For publishing a model generated by ICP
    ros::ServiceServer generateModelSrv_;  // It is called, and generate a model using ICP and publish the model

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr originalCloud_;  // Keep up-to-date a cloud data from Kinect
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr model_;          // GeFor determing scope of point cloud used in ICPunerated model by ICP

    Filter filter_;  // For determing scope of point cloud used in ICP 

};
