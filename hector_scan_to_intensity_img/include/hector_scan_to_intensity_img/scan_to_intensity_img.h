#ifndef SCAN_TO_INTENSITY_IMG_H
#define SCAN_TO_INTENSITY_IMG_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>

#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <pcl_to_cv_proc/to_cv_intensity_img.h>
#include <pcl_to_cv_proc/to_cv_depth_img.h>

namespace hector_scan_to_intensity_img {
  class ScanToIntensityImg {
  public:
    ScanToIntensityImg();
  private:
    void pointCloudCb(const sensor_msgs::PointCloud2ConstPtr& cloud_ptr);
    cv::Mat generateIntensityImage(const pcl::PointCloud<pcl::PointXYZI>& cloud);
    cv::Mat generateDepthImage(const pcl::PointCloud<pcl::PointXYZI>& cloud);

    ros::Subscriber cloud_sub_;
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Publisher image_pub_;
    image_transport::Publisher depth_image_pub_;

    // Parameters
    double min_distance_;
    double sensor_noise_;
    Eigen::Affine3d camera_pose_;
  };
}

#endif
