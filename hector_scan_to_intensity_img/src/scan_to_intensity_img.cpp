#include <hector_scan_to_intensity_img/scan_to_intensity_img.h>

namespace hector_scan_to_intensity_img {

ScanToIntensityImg::ScanToIntensityImg()
  : nh_(""), it_(nh_) {
  // Load params
  ros::NodeHandle pnh("~");
  pnh.param("min_distance", min_distance_, 0.0);
  ROS_INFO_STREAM("min_distance: " << min_distance_);
  pnh.param("sensor_noise", sensor_noise_, 0.0);
  ROS_INFO_STREAM("sensor_noise: " << sensor_noise_);
  std::vector<double> camera_pose_rpy;
  if (pnh.getParam("camera_pose_rpy", camera_pose_rpy)) {
    if (camera_pose_rpy.size() == 3) {
      //TODO
    } else {
      ROS_ERROR_STREAM("Parameter 'camera_pose_rpy' must have size 3. Using identity.");
      camera_pose_ = Eigen::Affine3d::Identity();
    }

  } else {
    camera_pose_ = Eigen::Affine3d::Identity();
  }

  image_pub_ = it_.advertise("intensity_image", 1, true);
  depth_image_pub_ = it_.advertise("depth_image", 1, true);
  cloud_sub_ = nh_.subscribe("cloud", 1, &ScanToIntensityImg::pointCloudCb, this);
}

void ScanToIntensityImg::pointCloudCb(const sensor_msgs::PointCloud2ConstPtr &cloud_ptr) {
  pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
  pcl::fromROSMsg(*cloud_ptr, pcl_cloud);
  ROS_INFO_STREAM("Received pc with size: " << pcl_cloud.size());


  cv_bridge::CvImage cv_image;

  cv_image.header.stamp = cloud_ptr->header.stamp;
  cv_image.header.frame_id = cloud_ptr->header.frame_id;

  cv_image.image = generateIntensityImage(pcl_cloud);
  cv_image.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
//  cv_image.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  image_pub_.publish(cv_image.toImageMsg());

  cv_image.image = generateDepthImage(pcl_cloud);
  cv_image.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  depth_image_pub_.publish(cv_image.toImageMsg());
}

cv::Mat ScanToIntensityImg::generateIntensityImage(const pcl::PointCloud<pcl::PointXYZI>& cloud) {
  cv::Mat intensity_img;
  pcl_to_cv_proc::generateIntensityImage(cloud, Eigen::Affine3d::Identity(), intensity_img, sensor_noise_, min_distance_);

  ROS_INFO_STREAM("Publishing intensity image with shape: " << intensity_img.cols << "x" << intensity_img.rows);
  double min, max;
  cv::minMaxLoc(intensity_img, &min, &max);
  ROS_INFO_STREAM("Min: " << min << ", Max: " << max);
  return intensity_img;
}

cv::Mat ScanToIntensityImg::generateDepthImage(const pcl::PointCloud<pcl::PointXYZI>& cloud) {
  cv::Mat depth_image;
  pcl::RangeImagePlanar pcl_range_image;
  pcl_to_cv_proc::generateDepthImage(cloud, Eigen::Affine3d::Identity(), pcl_range_image, depth_image);

  ROS_INFO_STREAM("Publishing depth image with shape: " << depth_image.cols << "x" << depth_image.rows);
  double min, max;
  cv::minMaxLoc(depth_image, &min, &max);
  ROS_INFO_STREAM("Min: " << min << ", Max: " << max);
  return depth_image;
}

}
