#ifndef TO_CV_INTENSITY_IMG_H
#define TO_CV_INTENSITY_IMG_H

#include <pcl_conversions/pcl_conversions.h>

#include <opencv/cv.h>
#include <opencv2/imgproc/imgproc.hpp>

#include <sensor_msgs/CameraInfo.h>

#include <image_geometry/pinhole_camera_model.h>

namespace pcl_to_cv_proc {

template <typename PointT>

double pcl_norm(const PointT& point) {
  return std::sqrt(std::pow(point.x, 2) + std::pow(point.y, 2) + std::pow(point.z, 2));
}


bool generateIntensityImage(const pcl::PointCloud<pcl::PointXYZI>& cloud_in, const Eigen::Affine3d& camera_pose, cv::Mat& img_out, double sensor_noise = 0.0, double min_distance = 0.0);

}

#endif
