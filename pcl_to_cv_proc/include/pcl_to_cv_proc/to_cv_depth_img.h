//=================================================================================================
// Copyright (c) 2016, Stefan Kohlbrecher, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#ifndef TO_CV_DEPTH_IMG_H___
#define TO_CV_DEPTH_IMG_H___

#include <opencv/cv.h>
#include <opencv2/imgproc/imgproc.hpp>

#include <pcl/point_cloud.h>
#include <pcl/range_image/range_image_planar.h>

#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/Pose.h>

namespace pcl_to_cv_proc{

void toCv(const pcl::RangeImagePlanar& range_image, cv::Mat& cv_image) {

  cv_image = cv::Mat(range_image.height, range_image.width, CV_32FC1, cv::Scalar::all(NAN));

  for (size_t i = 0; i < range_image.size(); ++i){
    // Invalid values in range image have -inf
    if (range_image[i].range > 0.0){
      cv_image.at<float>(i) = range_image[i].range;
    }
  }

}

template <typename PointT>
bool generateDepthImage(const pcl::PointCloud<PointT>& cloud,
                        const Eigen::Affine3d& camera_pose,
                        pcl::RangeImagePlanar& range_image,
                        cv::Mat& out)
{

  // We use camera info here as it allows for potentially
  // easier use with camera type sensors in the future
  sensor_msgs::CameraInfo camera_info;
  camera_info.width = 600;
  camera_info.height = camera_info.width;
  camera_info.K[0] = camera_info.width;
  camera_info.K[4] = camera_info.K[0];
  camera_info.P[0] = camera_info.K[0];
  camera_info.P[5] = camera_info.K[4];

  camera_info.K[2] = camera_info.width / 2;
  camera_info.K[5] = camera_info.height / 2;
  camera_info.P[2] = camera_info.K[2];
  camera_info.P[6] = camera_info.K[5];

  range_image.createFromPointCloudWithFixedSize( cloud,
                                                 static_cast<int>(camera_info.width),
                                                 static_cast<int>(camera_info.height),
                                                 static_cast<float>(camera_info.width*0.5),
                                                 static_cast<float>(camera_info.height*0.5f),
                                                 static_cast<float>(camera_info.K[0]),
                                                 static_cast<float>(camera_info.K[4]),
                                                 camera_pose.cast<float>(),
                                                 pcl::RangeImage::LASER_FRAME);

  toCv(range_image, out);
  return true;
}

template <typename PointT>
bool generateDepthImage(const sensor_msgs::PointCloud2& cloud,
                        const Eigen::Affine3d& camera_pose,
                        pcl::RangeImagePlanar& range_image,
                        cv::Mat& out)
{
  pcl::PointCloud<PointT> cloud_pcl;
  pcl::fromROSMsg(cloud, cloud_pcl);

  return generateDepthImage<PointT>(cloud_pcl,
                             camera_pose,
                             range_image,
                             out);
}



}

#endif
