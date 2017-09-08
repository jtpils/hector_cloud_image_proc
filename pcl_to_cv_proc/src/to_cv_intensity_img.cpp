#include <pcl_to_cv_proc/to_cv_intensity_img.h>

namespace pcl_to_cv_proc {


bool generateIntensityImage(const pcl::PointCloud<pcl::PointXYZI>& cloud_in, const Eigen::Affine3d& camera_pose, cv::Mat& img_out, double sensor_noise, double min_distance) {
  // Generate virtual camera
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

  // Create model
  image_geometry::PinholeCameraModel pcm;
  pcm.fromCameraInfo(camera_info);

  std::vector<int> counter(camera_info.width * camera_info.height, 0);
  std::vector<double> ranges(camera_info.width * camera_info.height, 0);

  img_out = cv::Mat(camera_info.height, camera_info.width, CV_32FC1, cv::Scalar::all(0.0));

  // Iterate over every point in cloud and project to image plane
  for (unsigned int i = 0; i < cloud_in.size(); i++) {
    const pcl::PointXYZI& point = cloud_in[i];
    if (!pcl::isFinite(point)) {
      continue;
    }

    cv::Point3d cv_point(point.x, point.y, point.z);
    cv::Point2d uv = pcm.project3dToPixel(cv_point);

    if (uv.x < 0 || uv.y < 0 || uv.x >= camera_info.width || uv.y >= camera_info.height) {
      continue;
    }

    double range = pcl_norm(point);
    if (range < min_distance) {
      continue;
    }

    // TODO replace flooring with interpolation
    int pixel_x = std::floor(uv.x);
    int pixel_y = std::floor(uv.y);
    int pixel_idx = pixel_x * camera_info.width + pixel_y;

    double range_at_pixel = ranges[pixel_idx];

    bool replace_pixel = false;
    bool add_point = false;
    if (counter[pixel_idx] == 0 || range < range_at_pixel - sensor_noise) {
      replace_pixel = true;
    } else if (std::abs(range_at_pixel - range) < sensor_noise) {
      add_point = true;
    }

    if (replace_pixel) {
      counter[pixel_idx] = 1;
      img_out.at<float>(pixel_y, pixel_x) = point.intensity;
//      img_out.at<float>(pixel_y, pixel_x) = range;
      ranges[pixel_idx] = range;
    } else if (add_point) {
      counter[pixel_idx]++;
      img_out.at<float>(pixel_y, pixel_x) += (point.intensity - img_out.at<float>(pixel_y, pixel_x))/counter[pixel_idx];
//      img_out.at<float>(pixel_y, pixel_x) += (range - img_out.at<float>(pixel_y, pixel_x))/counter[pixel_idx];

    }
  }

//  cv::normalize(img_out, img_out, 0, 1, cv::NORM_MINMAX);
}

}
