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
  std::vector<double> ranges(camera_info.width * camera_info.height, std::numeric_limits<double>::infinity());

  img_out = cv::Mat(camera_info.height, camera_info.width, CV_32FC1, cv::Scalar::all(-1));

  // Iterate over every point in cloud and project to image plane
  for (unsigned int i = 0; i < cloud_in.size(); i++) {
    const pcl::PointXYZI& point = cloud_in[i];
    if (point.z < 0) { // check if point is behind camera
      continue;
    }
    if (!pcl::isFinite(point)) { // check for NaN/Inf
      continue;
    }

    cv::Point3d cv_point(point.x, point.y, point.z);
    cv::Point2d uv = pcm.project3dToPixel(cv_point);

    int pixel_x = std::round(uv.x);
    int pixel_y = std::round(uv.y);

    // check if projected point is in image
    if (!isInImage(camera_info.width, camera_info.height, pixel_x, pixel_y)) {
      continue;
    }

    double range = pcl_norm(point);
    if (range < min_distance) { // discard points, that are too close
      continue;
    }

    // little bit of interpolation
    int floor_x = std::floor(uv.x), floor_y = std::floor(uv.y),
        ceil_x  = std::ceil(uv.x),  ceil_y  = std::ceil(uv.y);

    int neighbor_x[4], neighbor_y[4];
    neighbor_x[0]=floor_x; neighbor_y[0]=floor_y;
    neighbor_x[1]=floor_x; neighbor_y[1]=ceil_y;
    neighbor_x[2]=ceil_x;  neighbor_y[2]=floor_y;
    neighbor_x[3]=ceil_x;  neighbor_y[3]=ceil_y;

    for (int j = 0; j < 4; ++j) {
      int n_x = neighbor_x[j], n_y = neighbor_y[j];
      if (n_x == pixel_x && n_y == pixel_y)
        continue;
      if (isInImage(camera_info.width, camera_info.height, n_x, n_y)) {
        int neighbor_idx = n_y * camera_info.width + n_x;
        if (counter[neighbor_idx] == 0) {
          if (std::isinf(ranges[neighbor_idx])) {
            img_out.at<float>(n_y, n_x) = point.intensity;
          } else if (range < ranges[neighbor_idx]) {
            img_out.at<float>(n_y, n_x) = point.intensity;
          }
        }
     }
    }

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

  cv::Mat inpainting_mask(camera_info.height, camera_info.width, CV_8UC1, cv::Scalar::all(0));
  inpainting_mask.setTo(1, img_out < 0);
  double min, max;
  cv::minMaxLoc(img_out, &min, &max);
  img_out.convertTo(img_out, CV_8UC1, 255.0/max);
  cv::inpaint(img_out, inpainting_mask, img_out, 10, cv::INPAINT_NS);
//  cv::imshow("image", img_out);
//  cv::waitKey(1);
}

}
