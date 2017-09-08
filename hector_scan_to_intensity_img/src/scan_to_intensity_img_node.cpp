#include <ros/ros.h>
#include <hector_scan_to_intensity_img/scan_to_intensity_img.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "scan_to_intensity_image");

  ROS_INFO_STREAM("Started scan to intensity image node");

  hector_scan_to_intensity_img::ScanToIntensityImg sti;

  ros::spin();
  return 0;
}
