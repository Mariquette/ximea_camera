/******************************************************************************

Copyright 2015  Abdelhamid El-Bably (University of Waterloo)
                      [ahelbably@uwaterloo.ca]
                Arun Das (University of Waterloo)
                      [adas@uwaterloo.ca]
                Michael Tribou (University of Waterloo)
                      [mjtribou@uwaterloo.ca]
                Modified by: Pavel Fiala (University of West bohemia - 2017)       

All rights reserved.

********************************************************************************/

#ifndef XIMEA_CAMERA_XIMEA_ROS_DRIVER_H
#define XIMEA_CAMERA_XIMEA_ROS_DRIVER_H

#include <ximea_camera/ximea_driver.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/publisher.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Temperature.h>
#include <ximea_camera/XimeaCamSensorMsg.h>
#include <std_msgs/String.h>
#include <camera_info_manager/camera_info_manager.h>
#include <string>
#include <sstream>
#include <boost/shared_ptr.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

class ximea_ros_driver : public ximea_driver
{
public:
  ximea_ros_driver(const ros::NodeHandle &nh, std::string cam_name, int serial_no , std::string yaml_url);
  ximea_ros_driver(const ros::NodeHandle &nh, std::string file_name);
  virtual void setImageDataFormat(std::string s);
  void publishImage(const ros::Time & now);  // since these 2 functions should have the same time stamp we leave it up to the user to specify the timeif it is needed to do one or the other
  void publishCamInfo(const ros::Time &now);
  void publishCamTempChipInfo(const ros::Time &now);
  void publishCamSettings(const ros::Time &now);
  void publishCamCustomSettings(const ros::Time &now);
  void publishImageAndCamInfo();
protected:
  ros::NodeHandle pnh_;
  boost::shared_ptr<camera_info_manager::CameraInfoManager> cam_info_manager_;
  boost::shared_ptr<image_transport::ImageTransport> it_;
  image_transport::Publisher ros_cam_pub_;
  ros::Publisher cam_info_pub_;
  ros::Publisher cam_chip_temp_pub_;  
  ros::Publisher cam_settings_pub_;  
  ros::Publisher cam_custom_pub_; 
  
  sensor_msgs::Image ros_image_;
  sensor_msgs::CameraInfo cam_info_;
  sensor_msgs::Temperature temp_chip_info_;
  std_msgs::String cam_settings_string_;
  ximea_camera::XimeaCamSensorMsg cam_custom_info_;
  
  char * cam_buffer_;
  int cam_buffer_size_;
  int bpp_;  // the next 2 paramaeters are used by the ros_image_transport publisher
  std::string encoding_;

private:
  void common_initialize(const ros::NodeHandle &nh);
};

#endif  // XIMEA_CAMERA_XIMEA_ROS_DRIVER_H
