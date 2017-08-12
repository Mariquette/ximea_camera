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
#include <ximea_camera/ximea_ros_driver.h>
#include <string>
#include <algorithm>
#include <boost/make_shared.hpp>

// #define DEBUG

// toString helper function / template ...
// ---------------------------------------
template <typename T> std::string toString(const T& t){
      
    std::ostringstream os;
    os << t;
    return os.str();
}

// ximea_ros_driver constructor -> inherits from ximea_driver / 1 ...
// ------------------------------------------------------------------
ximea_ros_driver::ximea_ros_driver(const ros::NodeHandle &nh, std::string cam_name, int serial_no, std::string yaml_url): ximea_driver(serial_no, cam_name){
  
  pnh_ = nh;
  cam_info_manager_ = boost::make_shared<camera_info_manager::CameraInfoManager>(pnh_, cam_name_);
  // cam_info_manager_->loadCameraInfo(yaml_url);
  
  /*
  We create an ImageTransport instance, initializing it with our NodeHandle. We use methods of ImageTransport to create image publishers and subscribers, much as we use methods of NodeHandle to create generic ROS publishers and subscribers. 
  */
  it_ = boost::make_shared<image_transport::ImageTransport>(nh);
  
  /*

  Advertise that we are going to be publishing images on the base topic "camera/image". Depending on whether more plugins are built, additional (per-plugin) topics derived from the base topic may also be advertised. The second argument is the size of our publishing queue. 
  
  advertise() returns an image_transport::Publisher object, which serves two purposes: 1) it contains a publish() method that lets you publish images onto the base topic it was created with, and 2) when it goes out of scope, it will automatically unadvertise. 
  */
  
  ros_cam_pub_ = it_->advertise(std::string("image_raw"), 1);
  cam_info_pub_ = pnh_.advertise<sensor_msgs::CameraInfo>(std::string("camera_info"), 1);
  cam_chip_temp_pub_ = pnh_.advertise<sensor_msgs::Temperature>(std::string("chip_temp_info"), 1);
  cam_settings_pub_ = pnh_.advertise<std_msgs::String>(std::string("camera_settings"), 1);

  // Custom message /production ...
  // -----------------------------
  cam_custom_pub_= pnh_.advertise<ximea_camera::XimeaCamSensorMsg>(std::string("ximea_info_settings"), 1);		

}

// ximea_ros_driver constructor -> inherits from ximea_driver / 2 ...
// ------------------------------------------------------------------
ximea_ros_driver::ximea_ros_driver(const ros::NodeHandle &nh, std::string file_name) : ximea_driver(file_name){
  pnh_ = nh;
  cam_info_manager_ = boost::make_shared<camera_info_manager::CameraInfoManager>(pnh_, cam_name_);
  // cam_info_manager_->loadCameraInfo(yaml_url_);
  
  /*
  We create an ImageTransport instance, initializing it with our NodeHandle. We use methods of ImageTransport to create image publishers and subscribers, much as we use methods of NodeHandle to create generic ROS publishers and subscribers. 
  */
  
  it_ = boost::make_shared<image_transport::ImageTransport>(nh);
  
  /*

  Advertise that we are going to be publishing images on the base topic "camera/image". Depending on whether more plugins are built, additional (per-plugin) topics derived from the base topic may also be advertised. The second argument is the size of our publishing queue. 
  
  advertise() returns an image_transport::Publisher object, which serves two purposes: 1) it contains a publish() method that lets you publish images onto the base topic it was created with, and 2) when it goes out of scope, it will automatically unadvertise. 
  */
  
  ros_cam_pub_ = it_->advertise(std::string("image_raw"), 1);
  cam_info_pub_ = pnh_.advertise<sensor_msgs::CameraInfo>(std::string("camera_info"), 1);
  cam_chip_temp_pub_ = pnh_.advertise<sensor_msgs::Temperature>(std::string("chip_temp_info"), 1);
  cam_settings_pub_ = pnh_.advertise<std_msgs::String>(std::string("camera_settings"), 1);
  // Custom message /production ...
  // -----------------------------
  cam_custom_pub_= pnh_.advertise<ximea_camera::XimeaCamSensorMsg>(std::string("ximea_info_settings"), 1);
}

// common_initialize - private function ...
// ---------------------------------------- 
void ximea_ros_driver::common_initialize(const ros::NodeHandle &nh){
  pnh_ = nh;
  cam_info_manager_ = boost::make_shared<camera_info_manager::CameraInfoManager>(pnh_, cam_name_);
  // cam_info_manager_->loadCameraInfo("");   // TODO: yaml_url
  
  /*
  We create an ImageTransport instance, initializing it with our NodeHandle. We use methods of ImageTransport to create image publishers and subscribers, much as we use methods of NodeHandle to create generic ROS publishers and subscribers. 
  */
  
  it_ = boost::make_shared<image_transport::ImageTransport>(nh);
  
  /*

  Advertise that we are going to be publishing images on the base topic "camera/image". Depending on whether more plugins are built, additional (per-plugin) topics derived from the base topic may also be advertised. The second argument is the size of our publishing queue. 
  
  advertise() returns an image_transport::Publisher object, which serves two purposes: 1) it contains a publish() method that lets you publish images onto the base topic it was created with, and 2) when it goes out of scope, it will automatically unadvertise. 
  */
  
  ros_cam_pub_ = it_->advertise(cam_name_ + std::string("/image_raw"), 1);
  cam_info_pub_ = pnh_.advertise<sensor_msgs::CameraInfo>(cam_name_ + std::string("/camera_info"), 1);
  cam_chip_temp_pub_ = pnh_.advertise<sensor_msgs::Temperature>(std::string("/chip_temp_info"), 1);
  cam_settings_pub_ = pnh_.advertise<std_msgs::String>(std::string("/camera_settings"), 1);
  
  // Custom message /production ...
  // -----------------------------
  cam_custom_pub_= pnh_.advertise<ximea_camera::XimeaCamSensorMsg>(std::string("/ximea_info_settings"), 1);
  
}

// publishImage - public function ...
// ----------------------------------
void ximea_ros_driver::publishImage(const ros::Time & now){
    
  cam_buffer_ = reinterpret_cast<char *>(image_.bp);
  cam_buffer_size_ = image_.width * image_.height * bpp_;
  ros_image_.data.resize(cam_buffer_size_);
  ros_image_.encoding = encoding_;
  ros_image_.width = image_.width;
  ros_image_.height = image_.height;
  ros_image_.step = image_.width * bpp_;

  copy(reinterpret_cast<char *>(cam_buffer_),
       (reinterpret_cast<char *>(cam_buffer_)) + cam_buffer_size_,
       ros_image_.data.begin());

  ros_cam_pub_.publish(ros_image_);
}

// publishCamInfo - public function ...
// ------------------------------------
void ximea_ros_driver::publishCamInfo(const ros::Time &now){
  ros_image_.header.stamp = now;
  cam_info_ = cam_info_manager_->getCameraInfo();
  cam_info_.header.frame_id = frame_id_;
  cam_info_pub_.publish(cam_info_);
}

// publishCamTempChipInfo - public function ...
// --------------------------------------------
void ximea_ros_driver::publishCamTempChipInfo(const ros::Time &now){
   temp_chip_info_.header.stamp = now; 
   temp_chip_info_.temperature = readSensorTemperature();
   temp_chip_info_.variance = 0.0;
   cam_chip_temp_pub_.publish(temp_chip_info_);
}

// publishCamSettings - public function ...
// ----------------------------------------
void ximea_ros_driver::publishCamSettings(const ros::Time &now){
    
   boost::posix_time::ptime my_posix_time = now.toBoost(); 
   std::string iso_time_string = boost::posix_time::to_iso_extended_string(my_posix_time); 
    
   cam_settings_string_.data = "Time:"+ iso_time_string + "::Exposuse:"+ toString(getExposureSet())+ "::Gain:" + toString(getGainSet())+"::TempChip:"+toString(readSensorTemperature());
   cam_settings_pub_.publish(cam_settings_string_);
}

// publishCamCustomSettings - public function ...
// ----------------------------------------------
void ximea_ros_driver::publishCamCustomSettings(const ros::Time &now){
    
    cam_custom_info_.header.stamp = now;
    cam_custom_info_.exposure_time = (float)getExposureSet();
    cam_custom_info_.gain = (uint8_t)getGainSet();
    cam_custom_info_.temperature = (float)readSensorTemperature();
    
    cam_custom_pub_.publish(cam_custom_info_);
}

// publishImageAndCamInfo - public function ...
// --------------------------------------------
void ximea_ros_driver::publishImageAndCamInfo(){
  ros::Time now = ros::Time::now();
  publishImage(now);
#ifdef DEBUG_  
  publishCamInfo(now);
  publishCamTempChipInfo(now);
  publishCamSettings(now);
#endif
  publishCamCustomSettings(now);
}

// setImageDataFormat - public function / virtual ...
// --------------------------------------------------
void ximea_ros_driver::setImageDataFormat(std::string image_format){
  XI_RETURN stat;
  int image_data_format;

  if (!hasValidHandle()){
    return;
  }
  
  if (image_format == std::string("XI_MONO16")){
    image_data_format = XI_MONO16;
    encoding_ = std::string("mono16");
    bpp_ = 2;
  } else if (image_format == std::string("XI_RGB24")){
    image_data_format = XI_RGB24;
    encoding_ = std::string("bgr8");
    bpp_ = 3;
  } else if (image_format == std::string("XI_RGB32")){
    image_data_format = XI_RGB32;
    encoding_ = std::string("bgra8");
    bpp_ = 4;
  } else if (image_format == std::string("XI_RGB_PLANAR")){
    image_data_format = XI_MONO8;
    std::cout << "This is unsupported in ROS default to XI_MONO8" << std::endl;
    bpp_ = 1;
  } else if (image_format == std::string("XI_RAW8")){
    image_data_format = XI_RAW8;
    encoding_ = std::string("mono8");
    bpp_ = 1;
  } else if (image_format == std::string("XI_RAW16")){
    image_data_format = XI_RAW16;
    encoding_ = std::string("mono16");
    bpp_ = 2;
  } else {
    image_data_format = XI_MONO8;
    encoding_ = std::string("mono8");
    bpp_ = 1;
  }

  stat = xiSetParamInt(xiH_, XI_PRM_IMAGE_DATA_FORMAT, image_data_format);
  errorHandling(stat, "image_format");    // if we cannot set the format then there is something wrong we should probably quit then
  image_data_format_ = image_data_format;
}
