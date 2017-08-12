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
#ifndef XIMEA_CAMERA_XIMEA_DRIVER_H
#define XIMEA_CAMERA_XIMEA_DRIVER_H

// Sample for XIMEA Software Package V2.57
#include <m3api/xiApi.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/publisher.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/UInt8.h>
#include <camera_info_manager/camera_info_manager.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string>
#include <fstream>
#include <yaml-cpp/yaml.h>

class ximea_driver{
public:
  explicit ximea_driver(int serial_no = 0 , std::string cam_name = "");  // if no serial no is specified select the first cam on the bus
  explicit ximea_driver(std::string file_name);

  int readParamsFromFile(std::string file_name);
  void applyParameters();
  void errorHandling(XI_RETURN ret, std::string message);
  void enableTrigger(unsigned char trigger_mode);  // 0 none, 1 soft_trigger, 2 hard_trigger_rising edge (unsupported)
  void limitBandwidth(int mbps);
  void openDevice();
  void closeDevice();
  void startAcquisition();
  void stopAcquisition();
  void acquireImage();
  void triggerDevice();
  int getSerialNo() const{return serial_no_;}
  virtual void setImageDataFormat(std::string s);  // this is virtual because the ros class needs to do a bit more work to publish the right image
  void setROI(int rect_left, int rect_top, int rect_width, int rect_height);
  void setExposure(int time);
  void setGain(float db);
  void setAutoExposure(int auto_exposure);
  void setAutoExposureLimit(int ae_limit);
  void setAutoGainLimit(int ag_limit);
  void setAutoExposurePriority(float exp_priority);
  
  bool hasValidHandle(){ return xiH_ == NULL ? false : true;}
  const XI_IMG& getImage()const {return image_;}
  
  int getExposureSet() const { return exposure_set_; }    
  int getExposure() const { return exposure_time_; }
  int getExposure_1() const { return exposure_time_1_; }
  int getExposure_2() const { return exposure_time_2_; }
  int getExposure_3() const { return exposure_time_3_; }
  int getExposure_4() const { return exposure_time_4_; }
  
  float getGainSet() const { return gain_set_; }
  float getGain() const { return gain_; }
  float getGain_1() const { return gain_1_; }
  float getGain_2() const { return gain_2_; }
  float getGain_3() const { return gain_3_; }
  float getGain_4() const { return gain_4_; }
  float readSensorTemperature();
protected:
  void assignDefaultValues();

  // variables for ximea api internals
  std::string cam_name_;
  int serial_no_;
  std::string frame_id_;
  int cams_on_bus_;
  int bandwidth_safety_margin_;
  int frame_rate_;
  int bandwidth_;
  int exposure_set_;
  int exposure_time_;
  int exposure_time_1_;
  int exposure_time_2_;
  int exposure_time_3_;
  int exposure_time_4_;
  float gain_set_;
  float gain_;
  float gain_1_;
  float gain_2_;
  float gain_3_;
  float gain_4_;
  float chip_temperature_;
  int auto_exposure_;
  int auto_exposure_limit_;
  int auto_gain_limit_;
  float auto_exposure_priority_;
  bool binning_enabled_;
  int downsample_factor_;
  int rect_left_;
  int rect_top_;
  int rect_width_;
  int rect_height_;
  bool acquisition_active_;
  std::string image_data_format_;  // One of XI_MONO8, XI_RGB24, XI_RGB32, XI_RAW
  std::string yaml_url_;
  HANDLE xiH_;
  XI_IMG image_;
  int image_capture_timeout_;      // max amount of time to wait for an image to come in
  unsigned char trigger_mode_;
};

#endif  // XIMEA_CAMERA_XIMEA_DRIVER_H
