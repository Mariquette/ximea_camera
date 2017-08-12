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

#include <ximea_camera/ximea_driver.h>
#include <stdexcept>
#include <sstream>
#include <string>

// ximea_driver constructor / 1
// ----------------------------
ximea_driver::ximea_driver(int serial_no, std::string cam_name){
  serial_no_ = serial_no;
  cam_name_ = cam_name;
  assignDefaultValues();
}

// assignDefaultValues - protected function
// ----------------------------------------
void ximea_driver::assignDefaultValues(){
  
  // Basic settings
  // --------------  
  cams_on_bus_ = 1;
  bandwidth_safety_margin_ = 30;
  binning_enabled_ = false;
  downsample_factor_ = false;

  // Manual settings
  // -------------- 
  exposure_set_ = 10000;    // Default is 10 ms 
  exposure_time_ = 10000;   // Default is 10 ms 
  exposure_time_1_ = 10000; // Default is 10 ms
  exposure_time_2_ = 10000; // Default is 10 ms
  exposure_time_3_ = 10000; // Default is 10 ms
  exposure_time_4_ = 10000; // Default is 10 ms
  
  gain_set_=0; // Default is 0  
  gain_ = 0;   // Default is 0           
  gain_1_ = 0; // Default is 0  
  gain_2_ = 0; // Default is 0  
  gain_3_ = 0; // Default is 0 
  gain_4_ = 0; // Default is 0 
  
  // Automatic settings
  // ------------------
  auto_exposure_ = 0;
  auto_exposure_limit_ = 500000;
  auto_gain_limit_ = 2;
  auto_exposure_priority_ = 0.8;
  
  // Image format
  // ------------
  image_data_format_ = "XI_RGB24";   // "XI_MONO8";
  
  // ROI - not used 
  // --------------
  rect_left_ = 0;
  rect_top_ = 0;
  rect_width_ = 1280;
  rect_height_ = 1024;
  
  // XIMEA image
  // -----------
  xiH_ = NULL;
  image_.size = sizeof(XI_IMG);
  image_.bp = NULL;
  image_.bp_size = 0;
  
  // Acquisition params
  // ------------------
  acquisition_active_ = false;
  image_capture_timeout_ = 1000;
  frame_id_ = "";
}

// ximea_driver constructor / 2
// ----------------------------
ximea_driver::ximea_driver(std::string file_name){
  assignDefaultValues();
  readParamsFromFile(file_name);
  ROS_INFO_STREAM("ximea_driver: reading parameter values from file: " << file_name);
}

// errorHandling - public function ...
// -----------------------------------
void ximea_driver::errorHandling(XI_RETURN ret, std::string message){
  if (ret != XI_OK){
     std::stringstream errMsg;
     std::cout << "Error after "  << message << std::endl;
     closeDevice();
  }
}

// applyParameters - public function ...
// -------------------------------------
void ximea_driver::applyParameters(){
    
  setImageDataFormat(image_data_format_);
  
  if (auto_exposure_ == 0) {
      // Manual settings ...
      // -------------------
      setExposure(exposure_time_);
      setGain(gain_);
  } else {
      // Automatic settings ...
      // ----------------------
      setAutoExposure(auto_exposure_);
      setAutoExposureLimit(auto_exposure_limit_);
      setAutoGainLimit(auto_gain_limit_);
      setAutoExposurePriority(auto_exposure_priority_);
  }
  
  // -- Setting of ROI / Region Of Interrest is disabled for our purpose --
  // ----------------------------------------------------------------------
  // setROI(rect_left_, rect_top_, rect_width_, rect_height_);
}

// openDevice - public function ...
// --------------------------------
void ximea_driver::openDevice(){
  XI_RETURN stat;
  
  if (serial_no_ == 0){
    stat = xiOpenDevice(0, &xiH_); // by ID ...
    errorHandling(stat, "Open Device");
  }else{
    std::stringstream conv;
    conv << serial_no_;            // by SN ... 
    stat = xiOpenDeviceBy(XI_OPEN_BY_SN, conv.str().c_str(), &xiH_);
    errorHandling(stat, "Open Device");
  }
  
  // Apply parameters ...
  // --------------------
  applyParameters();
}

// closeDevice - public function ...
// ---------------------------------
void ximea_driver::closeDevice(){
  
    if (xiH_){
        xiCloseDevice(xiH_);
        xiH_ = NULL;
    }
}

// startAcquisition - public function ...
// --------------------------------------
void ximea_driver::startAcquisition(){
  
  if (!hasValidHandle()){
    return;
  }
  
  XI_RETURN stat = xiStartAcquisition(xiH_);
  errorHandling(stat, "Starting Acquisition");
  acquisition_active_ = true;
}

// stopAcquisition - public function ... 
// -------------------------------------
void ximea_driver::stopAcquisition(){
  XI_RETURN stat;
  
  if (!hasValidHandle()){
    return;
  }
  
  stat = xiStopAcquisition(xiH_);
  errorHandling(stat, "Starting Acquisition");      // die if we cannot stop acquisition ...
  acquisition_active_ = false;
}

// acquireImage - public function ...
// ----------------------------------
void ximea_driver::acquireImage(){
  
  if (!hasValidHandle()){
    return;
  }
  
  XI_RETURN stat = xiGetImage(xiH_, image_capture_timeout_, &image_);
  
  if (stat != 0){
     std::cout << "Error on" << cam_name_ << " with error " <<  stat << std::endl;
  }
}

// setImageDataFormat - public function ...
// ----------------------------------------
void ximea_driver::setImageDataFormat(std::string image_format){
  
  XI_RETURN stat;
  int image_data_format;

  if (!hasValidHandle()){
    return;
  }
  
  if (image_format == std::string("XI_MONO16")){
    image_data_format = XI_MONO16;
  } else if (image_format == std::string("XI_RGB24")){
    image_data_format = XI_RGB24;
  } else if (image_format == std::string("XI_RGB32")){
    image_data_format = XI_RGB32;
  } else if (image_format == std::string("XI_RGB_PLANAR")){
    image_data_format = XI_MONO8;
    std::cout << "This is unsupported in ROS default to XI_MONO8" << std::endl;
  } else if (image_format == std::string("XI_RAW8")){
    image_data_format = XI_RAW8;
  } else if (image_format == std::string("XI_RAW16")){
    image_data_format = XI_RAW16;
  } else {
    image_data_format = XI_MONO8;
  }

  stat = xiSetParamInt(xiH_, XI_PRM_IMAGE_DATA_FORMAT, image_data_format);
  // if we cannot set the format then there is something wrong we should probably quit then ...
  // ------------------------------------------------------------------------------------------
  errorHandling(stat, "image_format"); 
  image_data_format_ = image_data_format;
}

// setROI - public function - not supported - disabled ...
// -------------------------------------------------------
void ximea_driver::setROI(int l, int t, int w, int h){
  
  XI_RETURN stat;

  if (!hasValidHandle()){
    return;
  }

  if (l < 0 || l > 1280) rect_left_ = 0;
  else rect_left_ = l;
  if (t < 0 || t > 1024) rect_top_ = 0;
  else rect_top_ = t;
  if (w < 0 || w > 1280) rect_width_ = 1280;
  else rect_width_ = w;
  if (h < 0 || h > 1024) rect_height_ = 1024;
  else rect_height_ = h;
  if (l + w > 1280){
    rect_left_ =  0;
    rect_width_ = 1280;
  }
  
  if (h + t > 1024){
    rect_top_ =  0;
    rect_height_ = 1024;
  }

  std::cout << rect_height_ << " " << rect_width_ << " " << rect_left_ << " " << rect_top_ << std::endl;

  int tmp;
  stat = xiSetParamInt(xiH_, XI_PRM_WIDTH, rect_width_);
  errorHandling(stat, "xiSetParamInt (aoi width)");
  xiGetParamInt(xiH_, XI_PRM_WIDTH XI_PRM_INFO_INCREMENT, &tmp);
  std::cout << "width increment " << tmp << std::endl;

  stat = xiSetParamInt(xiH_, XI_PRM_HEIGHT, rect_height_);
  errorHandling(stat, "xiSetParamInt (aoi height)");
  xiGetParamInt(xiH_, XI_PRM_HEIGHT XI_PRM_INFO_INCREMENT, &tmp);
  std::cout << "height increment " << tmp << std::endl;

  stat = xiSetParamInt(xiH_, XI_PRM_OFFSET_X, rect_left_);
  errorHandling(stat, "xiSetParamInt (aoi left)");
  xiGetParamInt(xiH_, XI_PRM_OFFSET_X XI_PRM_INFO_INCREMENT, &tmp);
  std::cout << "left increment " << tmp << std::endl;

  stat = xiSetParamInt(xiH_, XI_PRM_OFFSET_Y, rect_top_);
  errorHandling(stat, "xiSetParamInt (aoi top)");
  xiGetParamInt(xiH_, XI_PRM_OFFSET_Y XI_PRM_INFO_INCREMENT, &tmp);
  std::cout << "top increment " << tmp << std::endl;
}

// setExposure - public function / sets exposure time in microseconds !!! ...
// --------------------------------------------------------------------------
void ximea_driver::setExposure(int time){
  XI_RETURN stat = xiSetParamInt(xiH_, XI_PRM_EXPOSURE, time);
  errorHandling(stat, "xiOSetParamInt (Exposure Time)");
  if (stat == XI_OK) exposure_set_ = time;
}

// setGain - public function / sets gain in dB / 0 - 15 default ??? ...
// --------------------------------------------------------------------
void ximea_driver::setGain(float db){
  XI_RETURN stat = xiSetParamFloat(xiH_, XI_PRM_GAIN, db);
  errorHandling(stat, "xiOSetParamInt (Gain)");
  if (stat == XI_OK) gain_set_ = db;
}

/*
readSensorTemperature - / public function temperature reading of thermometer chip. 
sensor is located on the PCB close to imaging sensor. Units: degrees of Celsius ...
-----------------------------------------------------------------------------------
*/
float ximea_driver::readSensorTemperature (){
   
    float temperature_chip_temp = 0.0;    
    XI_RETURN stat = xiGetParamFloat(xiH_, XI_PRM_CHIP_TEMP, &temperature_chip_temp); 
    
    if (stat == XI_OK){ 
        chip_temperature_ = temperature_chip_temp;
        return temperature_chip_temp;
    }else{
        temperature_chip_temp = -1.0;
        return temperature_chip_temp;
    }    
}

// setAutoExposure - public function / automatic exposure/gain ...
// ---------------------------------------------
void ximea_driver::setAutoExposure(int auto_exposure){
    
  XI_RETURN stat = xiSetParamInt(xiH_, XI_PRM_AEAG, auto_exposure);
  errorHandling(stat, "xiOSetParamInt (AutoExposure Time)");
  if (!stat){
     // auto_exposureme_ = time;
  }
}

// setAutoExposureLimit - public function / maximum limit of exposure (in uSec) in AEAG procedure
// ----------------------------------------------------------------------------------------------
void ximea_driver::setAutoExposureLimit(int ae_limit){
  XI_RETURN stat = xiSetParamFloat(xiH_, XI_PRM_AE_MAX_LIMIT, ae_limit);
  errorHandling(stat, "xiOSetParamInt (AutoExposure Limit Time)");
  if (!stat){
    // auto_exposureme_ = time;
  }
}

// setAutoGainLimit - public function / maximum limit of gain in AEAG procedure ...
// --------------------------------------------------------------------------------
void ximea_driver::setAutoGainLimit(int ag_limit){
  XI_RETURN stat = xiSetParamInt(xiH_, XI_PRM_AG_MAX_LIMIT, ag_limit);
  errorHandling(stat, "xiOSetParamInt (AutoExposure Limit GAIN)");
  if (!stat){
    // auto_exposureme_ = time;
  }
}

// setAutoExposurePriority - public function /exposure priority for Auto Exposure / Auto Gain function ...
// -------------------------------------------------------------------------------------------------------
void ximea_driver::setAutoExposurePriority(float exp_priority){
  XI_RETURN stat = xiSetParamFloat(xiH_, XI_PRM_EXP_PRIORITY, exp_priority);
  errorHandling(stat, "xiOSetParamInt (AutoExposure Priority)");
  if (!stat){
    // auto_exposureme_ = time;
  }
}

// readParamsFromFile - public function ...
// ----------------------------------------
int ximea_driver::readParamsFromFile(std::string file_name){
  
  std::ifstream fin(file_name.c_str());
  
  if(fin.fail()){
     ROS_ERROR_STREAM("could not open file " << file_name.c_str() << std::endl);
     exit(-1);
  }

  YAML::Node doc = YAML::LoadFile(file_name);
  std::string tmpS;
  int tmpI1, tmpI2, tmpI3, tmpI4;
  bool tmpB;

  try{
    serial_no_ = doc["serial_no"].as<int>();     
  } catch (std::runtime_error) {}
   
  try{
    cam_name_ =  doc["cam_name"].as<std::string>();
  } catch (std::runtime_error) {}
  
  try{
    frame_id_ =  doc["frame_id"].as<std::string>();
  } catch (std::runtime_error) {}
  
  try{
    yaml_url_ = doc["yaml_url"].as<std::string>();
  } catch (std::runtime_error) {}

  try{
    cams_on_bus_ = doc["cams_on_bus"].as<int>();
  } catch (std::runtime_error) {}

  try{
    bandwidth_safety_margin_ = doc["bandwidth_safety_margin"].as<int>();
  } catch (std::runtime_error) {}

  try{
    frame_rate_ = doc["frame_rate"].as<int>();
  } catch (std::runtime_error) {}
  
  try{
    exposure_time_ = doc["exposure_time"].as<int>();
  } catch (std::runtime_error) {}
    
  try{
    exposure_time_1_ = doc["exposure_time_1"].as<int>();
  } catch (std::runtime_error) {}  
  
  try{
    exposure_time_2_ = doc["exposure_time_2"].as<int>();
  } catch (std::runtime_error) {}  

  try{
    exposure_time_3_ = doc["exposure_time_3"].as<int>();
  } catch (std::runtime_error) {}  
  
  try{
    exposure_time_4_ = doc["exposure_time_4"].as<int>();
  } catch (std::runtime_error) {}  
  
  try{
    gain_ = doc["manual_gain"].as<float>();
  } catch (std::runtime_error) {}  
    
  try{
    gain_1_ = doc["manual_gain_1"].as<float>();
  } catch (std::runtime_error) {}
  
  try{
    gain_2_ = doc["manual_gain_2"].as<float>();
  } catch (std::runtime_error) {}
  
  try{
    gain_3_ = doc["manual_gain_3"].as<float>();
  } catch (std::runtime_error) {}
  
  try{
    gain_4_ = doc["manual_gain_4"].as<float>();
  } catch (std::runtime_error) {}
  
  try{
    auto_exposure_ = doc["auto_exposure"].as<int>();
  } catch (std::runtime_error) {}

  try{
    auto_exposure_limit_ = doc["auto_exposure_limit"].as<int>();
  } catch (std::runtime_error) {}

  try{
    auto_gain_limit_ = doc["auto_gain_limit"].as<int>();
  } catch (std::runtime_error) {}

  try{
    auto_exposure_priority_ = doc["auto_exposure_priority"].as<float>();
  } catch (std::runtime_error) {}

  try{
    binning_enabled_ = doc["binning_enabled"].as<bool>();
  } catch (std::runtime_error) {}

  try{
    downsample_factor_ = doc["downsample_factor_"].as<int>();
  } catch (std::runtime_error) {}

  try {
    rect_left_ = doc["rect_left"].as<int>();
  } catch (std::runtime_error) {}

  try{
    rect_top_ = doc["rect_top"].as<int>();
  } catch (std::runtime_error) {}
  
  try{
    rect_width_ = doc["rect_width"].as<int>();
  } catch (std::runtime_error) {}

  try{
    rect_height_ = doc["rect_height"].as<int>();
  } catch (std::runtime_error) {}
  
  // Setting of ROI is disabled ...
  // ------------------------------ 
  // setROI(rect_left_, rect_top_, rect_width_, rect_height_);
 
  try{
    image_data_format_ = doc["image_data_format"].as<std::string>();
  } catch (std::runtime_error) {}
  
   setImageDataFormat(image_data_format_);
}

// enableTrigger - public function ...
// -----------------------------------
void ximea_driver::enableTrigger(unsigned char trigger_mode)
{
  if (!xiH_) return;
    trigger_mode_ = trigger_mode;
  if (trigger_mode_ > 3 || trigger_mode_ < 0){
    trigger_mode_ = 0;
  }
  
  XI_RETURN stat;
  
  switch (trigger_mode_){
  case (0):
    break;
  case (1):
    stat = xiSetParamInt(xiH_, XI_PRM_TRG_SOURCE, XI_TRG_SOFTWARE);
    errorHandling(stat, "Could not enable software trigger");
    break;
  case (2):
    break;
  default:
    break;
  }
}

// triggerDevice - public function ...
// -----------------------------------
void ximea_driver::triggerDevice()
{
  if (!xiH_) return;
     XI_RETURN stat;
  switch (trigger_mode_){
  case (0):
    break;
  case (1):
    stat = xiSetParamInt(xiH_, XI_PRM_TRG_SOFTWARE, 1);
    errorHandling(stat, "Error During triggering");
    std::cout << "triggering " << cam_name_ <<  std::endl;
    break;
  case (2):
    break;
  default:
    break;
  }
}

// limitBandwidth - public function / default is single camera ...
// ---------------------------------------------------------------
void ximea_driver::limitBandwidth(int mbps){
  if (!xiH_) return;
  XI_RETURN stat;
  // stat = xiSetParamInt(xiH_, XI_PRM_LIMIT_BANDWIDTH , mbps);
  // errorHandling(stat, "could not limit bandwidth");
}
