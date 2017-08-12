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

#include <ximea_camera/ximea_ros_cluster.h>
#include <string>
#include <vector>
#include <iostream>

#define MAX_RECONFIG 25
#define MAX_RECONFIG_MOD 5
#define FRAME_SLEEP 1   // for camera chip cooling ...

int main(int argc, char ** argv){
    
  ros::init(argc, argv, "ximea");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  int frame_rate_=0;
  int counter_=0;
  
  std::vector<std::string> file_names;
  pnh.param<int>("frame_rate", frame_rate_, 10);
  pnh.getParam("camera_param_file_paths", file_names);
  ros::Rate loop(frame_rate_);
  ros::Rate loop_1(FRAME_SLEEP);
  
  // check size of camera file ...
  // -----------------------------
  if (file_names.size() == 0){
    ROS_ERROR("ximea_driver: No camera files name specified. Please set 'camera_param_file_paths' parameter to camera yaml file locations in launch file");
    return 0;
  }else{
    for (unsigned int i = 0; i < file_names.size(); i++){
       ROS_INFO_STREAM("loading camera parameter file: " << file_names[i] << std::endl);
    }
  }

  ximea_ros_cluster xd(file_names);
  xd.clusterInit();
  
  // TODO: need to robustify against replugging and cntrlc
  // -----------------------------------------------------
  while (ros::ok()){
    ros::spinOnce();
    xd.clusterReconfigure(counter_++); // increment counter_ after ...
    xd.clusterAcquire();
    xd.clusterPublishImageAndCamInfo();
    if(counter_ == MAX_RECONFIG){
       counter_=0; loop_1.sleep();
    }else if((counter_ % MAX_RECONFIG_MOD) == 0){
       loop_1.sleep(); 
    }else{    
       loop.sleep();
    }   
  }
  xd.clusterEnd();
  return 1;
}
