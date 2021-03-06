// some ros includes
#include <ros/ros.h>
#include <ros/package.h>

// some std includes
#include <stdlib.h>
#include <stdio.h>

// message
//#include <ximea_camera/XimeaMeta.h>
#include <ximea_camera/XimeaCamSensorMsg.h>
//#include <rospix/Image.h>
#include <std_msgs/Float64.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>

using namespace std;

// define subscribers
image_transport::Subscriber image_subscriber;
ros::Subscriber meta_subscriber;

// stores the name of the directory to save files
string directory;

double date;

//void metadataCallBack(const ximea_camera::XimeaMetaConstPtr& msg)
void metadataCallBack(const ximea_camera::XimeaCamSensorMsgConstPtr& msg)
{
    // create a filename based on current time, precision on miliseconds    
    char filename[40];
    date =  msg->stamp.toSec();
    sprintf(filename, "%.3f.txt", date);  

    // add the directory name
    string path = string(directory+string(filename));
    
    // open the file
    FILE * f = fopen(path.c_str(), "w");
    
    if (f == NULL) {
        ROS_ERROR("Cannot open the file %s for writing.", path.c_str());
    } else {
        // print all metadata
        fprintf(f, "time: %f\n", msg->stamp.toSec());
        fprintf(f, "exposure_time: %f\n", msg->exposure_time);
        fprintf(f, "gain: %u\n", msg->gain);
        fprintf(f, "temperature: %f\n", msg->temperature);

        fflush(f);
        fclose(f);
    
        string fname= string(filename);
        ROS_INFO("Metadata for %s saved", fname.c_str());
    }
}

void imageCallBack(const sensor_msgs::ImageConstPtr& msg)
{
    // create a filename based on current time, precision on miliseconds    
    char filename[40];
    date =  msg->header.stamp.toSec();
    sprintf(filename, "%.3f.tiff", date);

    // add the directory name
    string path = string(directory+string(filename));

    // save image to file
    cv_bridge::CvImagePtr cv_ptr;    
    try{
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e){
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    if (!cv::imwrite(path.c_str(), cv_ptr->image)) {
        ROS_ERROR("Cannot save the image file.");
    } else {
      string fname= string(filename);
      ROS_INFO("Image %s saved", fname.c_str());
    }
}

int main(int argc, char **argv)
{
  //init node
  ros::init(argc, argv, "saver_ximea");
  ros::NodeHandle nh_ = ros::NodeHandle("~");
 
  // load parameters from config file (launch file)
  nh_.param("directory", directory, string());

  // subscribers
  meta_subscriber = nh_.subscribe("/camera0/ximea_info_settings", 10, &metadataCallBack, ros::TransportHints().tcpNoDelay());
//  meta_subscriber = nh_.subscribe("/camera0/ximea_meta", 10, &metadataCallBack, ros::TransportHints().tcpNoDelay());
  image_transport::ImageTransport it(nh_);
  image_subscriber = it.subscribe("/camera0/image_raw", 1, &imageCallBack);
    
  ros::spin();

  return 0;
}
