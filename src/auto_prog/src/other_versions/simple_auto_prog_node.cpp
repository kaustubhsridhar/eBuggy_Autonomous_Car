#include "ros/ros.h"
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>

#include "auto_prog/Gen_Cmd.h"
#include "cam_data/Yaw.h"
#include "reciever/Cmd.h"

#include <sstream>
#include <string>
#include <iostream>

#include <pthread.h> //for C++ multithreading

// this is autonomous program that generates control commands and publishes to /command topic via auto_prog::Gen_Cmd message
// currently subscribes to /rec_data and send same to /command
// this will also subscribe to /cam_yaw in the future 

using namespace std;

class SubscribeAndPublish_torec
{
public:
  SubscribeAndPublish_torec()
  {
    read_pub = nh.advertise<auto_prog::Gen_Cmd>("/command", 1000);
    write_sub = nh.subscribe("/rec_data", 1000, &SubscribeAndPublish_torec::rec_data_callback, this);
  }

  void rec_data_callback(const reciever::Cmd::ConstPtr& rec_msg) 
  {
    auto_prog::Gen_Cmd msg;
    //.... do something with the input and generate the output...
    ROS_INFO_STREAM("Reading from /rec_data");
    msg.mode = rec_msg->mode;	
    msg.servo = rec_msg->servo;
    msg.vel = rec_msg->vel;
    ROS_INFO_STREAM("Publishing to /command");
    read_pub.publish(msg);
  }

private:
  ros::NodeHandle nh; 
  ros::Publisher read_pub;
  ros::Subscriber write_sub;

};//End of class SubscribeAndPublish_torec



class SubscribeAndPublish_tocam
{
public:
  SubscribeAndPublish_tocam()
  {
    read_pub = nh.advertise<auto_prog::Gen_Cmd>("/command", 1000);
    cam_sub = nh.subscribe("/cam_yaw", 1000, &SubscribeAndPublish_tocam::cam_yaw_callback, this);
  }

  void cam_yaw_callback(const cam_data::Yaw::ConstPtr& cam_msg) 
  {
    auto_prog::Gen_Cmd msg;
    //.... do something with the input and generate the output...
    ROS_INFO_STREAM("Reading from /cam_yaw");
    msg.mode = 1;	
    msg.servo = cam_msg->yaw_num;
    msg.vel = 0;
    ROS_INFO_STREAM("Publishing to /command");
    read_pub.publish(msg);
  }

private:
  ros::NodeHandle nh; 
  ros::Publisher read_pub;
  ros::Subscriber cam_sub;

};//End of class SubscribeAndPublish_tocam



int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "subscribe_and_publish");

  //Create an object of class SubscribeAndPublish that will take care of everything
  
  //SubscribeAndPublish_tocam SAPObject_cam; //this if you want to use camera
  SubscribeAndPublish_torec SAPObject_rec; //this if you want to use receiver

  ros::spin();

  return 0;
}
