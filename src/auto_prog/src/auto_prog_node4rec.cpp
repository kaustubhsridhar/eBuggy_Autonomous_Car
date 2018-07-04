#include "ros/ros.h"
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>

#include "auto_prog/Gen_Cmd.h"
#include "cam_data/Yaw.h"
#include "reciever/Cmd.h"

#include <sstream>
#include <string>
#include <iostream>


// this is autonomous program that generates control commands and publishes to /command topic via auto_prog::Gen_Cmd message
// currently subscribes to /rec_data and send same to /command


using namespace std;

class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    read_pub = nh.advertise<auto_prog::Gen_Cmd>("/command", 1000);
    write_sub = nh.subscribe("/rec_data", 1000, &SubscribeAndPublish::rec_callback, this);
    rec_updated = false;
  }

  void rec_callback(const reciever::Cmd::ConstPtr& rec_msg) 
  {
    //.... do something with the input and generate the output...
    ROS_INFO_STREAM("Reading from /rec_data");
    rec_given_mode = rec_msg->mode;	
    rec_given_servo = rec_msg->servo;
    rec_given_vel = rec_msg->vel;
    rec_updated = true;

    if(rec_updated)
      Calc_and_Pub();
  }

  void Calc_and_Pub()
  {
    auto_prog::Gen_Cmd msg;
    msg.mode = rec_given_mode;	
    msg.vel = rec_given_vel;
    msg.servo = rec_given_servo;

    ROS_INFO_STREAM("Publishing to /command");
    read_pub.publish(msg);    
  }

private:
  ros::NodeHandle nh; 
  ros::Publisher read_pub;
  ros::Subscriber write_sub;
  float rec_given_mode, rec_given_servo, rec_given_vel;
  bool rec_updated;

};//End of class SubscribeAndPublish


int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "subscribe_and_publish");

  //Create an object of class SubscribeAndPublish that will take care of everythin
  SubscribeAndPublish SAPObject;

  ros::spin();

  return 0;
}
