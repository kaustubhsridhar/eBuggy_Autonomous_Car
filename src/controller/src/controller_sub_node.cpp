#include "ros/ros.h"
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
// #include "controller/Control_Cmd.h"
#include "controller/Control_Data.h"

// #include "reciever/Cmd.h"	//for now using message corresponding to /rec_data. Change to that corresponding to /command topic's msg (ie auto_prog::Gen_Cmd)
#include "auto_prog/Gen_Cmd.h" //changed

#include <sstream>
#include <string>
#include "serial/serial.h"

// have to publish Control_Data from serial to topic /con_data and subscribe via Control_Cmd to /command topic
// keep controller at serial port /dev/ttyACM1

using namespace std;

serial::Serial ser;
controller::Control_Data msg;

string Convert (float number){
    std::ostringstream buff;
    buff<<number;
    return buff.str();   
}

/*void split(char const *line)
{

	char *ssl;//ssl = stop string location
	char const *ss1 = strstr(line,","); msg.mode = strtoul(line,&ssl,10); 
	char const *ss2 = strstr(ss1+1,","); msg.ref_vel = strtoul(ss1+1,&ssl,10);		
	char const *ss3 = strstr(ss2+1,","); msg.servo = strtoul(ss2+1,&ssl,10);
	char const *ss4 = strstr(ss3+1,","); msg.esc = strtoul(ss3+1,&ssl,10);
	char const *ss5 = strstr(ss4+1,","); msg.current = strtoul(ss4+1,&ssl,10);
	char const *ss6 = strstr(ss5+1,","); msg.lf = strtoul(ss5+1,&ssl,10);
	char const *ss7 = strstr(ss6+1,","); 
	char const *ss8 = strstr(ss7+1,","); msg.rf = strtoul(ss7+1,&ssl,10);
	char const *ss9 = strstr(ss8+1,","); 
	char const *ss10 = strstr(ss9+1,","); msg.lr = strtoul(ss9+1,&ssl,10);
	char const *ss11 = strstr(ss10+1,","); 
	char const *ss12 = strstr(ss11+1,","); msg.rr = strtoul(ss11+1,&ssl,10);
	char const *ss13 = strstr(ss12+1,","); 
	char const *ss14 = strstr(ss13+1,","); msg.motor = strtoul(ss13+1,&ssl,10);
	char const *ss15 = strstr(ss14+1,","); 
	char const *ss16 = strstr(ss15+1,","); msg.kalman_lf = strtoul(ss15+1,&ssl,10);
	char const *ss17 = strstr(ss16+1,","); msg.kalman_rf = strtoul(ss16+1,&ssl,10);
	char const *ss18 = strstr(ss17+1,","); msg.kalman_lr = strtoul(ss17+1,&ssl,10);
	char const *ss19 = strstr(ss18+1,","); msg.kalman_rr = strtoul(ss18+1,&ssl,10);
	char const *ss20 = strstr(ss19+1,","); msg.kalman_gy = strtoul(ss19+1,&ssl,10);	
	char const *ss21 = strstr(ss20+1,","); msg.kalman_ax = strtoul(ss20+1,&ssl,10);
	char const *ss22 = strstr(ss21+1,","); // 22 to 30 original fields have been pushed to 24 to 32 by Sunny // 2 dummy fields 22 and 23 added in between
	char const *ss23 = strstr(ss22+1,","); 
	char const *ss24 = strstr(ss23+1,","); msg.gx = strtoul(ss23+1,&ssl,10);
	char const *ss25 = strstr(ss24+1,","); msg.gy = strtoul(ss24+1,&ssl,10);
	char const *ss26 = strstr(ss25+1,","); msg.gz = strtoul(ss25+1,&ssl,10);
	char const *ss27 = strstr(ss26+1,","); msg.mx = strtoul(ss26+1,&ssl,10);
	char const *ss28 = strstr(ss27+1,","); msg.my = strtoul(ss27+1,&ssl,10);
	char const *ss29 = strstr(ss28+1,","); msg.mz = strtoul(ss28+1,&ssl,10);
	char const *ss30 = strstr(ss29+1,","); msg.ax = strtoul(ss29+1,&ssl,10);
	char const *ss31 = strstr(ss30+1,","); msg.ay = strtoul(ss30+1,&ssl,10);
	char const *ss32 = strstr(ss31+1,","); msg.az = strtoul(ss31+1,&ssl,10);

}*/

void write_callback(const auto_prog::Gen_Cmd::ConstPtr& write_msg) // ------------------------------------change here if needed
{

	ROS_INFO_STREAM("Writing to serial port");
	string send_line = Convert(write_msg->mode) + "," + Convert(write_msg->servo) + "," + Convert(write_msg->vel)+"\n";
	cout<< send_line<<endl;
	ser.write(send_line);

}


int main (int argc, char** argv){
	ros::init(argc, argv, "serial_example_node");
	ros::NodeHandle nh;

	//ros::Publisher read_pub = nh.advertise<controller::Control_Data>("/con_data", 1000);
	
	ros::Subscriber write_sub = nh.subscribe("/command", 1000, write_callback);	// ------------------------------------change here if needed

	try
	{
		ser.setPort("/dev/ttyACM0");
		ser.setBaudrate(115200);

		serial::Timeout to = serial::Timeout::simpleTimeout(1000);
		ser.setTimeout(to);

		ser.open();
	}
	catch (serial::IOException& e)
	{
		ROS_ERROR_STREAM("Unable to open port ");
		return -1;
	}
	if(ser.isOpen()){
		ROS_INFO_STREAM("Serial Port initialized");
	}else{
		return -1;
	}

	//ros::Rate loop_rate(20);
	while(ros::ok()){

		//ros::spinOnce(); //spinonce --> callback function called. subscribed.
		
		/*if(ser.available()){
			ROS_INFO_STREAM("Reading from serial port");
			//parsing only if 31 commas present
			string line_read = ser.readline();

			if (count(line_read.begin(), line_read.end(), ',') == 31){

				char const *line = line_read.c_str();
				cout<< line<<endl;

				//splitting (parsing) the comma separated line (string) [see ~/test.cpp to understand]
				split(line);
				read_pub.publish(msg);
			}
		}*/

		ros::spinOnce(); //spinonce --> callback function called. subscribed.
		//loop_rate.sleep();
	}
}

