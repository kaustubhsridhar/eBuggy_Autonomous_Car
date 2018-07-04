#include "ros/ros.h"
#include "std_msgs/String.h"
#include "reciever/Cmd.h"
#include <sstream>
#include <string.h>

//for serial port reading
#include <stdio.h>      
#include <unistd.h>     
#include <fcntl.h>      
#include <errno.h>      
#include <termios.h>    
#include <iostream>

using namespace std;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "talker");
	ros::NodeHandle n;
	ros::Publisher rec_pub = n.advertise<reciever::Cmd>("/rec_data", 1000);
	//ros::Rate loop_rate(10);

	//	FOR SERIAL PORT READING
	int  serial_port = open( "/dev/ttyACM0", O_RDWR| O_NOCTTY );
	//fcntl(serial_port, F_SETFL, 0);
	if(serial_port == -1) /* Error Checking */
		printf("\n Error! in Opening ttyUSB0 ");
	else
		printf("\n ttyUSB0 Opened Successfully ");
	
	struct termios serial_options;				/*Define the POSIX structure*/
	tcgetattr(serial_port, &serial_options);		/*Read the attribute structure*/
	cfsetispeed(&serial_options, B9600);			/*Set the baud rate of the port in input (i) and output (i) to 9600*/
	cfsetospeed(&serial_options, B9600);

	/*Define other parameters in order to  realize the 8N1 standard*/
	serial_options.c_cflag &= ~PARENB; /* Disables the Parity Enable bit(PARENB),So No Parity */
	serial_options.c_cflag &= ~CSTOPB; /* CSTOPB = 2 Stop bits,here it is cleared so 1 Stop bit */
	serial_options.c_cflag &= ~CSIZE; /* Clears the mask for setting the data size */
	serial_options.c_cflag |= CS8; /* Set the data bits = 8 */
	serial_options.c_cflag &= ~CRTSCTS; /* No Hardware flow Control */
	serial_options.c_cflag |= CREAD | CLOCAL; /* Enable receiver,Ignore Modem Control lines */
	serial_options.c_iflag &= ~(IXON | IXOFF | IXANY); /* Disable XON/XOFF flow control both i/p and o/p */
	serial_options.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG); /* Non Cannonical mode */
	serial_options.c_oflag &= ~OPOST;/*No Output Processing*/
	
	//tcsetattr(serial_port, TCSANOW, &serial_options);	/*Apply the new attributes */
	if((tcsetattr(serial_port,TCSANOW,&serial_options)) != 0) 	/* Set and Check the attributes to the termios structure*/
		printf("\n ERROR ! in Setting attributes");
	else
		printf("\n BaudRate = 9600 \n StopBits = 1 \n Parity = none");
	tcflush(serial_port, TCIFLUSH); 			/* Discards old data in the rx buffer */
	//char buf[1000];						/*Now, we read from the data stream into 'buf' string, then we close the port */
	int bytes_read = 0;					/* Number of bytes read by the read() system call */

	while (ros::ok())
		{
			//std_msgs::String msg;
			reciever::Cmd msg;
			//std::stringstream ss;
			//ss << "hello world ";
			//msg.data = ss.str();
			//ROS_INFO("%s", msg.data.c_str());
			char buf[1000];	
			bytes_read = read( serial_port, &buf , VEOL);	/*get data from serial port here*/
			

			char line[bytes_read+1];
			for(int i=0;i<bytes_read;i++) 			/*finding length of buf*/
				line[i]=buf[i];//printf("%c",buf[i]);
			line[bytes_read+1]='\0';
			cout<<line<<endl;

			stringstream strstr(buf);
			string word = "";
			//while (getline(strstr,word, ';')) cout << word << '\n';					
									
			rec_pub.publish(msg);
			ros::spinOnce();
			//loop_rate.sleep();
		}
	//for serial port	
	close(serial_port);
	
	return 0;
}
