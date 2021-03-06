/***
 * This example expects the serial port has a loopback on it.
 *
 * Alternatively, you could use an Arduino:
 *
 * <pre>
 *  void setup() {
 *    Serial.begin(<insert your baudrate here>);
 *  }
 *
 *  void loop() {
 *    if (Serial.available()) {
 *      Serial.write(Serial.read());
 *    }
 *  }
 * </pre>
 */

#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Char.h>
#include <iostream>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <serial_example/dataIMU.h>
#include "std_msgs/Float32.h"

serial::Serial ser;
std_msgs::String hasil;
std::string str;
float dataX, dataY, dataZ;
char receiveDataX[2], receiveDataY[2], receiveDataZ[2], receiveDataAll[3];
int tampung, x=0, ctr=0;

int DataMasukA[2];
unsigned char IterasiA, ReadyA;
int LSB, MSB;
double akselerasiZ, load, flag=0;

void write_callback(const std_msgs::String::ConstPtr& msg){
    //ROS_INFO_STREAM("Writing to serial port" << msg->data);
    ser.write(msg->data);
}

void pass(){
	if(load>=10&&flag==0){ load=0; flag=1;}
}

///////////////////////////////////About Parsing Data////////////////////////////////
/*///////////////////////////////
	a x d(dataX)
	b x e(dataY)
	c x f(dataZ)
	-----------
	X Y Z
	0 0 0
	0 0 1
	0 1 0
	0 1 1
	1 0 0
	1 0 1
	1 1 0
	1 1 1
	-----------
	a b c 
	a b f 
	a e c
	a e f
	d b c
	d b f
	d e c
	d e f
------------------------
0 = plus
1 = minus
------------------------
parsingData = (abc)(dataX)(dataY)(dataZ)
example     = abc504020

*////////////////////////////////////////////////////////////////////////////////////

void reset_receiveDataAll(){
	receiveDataAll[0]=NULL;	
	receiveDataAll[1]=NULL;
	receiveDataAll[2]=NULL;
}

void konversi(){
	load = load*0.8;
}

void parsingDataIMU(){

    if(ser.available()){
	//str = ser.read(ser.available()); 
    hasil.data = ser.read(ser.available()); 

	ROS_INFO_STREAM(hasil.data);
	//printf("%c-%c-%c-%c-%c-%c-%c-%c-%c\n", hasil.data[0], hasil.data[1], hasil.data[2], hasil.data[3], hasil.data[4], hasil.data[5], hasil.data[6], hasil.data[7], hasil.data[8]);

	/*for(int i=3; i<str.length(); i++){
	    if(str[i]=='*'){
		    dataX=atof(receiveDataAll);
		    x = 0;
	    } 
	    else if(str[i]=='#'){
		    dataY=atof(receiveDataAll);
		    x = 0;
	    }
	    else if(str[i]=='@'){
		    dataZ=atof(receiveDataAll); 
			x = 0;
			str.clear();
	    }
	    else receiveDataAll[x++]=str[i];
	}*/

	for(int i=3; i<hasil.data.length(); i++){
	    if(hasil.data[i]=='*'){
		    dataX=atof(receiveDataAll);
		    reset_receiveDataAll();
		    x = 0;
	    } 
	    else if(hasil.data[i]=='#'){
		    dataY=atof(receiveDataAll);
		    reset_receiveDataAll();
		    x = 0;
	    }
	    else if(hasil.data[i]=='@'){
		    dataZ=atof(receiveDataAll); 
		    reset_receiveDataAll();
		    x = 0;
	    }
	    else receiveDataAll[x++]=hasil.data[i];
	}


	//dataX=atof(receiveDataX); dataY=atof(receiveDataY); dataZ=atof(receiveDataZ);

	if(hasil.data[0]=='a'&&hasil.data[1]=='b'&&hasil.data[2]=='f'){
	    //XPlus, YPlus, ZMinus
	    dataZ*=-1;
	}
	else if(hasil.data[0]=='a'&&hasil.data[1]=='e'&&hasil.data[2]=='c'){
	    //XPlus, YMinus, ZPlus
	    dataY*=-1;
	}
	else if(hasil.data[0]=='a'&&hasil.data[1]=='e'&&hasil.data[2]=='f'){
	    //XPlus, YMinus, ZMinus
	    dataY*=-1; dataZ*=-1;
	}
	else if(hasil.data[0]=='d'&&hasil.data[1]=='b'&&hasil.data[2]=='c'){
	    //XMinus, YPlus, ZPlus
	    dataX*=-1;
	}
	else if(hasil.data[0]=='d'&&hasil.data[1]=='b'&&hasil.data[2]=='f'){
	    //XMinus, YPlus, ZMinus
	    dataX*=-1; dataZ*=-1;
	}
	else if(hasil.data[0]=='d'&&hasil.data[1]=='e'&&hasil.data[2]=='c'){
	    //XMinus, YMinus, ZPlus
	    dataX*=-1; dataY*=-1;
	}
	else if(hasil.data[0]=='d'&&hasil.data[1]=='e'&&hasil.data[2]=='f'){
	    //XMinus, YMinus, ZMinus
	    dataX*=-1; dataY*=-1; dataZ*=-1;
	}

	//printf("X=%.2f, Y=%.2f, Z=%.2f\n", dataX, dataY, dataZ);
    }

}

void readAccelero(){

    if(ser.available()){
    	//hasil.data = ser.read(ser.available()); 
	//ROS_INFO_STREAM(hasil.data);

	hasil.data = ser.read(ser.available()); 
	//ROS_INFO_STREAM(hasil.data);
	//printf("%c-%c-%c-%c\n", hasil.data[0], hasil.data[1], hasil.data[2], hasil.data[3]);

	for(int i=0; i<hasil.data.length(); i++){
	    if(hasil.data[i]=='*'){
	        LSB = atoi(receiveDataAll) & 0x00ff;
	        //MSB = atoi(receiveDataAll[1]) << 8;
	        //accZ = (LSB | MSB) / 1000;
	        reset_receiveDataAll();
	        x = 0;
	    } 
	    else if(hasil.data[i]=='#'){
	        //LSB = atoi(receiveDataAll);
	        MSB = atoi(receiveDataAll) << 8;
	        akselerasiZ = (double)(LSB | MSB) / 1000;
	        reset_receiveDataAll();
	        x = 0;
	    } 
	    else{
	        receiveDataAll[x]=hasil.data[i];
	        x++;		
	    }
        }
	//printf("accZ = %.2f\n", akselerasiZ);
    }
}

void readLoadCell(){

    if(ser.available()){
	    	//hasil.data = ser.read(ser.available()); 
		//ROS_INFO_STREAM(hasil.data);

		hasil.data = ser.read(ser.available()); 
		//ROS_INFO_STREAM(hasil.data);
		//printf("%c-%c-%c-%c\n", hasil.data[0], hasil.data[1], hasil.data[2], hasil.data[3]);

		for(int i=0; i<hasil.data.length(); i++){
		    if(hasil.data[i]=='*'){
		        LSB = atoi(receiveDataAll) & 0x00ff;
		        reset_receiveDataAll();
		        x = 0;
		    } 
		    else if(hasil.data[i]=='#'){
		        MSB = atoi(receiveDataAll) << 8;
		        load = (double)(LSB | MSB) / 1000;
		        reset_receiveDataAll();
		        x = 0;
		    } 
		    else{
		        receiveDataAll[x]=hasil.data[i];
		        x++;		
		    }
	    }
	    konversi();
	    pass();
		//printf("accZ = %.2f\n", load);
    }
}

int main (int argc, char** argv){
    ros::init(argc, argv, "serial_example_node");
    ros::NodeHandle nh;

    //serial_example::dataIMU acceleroZ;
    std_msgs::Float32 acceleroZ;
    std_msgs::Float32 loadCell;

    //ros::Subscriber write_sub = nh.subscribe("write", 1000, write_callback);
    //ros::Publisher pub = nh.advertise<std_msgs::Float32>("acceleroZ", 1000);
    ros::Publisher pub = nh.advertise<std_msgs::Float32>("loadCell", 1000);

    try
    {
        ser.setPort("/dev/ttyUSB1");
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(3);
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

    ros::Rate loop_rate(1000);
 
    while(ros::ok()){
    	readLoadCell();
		loadCell.data = load;
		printf("load = %.2f\n", loadCell.data);
		//ROS_INFO_STREAM(loadCell.data);

		//readAccelero();
		//acceleroZ.data = akselerasiZ;
		//ROS_INFO_STREAM(acceleroZ.data);
	
		//parsingDataIMU();

		pub.publish(loadCell);
    	//pub.publish(acceleroZ);
        ros::spinOnce();
        loop_rate.sleep();
    }
}




/*
#include "ros/ros.h"
#include <sensor_msgs/JointState.h>
#include <std_msgs/Header.h>
#include "serial/serial.h"
#include <string>
#include <iostream>
#include <cstdio>
 
using std::string;
 
int pozisyon = 0;
int ang=0;
double encoder_resolution=4200;
 
 
char a[] = {"base_tilt_joint"}; // 
 
double pos[]={0.0}; 
double vel[]={0.0};
double eff[]={0.0};
serial::Serial my_serial("/dev/ttyACM1", 57600, serial::Timeout::simpleTimeout(10));
sensor_msgs::JointState robot_state;
 
double degreTOrad(double degre);
//string trim(const string& str);
 
int main(int argc, char **argv)
{
 
  ros::init(argc, argv, "joint_state_publisher");
 
  ros::NodeHandle n;
  
  std_msgs::Header header;
 
 
  ros::Publisher chatter_pub = n.advertise<sensor_msgs::JointState>("joint_states", 10);
 
  ros::Rate loop_rate(100);
 
 
 
  robot_state.header;
  robot_state.header.stamp=ros::Time::now();
  robot_state.name.resize(1);
  robot_state.velocity.resize(1);
  robot_state.position.resize(1); /// here used for arduino time
  robot_state.effort.resize(1); /// here used for arduino time
 
    robot_state.name[0]=(a);
    pos[0]=degreTOrad(pozisyon);
	
 
 
    robot_state.position[0] = pos[0];
    robot_state.velocity[0] = vel[0];
    robot_state.effort[0] = eff[0];

    if(my_serial.isOpen())  // if connection is successful
      std::cout << " Yes." <<std:: endl;
    else
      std::cout << " No." <<std:: endl;

  while (ros::ok())
  {
 
    string s= my_serial.read(32);
 if(s[0]=='!'&&s.find("#")!=-1){
  string sss=s.substr(s.find("!")+1,s.find("#")-1);

  int a= std::stoi((sss.c_str()));
  pozisyon=a;
}
 
   ang=((pozisyon/encoder_resolution)*360.0);  // calculate angel according to pulse count
   ang=int(ang)%360;
  // ROS_INFO("anglee: %d",(ang));
   
   
    pos[0]=degreTOrad(ang);
    robot_state.header.stamp=ros::Time::now();
    robot_state.position[0] = pos[0];
    robot_state.velocity[0] = 0;
    robot_state.effort[0] = 0;
   
    chatter_pub.publish(robot_state);
 
    ros::spinOnce();
 
    loop_rate.sleep();
 
  }
 
 
  return 0;
}
 
double degreTOrad(double degre){   //degre to radyan for rviz
  
  
  return (degre/57.2958);
  
  }*/


