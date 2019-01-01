/*Menentukan posisi motor dengan slider*/

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <sensor_msgs/JointState.h>
#include <iostream>
#include <sstream>
using namespace std;

/*
std_msgs/Header header
string[] name
float64[] position
float64[] velocity
float64[] effort
*/

ros::Publisher joint22, joint23; //kepala
ros::Publisher joint18, joint16; //tangan kiri
ros::Publisher joint19, joint17; //tangan kanan
ros::Publisher joint15, joint14, joint13; //perut
ros::Publisher joint11, joint9, joint7, joint5, joint3, joint1; //kaki kanan
ros::Publisher joint12, joint10, joint8, joint6, joint4, joint2; //kaki kiri

//buat message
std_msgs::Float64 pos22, pos23; //kepala
std_msgs::Float64 pos18, pos16; //tangan kiri
std_msgs::Float64 pos19, pos17; //tangan kanan
std_msgs::Float64 pos15, pos14, pos13; //perut
std_msgs::Float64 pos11, pos9, pos7, pos5, pos3, pos1;
std_msgs::Float64 pos12, pos10, pos8, pos6, pos4, pos2;

void number_callback(const sensor_msgs::JointState& msg)
{
  //ROS_INFO("");	
/*	
	cout
	<< "j1 :" << msg.position[0] << " "
	<< "j2 :" << msg.position[1] << " "
	<< "j3 :" << msg.position[2] << " "
	<< "j4 :" << msg.position[3] << " "
	<< "j5 :" << msg.position[4] << " "
	<< "j6 :" << msg.position[5] << " "
	<< "j7 :" << msg.position[6] << " "
	<< "j8 :" << msg.position[7] << " "
	<< "j9 :" << msg.position[8] << " "
	<< "j10 :" << msg.position[9] << " "
	<< "j11 :" << msg.position[10] << " "
	<< "j12 :" << msg.position[11] << " "
	<< "j13 :" << msg.position[12] << " "
	<< "j14 :" << msg.position[13] << " "
	<< "j15 :" << msg.position[14] << 	" "
	<< "j16 :" << msg.position[15] << " "
	<< "j17 :" << msg.position[16] << " "
	<< "j18 :" << msg.position[17] << " "
	<< "j19 :" << msg.position[28] << " "
	<< "j22 :" << msg.position[19] << " "
	<< "j23 :" << msg.position[20] << " "
	<< endl;
*/

	pos1.data=msg.position[4];
	pos2.data=msg.position[5];
	pos3.data=msg.position[6];
	pos4.data=msg.position[7];
	pos5.data=msg.position[8];
	pos6.data=msg.position[9];
	pos7.data=msg.position[10];
	pos8.data=msg.position[11];
	pos9.data=msg.position[12];
	pos10.data=msg.position[13];
	pos11.data=msg.position[14];
	pos12.data=msg.position[15];
	pos13.data=msg.position[16];
	pos14.data=msg.position[17];
	pos15.data=msg.position[18];
	pos16.data=msg.position[19];
	pos17.data=msg.position[20];
	pos18.data=msg.position[21];
	pos19.data=msg.position[22];
	pos22.data=msg.position[23];
	pos23.data=msg.position[24];

	//publish nilai message ke joint controller --> file yaml;

	joint1.publish(pos1);
	joint2.publish(pos2);
	joint3.publish(pos3);
	joint4.publish(pos4);
	joint5.publish(pos5);

	joint6.publish(pos6);
	joint7.publish(pos7);
	joint8.publish(pos8);
	joint9.publish(pos9);
	joint10.publish(pos10);

	joint11.publish(pos11);
	joint12.publish(pos12);
	joint13.publish(pos13);
	joint14.publish(pos14);
	joint15.publish(pos15);

	joint16.publish(pos16);
	joint17.publish(pos17);
	joint18.publish(pos18);
	joint19.publish(pos19);
	joint22.publish(pos22);

	joint23.publish(pos23);
	
}

int main(int argc, char **argv)
{
  ros::init(argc, argv,"tflow_kaki");
  ros::NodeHandle node_obj;
  ros::NodeHandle node_obj22, node_obj23;
  ros::NodeHandle node_obj15, node_obj14, node_obj13;
  ros::NodeHandle node_obj18, node_obj16;
  ros::NodeHandle node_obj19, node_obj17;
  ros::NodeHandle node_obj11, node_obj9, node_obj7, node_obj5, node_obj3, node_obj1;
  ros::NodeHandle node_obj12, node_obj10, node_obj8, node_obj6, node_obj4, node_obj2;
	
  joint1 = node_obj1.advertise<std_msgs::Float64>("/joint1_controller/command",10);
  joint2 = node_obj2.advertise<std_msgs::Float64>("/joint2_controller/command",10);
  joint3 = node_obj3.advertise<std_msgs::Float64>("/joint3_controller/command",10);
  joint4 = node_obj4.advertise<std_msgs::Float64>("/joint4_controller/command",10);
  joint5 = node_obj5.advertise<std_msgs::Float64>("/joint5_controller/command",10);
  joint6 = node_obj6.advertise<std_msgs::Float64>("/joint6_controller/command",10);
  
  joint7 = node_obj7.advertise<std_msgs::Float64>("/joint7_controller/command",10);
  joint8 = node_obj8.advertise<std_msgs::Float64>("/joint8_controller/command",10);
  joint9 = node_obj9.advertise<std_msgs::Float64>("/joint9_controller/command",10);
  joint10 = node_obj10.advertise<std_msgs::Float64>("/joint10_controller/command",10);
  joint11 = node_obj11.advertise<std_msgs::Float64>("/joint11_controller/command",10);
  joint12 = node_obj12.advertise<std_msgs::Float64>("/joint12_controller/command",10);

  joint13 = node_obj13.advertise<std_msgs::Float64>("/joint13_controller/command",10);
  joint14 = node_obj14.advertise<std_msgs::Float64>("/joint14_controller/command",10);
  joint15 = node_obj15.advertise<std_msgs::Float64>("/joint15_controller/command",10);
  joint16 = node_obj16.advertise<std_msgs::Float64>("/joint16_controller/command",10);
  joint17 = node_obj17.advertise<std_msgs::Float64>("/joint17_controller/command",10);
  joint18 = node_obj18.advertise<std_msgs::Float64>("/joint18_controller/command",10);
  joint19 = node_obj19.advertise<std_msgs::Float64>("/joint19_controller/command",10);

  joint22 = node_obj22.advertise<std_msgs::Float64>("/joint22_controller/command",10);
  joint23 = node_obj23.advertise<std_msgs::Float64>("/joint23_controller/command",10);

  ros::Subscriber joint_subscriber = node_obj.subscribe("joint_states",10,number_callback);
  ros::spin();
  return 0;
}
