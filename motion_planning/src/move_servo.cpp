#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include <iostream>
#include <sstream>
#include "control.h"
#include "motion_planning/dataSetPoint.h"

#define id_servo 3
#define id_servo2 4

////////////////////////////////////////Sistem Identification//////////////////////////////////////////////////////
int nomor, pulse;

void chatterCallback(const motion_planning::dataSetPoint& subscMsg)
{
   nomor=subscMsg.counter;
   pulse=subscMsg.PRBS;
}

int main(int argc,char** argv)
{
    ros::init(argc, argv, "coba");
    ros::NodeHandle n;    

    control mimo;

    mimo.initPublishPosition();         // publish & advertise data position to master
    mimo.motors.update_data();          //class control(mimo) --> class dynamixel(motors) --> fungtion update_data();

    mimo.TS =20;                       // Time Sampling from class control --> 10=10ms, 100=100ms, 1000=1s
    ros::Rate loop_rate(1000/mimo.TS);  
    
    ROS_INFO("Start");
    while(ros::ok())
    {
		//updata data 20 servo
		mimo.update_data(1,20); //first id, last id
		
		mimo.publishPosition();
		ros::spinOnce();
	    loop_rate.sleep();
    }
}



