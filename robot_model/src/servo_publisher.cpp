#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include "dynamixel.h"

#define degtorad 0.01745329252

double pos[21] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

void joint_callback(const sensor_msgs::JointState& msg)
{
  for(int i = 0; i < 20; i++)
  {
      pos[i+1] = msg.position[i];
  }
}


int main(int argc,char** argv)
{
    ros::init(argc, argv, "servo_publisher");
    ros::NodeHandle n;
    Dynamixel motors;
    float sudut=0;

    ros::Subscriber joint_state_sub = n.subscribe("joint_states",10,joint_callback);
    ROS_INFO("Start");
    motors.update_data();

    ros::Rate loop_rate(60);
    while(ros::ok())
    {
        for(int i=1; i<=20;i++)
        {
	    motors.setspeed(i,0.5);
	    //ROS_INFO("Position %d : %f",i,pos[i]);
            //motors.moveMotor_deg(i,sudut);
            motors.moveMotor(i,pos[i]);
            //motors.setspeed(i,12);

        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}
