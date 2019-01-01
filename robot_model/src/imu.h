#ifndef IMU_H
#define IMU_H

#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <boost/bind.hpp>

using namespace std;
using namespace boost;

class Imu{
    private:
        ros::NodeHandle node;
        ros::Subscriber sub;
    public:
        int roll;
        int pitch;
        int yaw;
        void update_imu();
        void imu_callback(const geometry_msgs::Vector3& msg);
};

void Imu::update_imu()
{
    sub = node.subscribe("/sensor_imu",1,&Imu::imu_callback,this);
}

void Imu::imu_callback(const geometry_msgs::Vector3& msg)
{
    roll = msg.x;
    pitch = msg.y;
    yaw = msg.z;
}

#endif // IMU_H
