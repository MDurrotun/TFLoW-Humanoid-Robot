#ifndef DYNAMIXEL_H
#define DYNAMIXEL_H

/* Developed by Naufal Suryanto - ER2C PENS */

#include<ros/ros.h>
#include<ros/service.h>
#include<std_msgs/Float64.h>
#include<dynamixel_msgs/JointState.h>
#include<stdio.h>
#include<boost/bind.hpp>

#define id_max 20
#define degtorad 0.01745329252

using namespace std;
class Dynamixel{
    private:
        ros::NodeHandle node[id_max];
        ros::NodeHandle node_sub[id_max];
        ros::Publisher pub[id_max];
        ros::Subscriber sub[id_max];
        int selector;


    public:
        Dynamixel();
        int32_t temperature[id_max+1];
        float goal_pos[id_max+1];
        float current_pos[id_max+1];
        float error[id_max+1];
        float velocity[id_max+1];
        float load[id_max+1];
        bool is_moving[id_max+1];

        void jointstate_callback(const dynamixel_msgs::JointState& msg);
        void jointstate_callback_command(const std_msgs::Float64::ConstPtr& msg, int i);
        int moveMotor(int id, double position);
        int moveMotorAll(double value[], int start_id, int last_id);
        int moveMotor_deg(int id, double position);
        int moveMotor_degAll(float value[], int start_id, int last_id);
        void setspeed(int id, float speed);
        void entorque(int id,bool enable);
        void update_data();
        void update_data_command();

};

Dynamixel::Dynamixel(){
    char buf[30];
    /*for(int i=0; i<id_max; i++)
    {

        sprintf(buf,"/joint%d_controller/command",i+1);
        //ROS_INFO(buf);
        Dynamixel::pub[i] = Dynamixel::node[i].advertise<std_msgs::Float64>(buf,1);
    }
    ROS_INFO("Dynamixel Command Topic Created");*/
}

int Dynamixel::moveMotor(int id, double position)
{
    std_msgs::Float64 aux;
    aux.data = position;
    Dynamixel::pub[id-1].publish(aux);
    //ROS_INFO("kirim id %d , data : %f",id, position);
    return 1;
}

int Dynamixel::moveMotorAll(double value[], int start_id, int last_id)
{
    for(int i=start_id; i<=last_id; i++)
    {
        std_msgs::Float64 aux;
        aux.data = value[i];
        Dynamixel::pub[i-1].publish(aux);
    }
}

int Dynamixel::moveMotor_deg(int id, double position)
{
    std_msgs::Float64 aux;
    aux.data = position * 0.01745329252;
    pub[id-1].publish(aux);
    return 1;
}

int Dynamixel::moveMotor_degAll(float value[], int start_id, int last_id)
{
    for(int i=start_id; i<=last_id; i++)
    {
        std_msgs::Float64 aux;
        aux.data = value[i] * 0.01745329252;
        Dynamixel::pub[i-1].publish(aux);
    }
    //ROS_INFO("publishing servo data");
}

void Dynamixel::update_data()
{
    char buf[30];
    for(int i=0; i<id_max; i++)
    {
        sprintf(buf,"/joint%d_controller/state",i+1);
        sub[i] = node_sub[i].subscribe(buf,1,&Dynamixel::jointstate_callback,this);
    }
    //ROS_INFO("data_updated");
}

void Dynamixel::update_data_command()
{
    char buf[30];
    for(int i=0; i<id_max; i++)
    {
        sprintf(buf,"/joint%d_controller/command",i+1);
        selector = i+1;
        //sub[i] = node_sub[i].subscribe(buf,1,&Dynamixel::jointstate_callback_command,this);
        sub[i] = node_sub[i].subscribe<std_msgs::Float64>(buf,100,boost::bind(&Dynamixel::jointstate_callback_command, this, _1, i+1));
    }

    //ROS_INFO("data_updated");

}

void Dynamixel::jointstate_callback(const dynamixel_msgs::JointState& msg)
{
    //int32_t i;
    int i = msg.motor_ids[0];
    Dynamixel::goal_pos[i] = msg.goal_pos;
    Dynamixel::current_pos[i] = msg.current_pos;
    Dynamixel::error[i] = msg.error;
    Dynamixel::load[i] = msg.load;
    Dynamixel::velocity[i] = msg.velocity;
    Dynamixel::is_moving[i] = msg.is_moving;
    //cout << "ID " << i << " : " << Dynamixel::current_pos[i] << endl;
}

void Dynamixel::jointstate_callback_command(const std_msgs::Float64::ConstPtr& msg, int index)
{
    //int32_t i;
    Dynamixel::current_pos[index] = msg->data;
    //ROS_INFO("data_updated id %d - %g",index,);
    if(index==19) ROS_INFO("Nilai %d-%g",index,Dynamixel::current_pos[index]);
    //cout << "ID " << index << " : " << Dynamixel::current_pos[index] << endl;
}

void Dynamixel::setspeed(int id, float speed)
{
    char buf[100];
    sprintf(buf,"rosservice call /joint%d_controller/set_speed \"speed: %g\"",id,speed);
    ROS_INFO(buf);
    system(buf);
}

void Dynamixel::entorque(int id, bool enable)
{
    char buf[100];
    if(enable == true)
    {
        sprintf(buf,"rosservice call /joint%d_controller/torque_enable \"torque: true\"",id);
    }
    else sprintf(buf,"rosservice call /joint%d_controller%d/torque_enable \"torque: false\"",id);
    system(buf);
}

#endif // DYNAMIXEL_H
