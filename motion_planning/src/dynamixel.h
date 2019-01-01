#ifndef DYNAMIXEL_H
#define DYNAMIXEL_H

/* Developed by Naufal Suryanto - ER2C PENS */

#include<ros/ros.h>
#include<ros/service.h>
#include<std_msgs/Float64.h>
#include<std_msgs/String.h>
#include<dynamixel_msgs/JointState.h>
#include<dynamixel_controllers/SetSpeed.h>
#include<stdio.h>
#include<sstream>
//#include <boost/bind.hpp>

#define id_max 20
#define degtorad 0.01745329252

using namespace std;
class Dynamixel{
    private:
        ros::NodeHandle node[id_max];
        ros::NodeHandle node_sub[id_max];
        ros::Publisher pub[id_max];
        ros::Subscriber sub[id_max];
        ros::ServiceClient service;
        ros::NodeHandle n;

    public:
        Dynamixel();
	    int8_t counter;
        int32_t temperature[id_max+1];
        float goal_pos[id_max+1];
        float current_pos[id_max+1];
        float error[id_max+1];
        float velocity[id_max+1];
        float load[id_max+1];
        bool is_moving[id_max+1];

        int moveMotor(int id, double position);
        int moveMotorAll(double value[], int start_id, int last_id);
        int moveMotor_deg(int id, double position);
        int moveMotor_degAll(float value[], int start_id, int last_id);
        void setspeed(int id, float speed);
        void entorque(int id,bool enable);
        void update_data();

	
	    void jointstate_callback(const dynamixel_msgs::JointState& msg);
        void jointstate_call0(const dynamixel_msgs::JointState& msg);
        void jointstate_call1(const dynamixel_msgs::JointState& msg);
        void jointstate_call2(const dynamixel_msgs::JointState& msg);
        void jointstate_call3(const dynamixel_msgs::JointState& msg);
        void jointstate_call4(const dynamixel_msgs::JointState& msg);
        void jointstate_call5(const dynamixel_msgs::JointState& msg);
        void jointstate_call6(const dynamixel_msgs::JointState& msg);
        void jointstate_call7(const dynamixel_msgs::JointState& msg);
        void jointstate_call8(const dynamixel_msgs::JointState& msg);
        void jointstate_call9(const dynamixel_msgs::JointState& msg);
        void jointstate_call10(const dynamixel_msgs::JointState& msg);
        void jointstate_call11(const dynamixel_msgs::JointState& msg);
        void jointstate_call12(const dynamixel_msgs::JointState& msg);
        void jointstate_call13(const dynamixel_msgs::JointState& msg);
        void jointstate_call14(const dynamixel_msgs::JointState& msg);
        void jointstate_call15(const dynamixel_msgs::JointState& msg);
        void jointstate_call16(const dynamixel_msgs::JointState& msg);
        void jointstate_call17(const dynamixel_msgs::JointState& msg);
        void jointstate_call18(const dynamixel_msgs::JointState& msg);
        void jointstate_call19(const dynamixel_msgs::JointState& msg);
};

Dynamixel::Dynamixel(){
    char buf[30];
    for(int i=0; i<id_max; i++)
    {

        sprintf(buf,"/joint%d_controller/command",i+1);
        //ROS_INFO(buf);
        Dynamixel::pub[i] = Dynamixel::node[i].advertise<std_msgs::Float64>(buf,100);
    }
    ROS_INFO("Dynamixel Command Topic Created");
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
    //ROS_INFO("%g %g %g %g %g %g %g",value[7],value[8],value[9],value[10],value[11],value[12] );
}

void Dynamixel::setspeed(int id, float speed)
{
    //char buf[100];
    //sprintf(buf,"rosservice call /joint%d_controller/set_speed \"speed: %g\"",id,speed);

    char buf2[50];
    sprintf(buf2,"/joint%d_controller/set_speed",id);
    service = n.serviceClient<dynamixel_controllers::SetSpeed>(buf2);
    dynamixel_controllers::SetSpeed srv;
    srv.request.speed = speed;

    if(service.call(srv))
    {
        ROS_INFO("Speed id %d berhasil diganti menjadi %g",id,speed);
    }
    else ROS_INFO("Speed id %d gagal diganti ",id);


    //ROS_INFO(buf2);
    //system(buf);
}

/*    def set_speed(self, servo_id, speed):
        """
        Set the servo with servo_id to the specified goal speed.
        Speed can be negative only if the dynamixel is in "freespin" mode.
        """
        # split speed into 2 bytes
        if speed >= 0:
            loVal = int(speed % 256)
            hiVal = int(speed >> 8)
        else:
            loVal = int((1023 - speed) % 256)
            hiVal = int((1023 - speed) >> 8)

        # set two register values with low and high byte for the speed
        response = self.write(servo_id, DXL_GOAL_SPEED_L, (loVal, hiVal))
        if response:
            self.exception_on_error(response[4], servo_id, 'setting moving speed to %d' % speed)
        return response
*/

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


/////////////////////////////////////////////////Get Feedback Data from Dynamixel///////////////////////////////////
/*void Dynamixel::update_data()
{
    /*sub[0] = node_sub[0].subscribe("/joint1_controller/state",1,&Dynamixel::jointstate_call0,this);
    sub[1] = node_sub[1].subscribe("/joint2_controller/state",1,&Dynamixel::jointstate_call1,this);
    sub[2] = node_sub[2].subscribe("/joint3_controller/state",1,&Dynamixel::jointstate_call2,this);
    sub[3] = node_sub[3].subscribe("/joint4_controller/state",1,&Dynamixel::jointstate_call3,this);
    sub[4] = node_sub[4].subscribe("/joint5_controller/state",1,&Dynamixel::jointstate_call4,this);
    sub[5] = node_sub[5].subscribe("/joint6_controller/state",1,&Dynamixel::jointstate_call5,this);
    /*sub[6] = node_sub[6].subscribe("/joint7_controller/state",1,&Dynamixel::jointstate_call6,this);
    sub[7] = node_sub[7].subscribe("/joint8_controller/state",1,&Dynamixel::jointstate_call7,this);
    sub[8] = node_sub[8].subscribe("/joint9_controller/state",1,&Dynamixel::jointstate_call8,this);
    sub[9] = node_sub[9].subscribe("/joint10_controller/state",1,&Dynamixel::jointstate_call9,this);
    sub[10] = node_sub[10].subscribe("/joint11_controller/state",1,&Dynamixel::jointstate_call10,this);
    sub[11] = node_sub[11].subscribe("/joint12_controller/state",1,&Dynamixel::jointstate_call11,this);
    sub[12] = node_sub[12].subscribe("/joint13_controller/state",1,&Dynamixel::jointstate_call12,this);
    sub[13] = node_sub[13].subscribe("/joint14_controller/state",1,&Dynamixel::jointstate_call13,this);
    sub[14] = node_sub[14].subscribe("/joint15_controller/state",1,&Dynamixel::jointstate_call14,this);
    sub[15] = node_sub[15].subscribe("/joint16_controller/state",1,&Dynamixel::jointstate_call15,this);
    sub[16] = node_sub[16].subscribe("/joint17_controller/state",1,&Dynamixel::jointstate_call16,this);
    sub[17] = node_sub[17].subscribe("/joint18_controller/state",1,&Dynamixel::jointstate_call17,this);
    sub[18] = node_sub[18].subscribe("/joint19_controller/state",1,&Dynamixel::jointstate_call18,this);
    sub[19] = node_sub[19].subscribe("/joint20_controller/state",1,&Dynamixel::jointstate_call19,this);
}

/*void Dynamixel::jointstate_call0(const dynamixel_msgs::JointState& msg){current_pos[1] = msg.current_pos/degtorad;}
void Dynamixel::jointstate_call1(const dynamixel_msgs::JointState& msg){current_pos[2] = msg.current_pos/degtorad;}
void Dynamixel::jointstate_call2(const dynamixel_msgs::JointState& msg){current_pos[3] = msg.current_pos/degtorad;}
void Dynamixel::jointstate_call3(const dynamixel_msgs::JointState& msg){current_pos[4] = msg.current_pos/degtorad;}
void Dynamixel::jointstate_call4(const dynamixel_msgs::JointState& msg){current_pos[5] = msg.current_pos/degtorad;}
void Dynamixel::jointstate_call5(const dynamixel_msgs::JointState& msg){current_pos[6] = msg.current_pos/degtorad;}
/*void Dynamixel::jointstate_call6(const dynamixel_msgs::JointState& msg){current_pos[7] = msg.current_pos/degtorad;}
void Dynamixel::jointstate_call7(const dynamixel_msgs::JointState& msg){current_pos[8] = msg.current_pos/degtorad;}
void Dynamixel::jointstate_call8(const dynamixel_msgs::JointState& msg){current_pos[9] = msg.current_pos/degtorad;}
void Dynamixel::jointstate_call9(const dynamixel_msgs::JointState& msg){current_pos[10] = msg.current_pos/degtorad;}
void Dynamixel::jointstate_call10(const dynamixel_msgs::JointState& msg){current_pos[11] = msg.current_pos/degtorad;}
void Dynamixel::jointstate_call11(const dynamixel_msgs::JointState& msg){current_pos[12] = msg.current_pos/degtorad;}
void Dynamixel::jointstate_call12(const dynamixel_msgs::JointState& msg){current_pos[13] = msg.current_pos/degtorad;}
void Dynamixel::jointstate_call13(const dynamixel_msgs::JointState& msg){current_pos[14] = msg.current_pos/degtorad;}
void Dynamixel::jointstate_call14(const dynamixel_msgs::JointState& msg){current_pos[15] = msg.current_pos/degtorad;}
void Dynamixel::jointstate_call15(const dynamixel_msgs::JointState& msg){current_pos[16] = msg.current_pos/degtorad;}
void Dynamixel::jointstate_call16(const dynamixel_msgs::JointState& msg){current_pos[17] = msg.current_pos/degtorad;}
void Dynamixel::jointstate_call17(const dynamixel_msgs::JointState& msg){current_pos[18] = msg.current_pos/degtorad;}
void Dynamixel::jointstate_call18(const dynamixel_msgs::JointState& msg){current_pos[19] = msg.current_pos/degtorad;}
void Dynamixel::jointstate_call19(const dynamixel_msgs::JointState& msg){current_pos[20] = msg.current_pos/degtorad;}*/




void Dynamixel::update_data()
{
    char buf[30];
    for(int i=0; i<id_max; i++)     //for(int i=0; i<id_max; i++)
    {
        sprintf(buf,"/joint%d_controller/state",i+1);
        sub[i] = node_sub[i].subscribe(buf,1,&Dynamixel::jointstate_callback,this);
	    //ROS_INFO("position = %d", goal_pos[i]);
    }
    //ROS_INFO("data_updated");
}

void Dynamixel::jointstate_callback(const dynamixel_msgs::JointState& msg)
{
    //int32_t i;
    int i = msg.motor_ids[0];
    //Dynamixel::goal_pos[i] = msg.goal_pos;

    Dynamixel::current_pos[i] = msg.current_pos/degtorad; //nilai = derajat
    //Dynamixel::current_pos[i] = msg.current_pos; // nilai = radian

    //Dynamixel::error[i] = msg.error;
    //Dynamixel::load[i] = msg.load;
    //Dynamixel::velocity[i] = msg.velocity;
    //Dynamixel::is_moving[i] = msg.is_moving; 
    //cout << "ID " << 19 << " : " << Dynamixel::current_pos[19] * degtorad << endl;   
}


#endif // DYNAMIXEL_H
