#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include <iostream>
#include <sstream>
#include "dynamixel.h"
#include "control.h"
#include "motion_planning/dataSetPoint.h"
#include "math.h"

#define id_servo 8
#define id_servo2 10
#define id_servo3 12

int nomor;
float pulse;

void chatterCallback(const motion_planning::dataSetPoint& subscMsg)
{
   nomor=subscMsg.counter;
   //pulse=subscMsg.PRBS*0.01745329252;
   pulse=subscMsg.PRBS;
}

//////////////////////////////////////////////// _____________________________ ////////////////////////////////////////////
////////////////////////////////////////////////|                             |////////////////////////////////////////////
////////////////////////////////////////////////|Subscribe PRBS from node PRBS|////////////////////////////////////////////
////////////////////////////////////////////////|_____________________________|////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*int main(int argc,char** argv)
{
    ros::init(argc, argv, "mimo_control");
    ros::NodeHandle n;    
    ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
    //ros::Publisher chatter_pub = n.advertise<motion_Planning::dataSetPoint>("SP", 1000);

    Dynamixel motors;
    control mimo;

    mimo.initPublishPosition();         // publish & advertise data position to master
    mimo.motors.update_data();          //class control(mimo) --> class dynamixel(motors) --> fungtion update_data();

    mimo.TS =20;                        // Time Sampling from class control --> 10=10ms, 100=100ms, 1000=1s
    ros::Rate loop_rate(1000/mimo.TS);  

    mimo.resetData();
    
    ROS_INFO("Start");
    while(ros::ok())
    {
		//updata data 20 servo
		mimo.update_data(1,20); //first id, last id
		
		//Collect data from node PRBS to modelling a motors
		
		//filter
		//mimo.data.pos[id_servo] = mimo.filter(mimo.data.pos[id_servo]);
		//mimo.data.loadServo[id_servo] = mimo.filter(mimo.data.loadServo[id_servo]);
		//mimo.data.loadServo[id_servo2] = mimo.filter(mimo.data.loadServo[id_servo2]);
		//mimo.data.loadServo[id_servo3] = mimo.filter(mimo.data.loadServo[id_servo3]);

		//Data Servo per ID
		printf("%d SetPoint[%d]= %.2f position[%d]= %.2f \n",nomor, id_servo, pulse, id_servo, mimo.data.pos[id_servo]);

		//Data 2 Servo(MIMO)
		//printf("%d SetPoint[%d]= %.2f position[%d]= %.2f , SetPoint[%d]= %.2f position[%d]= %.2f \n",nomor, id_servo, pulse, id_servo, mimo.data.pos[id_servo], id_servo2, pulse, id_servo2, mimo.data.pos[id_servo2]);
		
		//Collision Data on Any Joint with position feedback
		//printf("%d SetPoint[%d]= %.2f position[%d]= %.2f \n",nomor, id_servo, pulse, id_servo2, mimo.data.pos[id_servo2]);

		//Collision Data on Any Joint with load feedback
		//printf("%d SP[%d]= %.2f , load[%d] = %.2f , load[%d] = %.2f \n",nomor, id_servo, pulse, id_servo, mimo.data.loadServo[id_servo], id_servo2, mimo.data.loadServo[id_servo2]);
		//printf("%d SP[%d]= %.2f , load[%d] = %.2f , load[%d] = %.2f , load[%d] = %.2f \n",nomor, id_servo, pulse, id_servo, mimo.data.loadServo[id_servo], id_servo2, mimo.data.loadServo[id_servo2], id_servo3, mimo.data.loadServo[id_servo3]);

		//Data Selisih / resultan
		//printf("%d SetPoint[%d]= %.2f , SetPoint[%d]= %.2f , Selisih= %.2f \n",nomor, id_servo, pulse, id_servo2, pulse*-1, mimo.data.selisih);

		mimo.publishPosition();
		ros::spinOnce();
    	loop_rate.sleep();
    }
}*/

//////////////////////////////////////////////// _____________________________________ ///////////////////////////////////
////////////////////////////////////////////////|                                     |///////////////////////////////////
////////////////////////////////////////////////|Publish MIMO Controller to ROS Master|///////////////////////////////////
////////////////////////////////////////////////|_____________________________________|///////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int main(int argc,char** argv)
{
    ros::init(argc, argv, "coba");
    ros::NodeHandle n;    
    //ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
    ros::Publisher chatter_pub = n.advertise<motion_planning::dataSetPoint>("SP", 1000);

    control mimo;
    motion_planning::dataSetPoint SP;

    mimo.initPublishPosition();         // publish & advertise data position to master
    mimo.motors.update_data();          //class control(mimo) --> class dynamixel(motors) --> fungtion update_data();

    mimo.TS =20;                        // Time Sampling from class control --> 10=10ms, 100=100ms, 1000=1s
    ros::Rate loop_rate(1000/mimo.TS);  

    int ctr=0;

    for(int i=0; i<=10; i++){
        mimo.resetData();
    }
    mimo.update_data(1,20); //first id, last id

    usleep(3000000);
    
    ROS_INFO("Start");
    while(ros::ok())
    {
	//updata data 20 servo
	mimo.update_data(1,20); //first id, last id
	
	/////////////////////////////////////////////////////////////////////////////////////////////
	// Single PID Controller
	if(ctr<=200){
	    mimo.setPoint[id_servo]=0;
	    mimo.setPoint[id_servo2]=0;
	    mimo.setPoint[id_servo3]=0;
	}
	else if(ctr>200 && ctr<400){
	    mimo.setPoint[id_servo]=10;
	    mimo.setPoint[id_servo2]=10;
	    mimo.setPoint[id_servo3]=-10;
	}
	else ctr=0;

    //Multivariable Control
	//mimo.singlePID(id_servo, 0, 1.7, 0, 100); //fix ID 1
	//mimo.singlePID(id_servo, 0, 1.8, 0, 100); //coba ID 1
	//mimo.singlePID(id_servo, 0, 2.71, 0, 100); //fix ID 2
	//mimo.singlePID(id_servo2, 0, 2.71, 0, 100); //fix ID 3
	//mimo.singlePID(id_servo2, 0, 1.8, 0, 100); //fix ID 4
	//mimo.singlePID(id_servo, 0, 2.21, 0, 100); //fix ID 5
	//mimo.singlePID(id_servo, 0, 2.21, 0, 100); //fix ID 6
	//mimo.singlePID(id_servo, 0, 2.42, 0, 100); //fix ID 7
	//mimo.singlePID(id_servo, 0, 2.21, 0, 100); //fix ID 8
	//mimo.singlePID(id_servo, 0, 2.21, 0, 100); //fix ID 9
	//mimo.singlePID(id_servo2, 0, 3.372, 0, 100); //fix ID 10
	//mimo.singlePID(id_servo, 0, 2.21, 0, 100); //fix ID 11
	//mimo.singlePID(id_servo3, 0, 3.31, 0, 100); //fix ID 12
	//mimo.mimoPID(id_servo, id_servo2, 0.6,1,0,100, 0.7,1,0,100);   //coba mimo osilasi
	//mimo.mimoPID(id_servo, id_servo2, 0,1.7,0,100, 0,0.3,0,100); //MIMO engkel Kiri
	//mimo.mimoPID(id_servo, id_servo2, 0,2.5,0,100, 0,2.5,0,100); //MIMO engkel Kiri fast
	//mimo.mimoPID(id_servo, id_servo2, 0,1.8,0,100, 0,2.71,0,100); //MIMO engkel Kanan
	//mimo.mimo3x3(id_servo, id_servo2, id_servo3, 0,2.21,0,100, 0,3.372,0,100, 0,3.31,0,100); //MIMO Hip Kiri
	mimo.mimo3x3(id_servo, id_servo2, id_servo3, 0,2.21,0,100, 0,3.01,0,100, 0,4.10,0,100); //MIMO Hip Kanan

	//copy data setpoint to message dataSetPoint & tes filter data
	//SP.setPoint[id_servo]=mimo.setPoint[id_servo]; 
	//SP.setPoint[id_servo2]=mimo.setPoint[id_servo2]; 
	//SP.setPoint[id_servo] = mimo.filter(mimo.data.pos[id_servo]);
	//SP.setPoint[id_servo2] = mimo.filter(mimo.data.pos[id_servo2]);


	/////////////////////////////////////////////////////////////////////////////////////////////
	//mimo.data.pos[id_servo] = mimo.filter(mimo.data.pos[id_servo]);
	//Monitoring Single PID
	//printf("SP[%d]= %.2f pos= %.2f e=%.2f i=%.2f d=%.2f %.2f\n",id_servo,SP.setPoint[id_servo],mimo.data.pos[id_servo], mimo.Error[id_servo], mimo.IError[id_servo], mimo.DError[id_servo], mimo.data.outMimo[id_servo]);
	//printf("SP[%d]= %.2f pos= %.2f \n",id_servo,mimo.setPoint[id_servo],mimo.data.pos[id_servo]);

	//monitoring mimoPID2x2
	//printf("SP[%d]= %.2f pos[%d]= %.2f SP[%d]= %.2f pos[%d]= %.2f  \n",id_servo,mimo.setPoint[id_servo],id_servo,mimo.data.pos[id_servo], id_servo2,mimo.setPoint[id_servo2],id_servo2,mimo.data.pos[id_servo2]);

	//monitoring mimoPID3x3
	printf("SP[%d]= %.2f pos[%d]= %.2f SP[%d]= %.2f pos[%d]= %.2f SP[%d]= %.2f pos[%d]= %.2f  \n",id_servo,mimo.setPoint[id_servo],id_servo,mimo.data.pos[id_servo], id_servo2,mimo.setPoint[id_servo2],id_servo2,mimo.data.pos[id_servo2], id_servo3,mimo.setPoint[id_servo3],id_servo3,mimo.data.pos[id_servo3]);

	//Collect data for System Identification
	//mimo.data.pos[id_servo] = mimo.filter(mimo.data.pos[id_servo]);
	//printf("ID=[%d] SetPoint=20.00 position[%d]= 19.88 \n",nomor, pulse, id_servo, mimo.data.pos[id_servo]);
	
	ctr++;
	mimo.publishPosition();
	chatter_pub.publish(SP);
	ros::spinOnce();
    loop_rate.sleep();
    }
}
