#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <stdio.h>
#include "dynamixel.h"
#include "motion_planning/dataSetPoint.h"
#include "motion_planning/displayData.h"
#include "control.h"
#include <sstream>

#define id_servo 14
#define id_servo2 3
#define id_servo3 12

float outMimo[21]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

// this function for collecting multivariable control data
/*void chatterCallback(const motion_planning::displayData& subscMsg)
{
    outMimo[id_servo]=subscMsg.outMimo[id_servo];
    outMimo[id_servo2]=subscMsg.outMimo[id_servo2];
    outMimo[id_servo3]=subscMsg.outMimo[id_servo3];
}*/

// this function for step jumping of humanoid robot
void chatterCallback(const motion_planning::displayData& subscMsg)
{
	outMimo[1]=subscMsg.outMimo[1];
    outMimo[2]=subscMsg.outMimo[2];
    outMimo[3]=subscMsg.outMimo[3];
    outMimo[4]=subscMsg.outMimo[4];
    outMimo[5]=subscMsg.outMimo[5];
    outMimo[6]=subscMsg.outMimo[6];
    outMimo[7]=subscMsg.outMimo[7];
    outMimo[8]=subscMsg.outMimo[8];
    outMimo[9]=subscMsg.outMimo[9];
    outMimo[10]=subscMsg.outMimo[10];
    outMimo[11]=subscMsg.outMimo[11];
    outMimo[12]=subscMsg.outMimo[12];
    outMimo[14]=subscMsg.outMimo[14];
}

//////////////////////////////////////////////// __________________________ //////////////////////////////////////////////
////////////////////////////////////////////////|                          |//////////////////////////////////////////////
////////////////////////////////////////////////|Publish PRBS to ROS Master|//////////////////////////////////////////////
////////////////////////////////////////////////|__________________________|//////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*int main(int argc,char** argv)
{
    ros::init(argc, argv, "PRBS");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<motion_planning::dataSetPoint>("chatter", 1000);
    //ros::Subscriber sub = n.subscribe("displayData", 1000, chatterCallback);

    Dynamixel motors;
    motion_planning::dataSetPoint SP;

    int ctr=0, TS=20;

    //min=-20, max=20 --> dataLoadID4ofID2, dataLoadID3ofID1
    //min=-7, max=15 --> dataLoadID2ofID4
    //min=-15, max=7 --> dataLoadID1ofID3
    //min=-7, max=7  --> dataLoadID9ID11ofID7 also data from left foot
    //min=-10, max=10  --> dataLoadID7ID11ofID9 also data from left foot
    //min=-10, max=10  --> dataLoadID9ID11ofID11 also data from left foot

    float min=-55, max=95;	
    
    // Time Sampling from class control --> 10=10ms, 100=100ms, 1000=1s
    ros::Rate loop_rate(1000/TS);  
    ROS_INFO("Start");

    for(int i=1; i<=20; i++)
    {
        motors.setspeed(i,0.5);
    }
    sleep(1);

    //for(int i=1; i<=20; i++)
    //{
    //    motors.moveMotor(i,0);
    //} 
    //sleep(4);
    //for(int i=1; i<=20; i++)
    //{
    //    motors.setspeed(i,8);
    //}
    //sleep(10);

    //for ID3 ID4
    //motors.setspeed(id_servo,8);
    //motors.setspeed(id_servo2,8);
    //motors.setspeed(id_servo3,8);

    while(ros::ok())
    {
		SP.counter = ctr;
		printf("%d\n",SP.counter);
		
	if(ctr<=400){
	    SP.setPoint[id_servo]=min;	
	    SP.PRBS=min;
	}
	else if(ctr>400 && ctr<=700){	 
	    SP.setPoint[id_servo]=max;	
	    SP.PRBS=max;
	}
	else if(ctr>700 && ctr<=1000){
	    SP.setPoint[id_servo]=min;		
	    SP.PRBS=min;
	}
	else if(ctr>1000 && ctr<=1500){
	    SP.setPoint[id_servo]=max;
	    SP.PRBS=max;
	}
	else if(ctr>1500 && ctr<=1800){
	    SP.setPoint[id_servo]=min;		
	    SP.PRBS=min;
	}
	else if(ctr>1800 && ctr<=2200){
	    SP.setPoint[id_servo]=max;		
	    SP.PRBS=max;
	}
	else if(ctr>2200 && ctr<=2500){
	    SP.setPoint[id_servo]=min;		
	    SP.PRBS=min;
	}
	else ctr=0;


		//SP.setPoint[id_servo2]=SP.setPoint[id_servo];

		//motors.moveMotor_deg(id_servo,SP.setPoint[id_servo]); 
		//motors.moveMotor_deg(id_servo2,SP.setPoint[id_servo]*-1); 

		//Ambil data kaki kiri ID 4 --> ID 4 = -10 & ID 2 --> min = -20, max = 20
		//Ambil data kaki kiri ID 2 --> ID 2 = -10 & ID 4 --> min = -3, max = 15
		//Ambil data kaki kanan ID 3 --> ID 1 = -7 & min = -10, max = 20

		//motors.moveMotor_deg(id_servo2, 10); 
		motors.moveMotor_deg(id_servo,SP.setPoint[id_servo]); 

		chatter_pub.publish(SP);

		ctr++;
	    ros::spinOnce();
	    loop_rate.sleep();
    }
}*/



//////////////////////////////////////////////// _______________________________________ //////////////////////////////////
////////////////////////////////////////////////|                                       |//////////////////////////////////
////////////////////////////////////////////////| Subscribe MIMO from node mimo_control |//////////////////////////////////
////////////////////////////////////////////////|             &                         |//////////////////////////////////
////////////////////////////////////////////////|  Move Servo After Controlled          |//////////////////////////////////
////////////////////////////////////////////////|_______________________________________|//////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*int main(int argc,char** argv)
{
    ros::init(argc, argv, "PRBS");
    ros::NodeHandle n;
    //ros::Publisher chatter_pub = n.advertise<motionPlanning::dataSetPoint>("chatter", 1000);
    ros::Subscriber sub = n.subscribe("displayData", 1000, chatterCallback);

    Dynamixel motors;
    motion_planning::dataSetPoint SP;


    motors.setspeed(id_servo,0.5);
    motors.setspeed(id_servo2,0.5);
    motors.setspeed(id_servo3,0.5);

	for(int i=0; i<5; i++){
	    motors.moveMotor_deg(id_servo, 0);
		motors.moveMotor_deg(id_servo2, 0);
		motors.moveMotor_deg(id_servo3, 0);
	}

	sleep(1);

    //Set Speed Servo
    motors.setspeed(id_servo,8);
    motors.setspeed(id_servo2,8);
    motors.setspeed(id_servo3,8);

    int ctr=0, TS=20;
    
    // Time Sampling from class control --> 10=10ms, 100=100ms, 1000=1s
    ros::Rate loop_rate(1000/TS);  
    ROS_INFO("Start");

    while(ros::ok())
    {
		//printf("pos[%d]= %.2f pos[%d]= %.2f \n",id_servo, outMimo[id_servo],id_servo2, outMimo[id_servo2]);
    	printf("pos[%d]= %.2f pos[%d]= %.2f pos[%d]= %.2f \n",id_servo, outMimo[id_servo],id_servo2, outMimo[id_servo2],id_servo3, outMimo[id_servo3]);
    	motors.moveMotor_deg(id_servo, outMimo[id_servo]);
		motors.moveMotor_deg(id_servo2, outMimo[id_servo2]);
		motors.moveMotor_deg(id_servo3, outMimo[id_servo3]);
 
    	ros::spinOnce();
    	loop_rate.sleep();
    }
}*/

//////////////////////////////////////////////// _______________________________________ //////////////////////////////////
////////////////////////////////////////////////|                                       |//////////////////////////////////
////////////////////////////////////////////////| Sbscribe MIMO from node motion_control|//////////////////////////////////
////////////////////////////////////////////////|             &                         |//////////////////////////////////
////////////////////////////////////////////////|  Move Servo After Controlled          |//////////////////////////////////
////////////////////////////////////////////////|_______________________________________|//////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int main(int argc,char** argv)
{
    ros::init(argc, argv, "PRBS");
    ros::NodeHandle n;
    //ros::Publisher chatter_pub = n.advertise<motionPlanning::dataSetPoint>("chatter", 1000);
    ros::Subscriber sub = n.subscribe("displayData", 1000, chatterCallback);

    Dynamixel motors;
    motion_planning::dataSetPoint SP;


    for(int i=1; i<=20; i++) motors.setspeed(i,0.5);
	sleep(1);

	for(int i=1; i<=20; i++)motors.moveMotor(i,0);
	sleep(2);

    //Set Speed Servo
    //motors.setspeed(id_servo,8);
    //motors.setspeed(id_servo2,8);
    //motors.setspeed(id_servo3,8);

    int ctr=0, TS=20;
    
    // Time Sampling from class control --> 10=10ms, 100=100ms, 1000=1s
    ros::Rate loop_rate(1000/TS);  
    ROS_INFO("Start");

    while(ros::ok())
    {
		//printf("pos[%d]= %.2f pos[%d]= %.2f \n",id_servo, outMimo[id_servo],id_servo2, outMimo[id_servo2]);
    	//printf("pos[%d]= %.2f pos[%d]= %.2f pos[%d]= %.2f \n",id_servo, outMimo[id_servo],id_servo2, outMimo[id_servo2],id_servo3, outMimo[id_servo3]);
    	//motors.moveMotor_deg(14, outMimo[14]);
    	motors.moveMotor_deg(14, 10);
    	motors.moveMotor_deg(1, outMimo[1]);
		motors.moveMotor_deg(2, outMimo[2]);
		motors.moveMotor_deg(3, outMimo[3]);
		motors.moveMotor_deg(4, outMimo[4]);
		motors.moveMotor_deg(5, outMimo[5]);
		motors.moveMotor_deg(6, outMimo[6]);
		motors.moveMotor_deg(7, outMimo[7]);
		motors.moveMotor_deg(8, outMimo[8]);
		motors.moveMotor_deg(9, outMimo[9]);
		motors.moveMotor_deg(10, outMimo[10]);
		motors.moveMotor_deg(11, outMimo[11]);
		motors.moveMotor_deg(12, outMimo[12]);
 
    	ros::spinOnce();
    	loop_rate.sleep();
    }
}

//////////////////////////////////////////////// __________________________ //////////////////////////////////////////////
////////////////////////////////////////////////|                          |//////////////////////////////////////////////
////////////////////////////////////////////////|  Generate Data of PRBS   |//////////////////////////////////////////////
////////////////////////////////////////////////|                          |//////////////////////////////////////////////
////////////////////////////////////////////////|__________________________|//////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	//PRBS for ID 1 - ID 4
	/*if(ctr<=100){
	    SP.setPoint[id_servo]=min;	
	    SP.PRBS=min;
	}
	else if(ctr>100 && ctr<=200){	 
	    SP.setPoint[id_servo]=max;	
	    SP.PRBS=max;
	}
	else if(ctr>200 && ctr<=300){
	    SP.setPoint[id_servo]=min;		
	    SP.PRBS=min;
	}
	else if(ctr>300 && ctr<=350){
	    SP.setPoint[id_servo]=max;
	    SP.PRBS=max;
	}
	else if(ctr>350 && ctr<=450){
	    SP.setPoint[id_servo]=min;		
	    SP.PRBS=min;
	}
	else if(ctr>450 && ctr<=600){
	    SP.setPoint[id_servo]=max;
	    SP.PRBS=max;
	}
	else if(ctr>600 && ctr<=700){
	    SP.setPoint[id_servo]=min;		
	    SP.PRBS=min;
	}
	else ctr=0;

	//ID 5 - ID 8 dan Mimo dataLoadID4of2/dataLoadID3of1
	if(ctr<=400){
	    SP.setPoint[id_servo]=min;	
	    SP.PRBS=min;
	}
	else if(ctr>400 && ctr<=800){	 
	    SP.setPoint[id_servo]=max;	
	    SP.PRBS=max;
	}
	else if(ctr>800 && ctr<=1000){
	    SP.setPoint[id_servo]=min;		
	    SP.PRBS=min;
	}
	else if(ctr>1000 && ctr<=1300){
	    SP.setPoint[id_servo]=max;
	    SP.PRBS=max;
	}
	else if(ctr>1300 && ctr<=1550){
	    SP.setPoint[id_servo]=min;		
	    SP.PRBS=min;
	}
	else if(ctr>1550 && ctr<=1700){
	    SP.setPoint[id_servo]=max;
	    SP.PRBS=max;
	}
	else if(ctr>1700 && ctr<=2000){
	    SP.setPoint[id_servo]=min;		
	    SP.PRBS=min;
	}
	else ctr=0;


	//Mimo dataLoadID2of4/dataLoadID3of1
	if(ctr<=400){
	    SP.setPoint[id_servo]=min;	
	    SP.PRBS=min;
	}
	else if(ctr>400 && ctr<=700){	 
	    SP.setPoint[id_servo]=max;	
	    SP.PRBS=max;
	}
	else if(ctr>700 && ctr<=1000){
	    SP.setPoint[id_servo]=min;		
	    SP.PRBS=min;
	}
	else if(ctr>1000 && ctr<=1500){
	    SP.setPoint[id_servo]=max;
	    SP.PRBS=max;
	}
	else if(ctr>1500 && ctr<=1800){
	    SP.setPoint[id_servo]=min;		
	    SP.PRBS=min;
	}
	else if(ctr>1800 && ctr<=2200){
	    SP.setPoint[id_servo]=max;		
	    SP.PRBS=max;
	}
	else if(ctr>2200 && ctr<=2500){
	    SP.setPoint[id_servo]=min;		
	    SP.PRBS=min;
	}
	else ctr=0;
	*/




