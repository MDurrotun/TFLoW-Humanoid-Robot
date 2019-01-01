#include <ros/ros.h>
#include <boost/thread/thread.hpp>
#include "dynamixel.h"
#include "loadparameter.h"
#include "motion4step.h"
#include <sensor_msgs/JointState.h>
#include "std_msgs/Float32.h"
#include <iostream>
#include <sstream>
#include "control.h"
#include "motion_planning/dataSetPoint.h"

#define id_servo 5
#define id_servo2 6
#define id_servo3 14
#define jumlahStep 1
#define timePerStep 20

float angleAll[21]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

int main(int argc,char** argv)
{
    ros::init(argc, argv, "motion_control");
    ros::NodeHandle n;
    //ros::Subscriber sub = n.subscribe("displayData", 1000, chatterCallback);
    //ros::Publisher publishAngle = n.advertise<motion_planning::dataAngle>("dataSudut", 1000);
    //motion_planning::dataAngle dataSudut;
    ros::Publisher chatter_pub = n.advertise<motion_planning::dataSetPoint>("SP", 1000);

    int ctr, gerakan=1, flag=0;
    float valueTheta[4];
    Dynamixel motors;
    Motion4step motion;
    motion_planning::dataSetPoint SP;
    Kinematics kinematics;
    control mimo;

    mimo.initPublishPosition();
    mimo.motors.update_data();

    for(int i=0; i<=10; i++){
        mimo.resetData();
    }
    mimo.update_data(1,20); //first id, last id

    ROS_INFO("Program Main Start");

    //Robot StandUp
    //motion.StepStartUp(motors);
    //sleep(1);

    // Time Sampling from class control --> 10=10ms, 100=100ms, 1000=1s             
    ros::Rate loop_rate(1000/20);  

    //trajectory palanning --> making robot can be stand up by itself with low speed
    /*for(int i=1; i<=20; i++)
    {
        motors.setspeed(i,0.5);
        usleep(500);
    }

   for(int i=1; i<=20; i++)
    {
        motors.moveMotor(i,0);
        usleep(500);
    }*/
    sleep(5);
    
    //Change speed motors to high speed
    for(int i=1; i<=20; i++) motors.setspeed(i,8);
    kinematics.initialPosition(0,0,0,0); 

    while(ros::ok())
    {
        mimo.update_data(1,15);

        if(gerakan==1){
            //initial posture berdiri
            valueTheta[0]=0;
            valueTheta[1]=0;
            valueTheta[2]=0;
            valueTheta[3]=0;

            kinematics.Angle_Jumping(valueTheta[0],valueTheta[1],valueTheta[2],valueTheta[3],100,1);

            mimo.setPoint[1]=kinematics.AngleJointAll[1];
            mimo.setPoint[2]=kinematics.AngleJointAll[2];
            mimo.setPoint[3]=kinematics.AngleJointAll[3];
            mimo.setPoint[4]=kinematics.AngleJointAll[4];
            mimo.setPoint[5]=kinematics.AngleJointAll[5];
            mimo.setPoint[6]=kinematics.AngleJointAll[6];
            mimo.setPoint[7]=kinematics.AngleJointAll[7];
            mimo.setPoint[8]=kinematics.AngleJointAll[8];
            mimo.setPoint[14]=kinematics.AngleJointAll[14];

            if(kinematics.step>1){
                gerakan=2;
                kinematics.initialPosition(valueTheta[0],valueTheta[1],valueTheta[2],valueTheta[3]);
                kinematics.resetKinematics(); 
                sleep(3);
            }

            mimo.mimoPID(1, 3, 0,0.8,0,100, 0,1.2,0,100); //MIMO Engkel Kanan
            mimo.mimoPID(2, 4, 0,0.9,0,100, 0,1.2,0,100); //MIMO engkel Kiri
            mimo.singlePID(5, 0, 0.8, 0, 100); //single PID Lutut kanan
            mimo.singlePID(6, 0, 0.8, 0, 100); //single PID Lutut kiri
            mimo.mimo3x3(7, 9, 11, 0,1.2,0,100, 0,1.2,0,100, 0,1.2,0,100); //MIMO Hip Kanan
            mimo.mimo3x3(8, 10, 12, 0,1.2,0,100, 0,1.2,0,100, 0,1.2,0,100); //MIMO Pinggul Kiri
            mimo.singlePID(14, 0, 1.5, 0, 100); //single PID perut
        }
        else if(gerakan==2){
             //gerakan jongkok
            valueTheta[0]=55;
            valueTheta[1]=100;
            valueTheta[2]=60;
            valueTheta[3]=20;

            kinematics.Angle_Jumping(valueTheta[0],valueTheta[1],valueTheta[2],valueTheta[3],100,jumlahStep);

            mimo.setPoint[1]=kinematics.AngleJointAll[1];
            mimo.setPoint[2]=kinematics.AngleJointAll[2];
            mimo.setPoint[3]=kinematics.AngleJointAll[3];
            mimo.setPoint[4]=kinematics.AngleJointAll[4];
            mimo.setPoint[5]=kinematics.AngleJointAll[5];
            mimo.setPoint[6]=kinematics.AngleJointAll[6];
            mimo.setPoint[7]=kinematics.AngleJointAll[7];
            mimo.setPoint[8]=kinematics.AngleJointAll[8];
            mimo.setPoint[14]=kinematics.AngleJointAll[14];
            //usleep(2000);
            if(kinematics.step>jumlahStep){
                gerakan=3;
                kinematics.initialPosition(valueTheta[0],valueTheta[1],valueTheta[2],valueTheta[3]);
                kinematics.resetKinematics(); 
            }

            mimo.mimoPID(1, 3, 0,0.8,0,100, 0,1.7,0,100); //MIMO Engkel Kanan
            mimo.mimoPID(2, 4, 0,0.8,0,100, 0,1.7,0,100); //MIMO Engkel Kiri
            mimo.singlePID(5, 0, 0.9, 0, 100); //single PID Lutut kanan
            mimo.singlePID(6, 0, 0.9, 0, 100); //single PID Lutut kiri
            mimo.mimo3x3(7, 9, 11, 0,1.21,0,100, 0,2.01,0,100, 0,3.10,0,100); //MIMO Pinggul Kanan
            mimo.mimo3x3(8, 10, 12, 0,1.21,0,100, 0,2.372,0,100, 0,2.31,0,100); //MIMO Pinggul Kiri
            mimo.singlePID(14, 0, 2, 0, 100); //single PID perut
        }
        else if(gerakan==3){
             //gerakan melayang
            valueTheta[0]=0;
            valueTheta[1]=0;
            valueTheta[2]=0;
            valueTheta[3]=0;

            kinematics.Angle_Jumping(valueTheta[0],valueTheta[1],valueTheta[2],valueTheta[3],20,jumlahStep);

            mimo.setPoint[1]=kinematics.AngleJointAll[1];
            mimo.setPoint[2]=kinematics.AngleJointAll[2];
            mimo.setPoint[3]=kinematics.AngleJointAll[3];
            mimo.setPoint[4]=kinematics.AngleJointAll[4];
            mimo.setPoint[5]=kinematics.AngleJointAll[5];
            mimo.setPoint[6]=kinematics.AngleJointAll[6];
            mimo.setPoint[7]=kinematics.AngleJointAll[7];
            mimo.setPoint[8]=kinematics.AngleJointAll[8];
            mimo.setPoint[14]=kinematics.AngleJointAll[14];

            if(kinematics.step>jumlahStep){
                gerakan=4;
                kinematics.initialPosition(valueTheta[0],valueTheta[1],valueTheta[2],valueTheta[3]);
                kinematics.resetKinematics(); 
            }


            /*if(flag==0){
                flag=1;
                mimo.IError[3]=-25; mimo.IError[4]=25;
                mimo.IError[5]=-50; mimo.IError[6]=50;
                //mimo.IError[7]=30; mimo.IError[8]=-30;
            }*/

            mimo.mimoPID(1, 3, 0,1.2,0,100, 0,1.7,0,100); //MIMO Engkel Kanan
            mimo.mimoPID(2, 4, 0,1.2,0,100, 0,1.7,0,100); //MIMO Engkel Kiri
            mimo.singlePID(5, 0, 1.2, 0, 100); //single PID Lutut kanan
            mimo.singlePID(6, 0, 1.2, 0, 100); //single PID Lutut kiri
            mimo.mimo3x3(7, 9, 11, 0,1.21,0,100, 0,2.01,0,100, 0,3.10,0,100); //MIMO Pinggul Kanan
            mimo.mimo3x3(8, 10, 12, 0,1.21,0,100, 0,2.372,0,100, 0,2.31,0,100); //MIMO Pinggul Kiri
            mimo.singlePID(14, 0, 2, 0, 100); //single PID perut
            //printf("gerakan 3 \n");
        }
        else if(gerakan==4){
            //gerakan mendarat
            valueTheta[0]=30;
            valueTheta[1]=60;
            valueTheta[2]=30;
            valueTheta[3]=10;

            kinematics.Angle_Jumping(valueTheta[0],valueTheta[1],valueTheta[2],valueTheta[3],20,jumlahStep);

            mimo.setPoint[1]=kinematics.AngleJointAll[1];
            mimo.setPoint[2]=kinematics.AngleJointAll[2];
            mimo.setPoint[3]=kinematics.AngleJointAll[3];
            mimo.setPoint[4]=kinematics.AngleJointAll[4];
            mimo.setPoint[5]=kinematics.AngleJointAll[5];
            mimo.setPoint[6]=kinematics.AngleJointAll[6];
            mimo.setPoint[7]=kinematics.AngleJointAll[7];
            mimo.setPoint[8]=kinematics.AngleJointAll[8];
            mimo.setPoint[14]=kinematics.AngleJointAll[14];
            //usleep(2000);
            if(kinematics.step>jumlahStep){
                gerakan=8;
                kinematics.initialPosition(valueTheta[0],valueTheta[1],valueTheta[2],valueTheta[3]);
                kinematics.resetKinematics(); 
                for(int i=1; i<=20; i++) motors.setspeed(i,0.3);
                sleep(2);
            }

            mimo.mimoPID(1, 3, 0,1.8,0,100, 0,2.3,0,100); //MIMO Engkel Kanan
            mimo.mimoPID(2, 4, 0,1.8,0,100, 0,2.3,0,100); //MIMO Engkel Kiri
            mimo.singlePID(5, 0, 1.8, 0, 100); //single PID Lutut kanan
            mimo.singlePID(6, 0, 1.8, 0, 100); //single PID Lutut kiri
            mimo.mimo3x3(7, 9, 11, 0,2.21,0,100, 0,3.01,0,100, 0,4.10,0,100); //MIMO Pinggul Kanan
            mimo.mimo3x3(8, 10, 12, 0,2.21,0,100, 0,3.372,0,100, 0,3.31,0,100); //MIMO Pinggul Kiri
            mimo.singlePID(14, 0, 2, 0, 100); //single PID perut
            //printf("gerakan 4 \n");
        }
        else if(gerakan==5){
            //gerakan kembali ke posisi initial posture

            valueTheta[0]=40;
            valueTheta[1]=50;
            valueTheta[2]=40;
            valueTheta[3]=-20;

            kinematics.Angle_Jumping(valueTheta[0],valueTheta[1],valueTheta[2],valueTheta[3],5,20);

            mimo.setPoint[1]=kinematics.AngleJointAll[1];
            mimo.setPoint[2]=kinematics.AngleJointAll[2];
            mimo.setPoint[3]=kinematics.AngleJointAll[3];
            mimo.setPoint[4]=kinematics.AngleJointAll[4];
            mimo.setPoint[5]=kinematics.AngleJointAll[5];
            mimo.setPoint[6]=kinematics.AngleJointAll[6];
            mimo.setPoint[7]=kinematics.AngleJointAll[7];
            mimo.setPoint[8]=kinematics.AngleJointAll[8];
            mimo.setPoint[14]=kinematics.AngleJointAll[14];

            /*mimo.data.outMimo[1]=kinematics.AngleJointAll[1];
            mimo.data.outMimo[2]=kinematics.AngleJointAll[2];
            mimo.data.outMimo[3]=kinematics.AngleJointAll[3];
            mimo.data.outMimo[4]=kinematics.AngleJointAll[4];
            mimo.data.outMimo[5]=kinematics.AngleJointAll[5];
            mimo.data.outMimo[6]=kinematics.AngleJointAll[6];
            mimo.data.outMimo[7]=kinematics.AngleJointAll[7];
            mimo.data.outMimo[8]=kinematics.AngleJointAll[8];*/
            //mimo.data.outMimo[14]=kinematics.AngleJointAll[14];

            //usleep(2000);
            if(kinematics.step>20){
                gerakan=6;
                kinematics.initialPosition(valueTheta[0],valueTheta[1],valueTheta[2],valueTheta[3]);
                kinematics.resetKinematics(); 
                flag=0;
            }

            /*if(flag==0){
                mimo.IError[3]=-30; mimo.IError[4]=30;
                mimo.IError[5]=-60; mimo.IError[6]=60;
                //mimo.IError[7]=30; mimo.IError[8]=-30;
                flag=1;
            }*/


            //mimo.mimoPID(1, 3, 0,0.8,0,100, 0,0.8,0,100); //MIMO Engkel Kanan
            //mimo.mimoPID(2, 4, 0,0.8,0,100, 0,0.8,0,100); //MIMO Engkel Kiri
            //mimo.singlePID(5, 0, 0.1, 0, 100); //single PID Lutut kanan
            //mimo.singlePID(6, 0, 0.1, 0, 100); //single PID Lutut kiri
            //mimo.mimo3x3(7, 9, 11, 0,1.21,0,100, 0,2.01,0,100, 0,3.10,0,100); //MIMO Pinggul Kanan
            //mimo.mimo3x3(8, 10, 12, 0,1.21,0,100, 0,2.372,0,100, 0,2.31,0,100); //MIMO Pinggul Kiri
            //mimo.singlePID(14, 0, 2, 0, 100); //single PID perut
            //printf("gerakan 5 \n");
        }
    
        //monitoring singlePID
        //printf("SP[%d]= %.2f pos= %.2f \n",id_servo,mimo.setPoint[id_servo],mimo.data.pos[id_servo]);

        //monitoring mimoPID2x2
        //printf("SP[%d]= %.2f pos[%d]= %.2f SP[%d]= %.2f pos[%d]= %.2f  \n",id_servo,mimo.setPoint[id_servo],id_servo,mimo.data.pos[id_servo], id_servo2,mimo.setPoint[id_servo2],id_servo2,mimo.data.pos[id_servo2]);

        //monitoring jumping
        printf("SP[%d]= %.2f pos[%d]= %.2f , SP[%d]= %.2f pos[%d]= %.2f , SP[%d]= %.2f pos[%d]= %.2f \n",id_servo,mimo.setPoint[id_servo],id_servo,mimo.data.pos[id_servo], id_servo2,mimo.setPoint[id_servo2],id_servo2,mimo.data.pos[id_servo2], id_servo3,mimo.setPoint[id_servo3],id_servo3,mimo.data.pos[id_servo3]);

        //checking PID controller
        //printf("SP[%d]= %.2f pos= %.2f e=%.2f i=%.2f d=%.2f %.2f\n",id_servo,mimo.setPoint[id_servo],mimo.data.pos[id_servo], mimo.Error[id_servo], mimo.IError[id_servo], mimo.DError[id_servo], mimo.data.outMimo[id_servo]);
        

        //Trajectory Walking
        //motion.StepTrajectoryWalking(10,motors);

        //coba jumping
        //motion.StepJumping(motors);

        ctr++;
        mimo.publishPosition();
        chatter_pub.publish(SP);

        //motion.motion_publish();
        ros::spinOnce();
        loop_rate.sleep();
    }
}
