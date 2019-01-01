#ifndef CONTROL_H
#define CONTROL_H

/* Developed by Durrotun Nashihin - ER2C PENS */

#include <ros/ros.h>
#include "dynamixel.h"
#include <sensor_msgs/JointState.h>
#include "std_msgs/Float32.h"
#include <iostream>
#include <sstream>
#include "motion_planning/displayData.h"
#include "loadparameter.h"
#include "definition.h"
#include "globalvars.h"
#include "kinematics.h"
#include "math.h"

#define degtorad 0.01745329252


class control{
    private :
    ros::Publisher pub;
    ros::NodeHandle n; 
    float PID1,PID2;
    float p11,p12,p21,p22;
    float tf11,tf12,tf13,tf21,tf22,tf23,tf31,tf32,tf33;
    float total=0, average=0;
    float readings[20];
    unsigned char readIndex = 0;
    float dataLPF=0;

    public :
    Dynamixel motors;
    motion_planning::displayData data;


    float Error[21]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    float IError[21]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    float DError[21]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    float lastPos[21]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    float LastError[21]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    float setPoint[21]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    float filterCoefficient[21]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    float filter_DSTATE[21]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    float TS;

    void resetData();
    void initPublishPosition();
    void publishPosition();
    void update_data(int firstID, int lastID);
    void singlePID(int id, float kp, float ki, float kd, float N);
    void mimoPID(int id, int id2, float kp,float ki,float kd,float N, float kp2,float ki2,float kd2,float N2);
    void mimo3x3(int id, int id2, int id3, float kp,float ki,float kd,float N, float kp2,float ki2,float kd2,float N2, float kp3,float ki3,float kd3,float N3);
    float filter(float data);
};

void control::resetData(){
    for(int i=0; i<=20; i++){
        data.outMimo[i]=0;
        data.pos[i]=0;
        IError[i]=0;
    }
}

void control::initPublishPosition(){
    pub = n.advertise<motion_planning::displayData>("displayData", 1000); //, this
}

void control::publishPosition(){
    pub.publish(data);
}

void control::update_data(int firstID, int lastID){
    //position
    for(int i=firstID; i<=lastID; i++) {
        data.pos[i]=motors.current_pos[i];
        if(data.pos[i]>180)data.pos[i]=0;
        if(data.pos[i]<-180)data.pos[i]=0;
    }

    //load
    //for(int i=firstID; i<=lastID; i++) data.loadServo[i]=motors.load[i];

    //data.selisih = abs(motors.current_pos[3]) - abs(motors.current_pos[4]);
    //printf("selisih = %.2f", data.selisih);
}


//Control = SP+PID
/*void control::singlePID(int id, float kp, float ki, float kd, float N){
    lastPos[id] = data.pos[id];
  
    Error[id]  = setPoint[id] - data.pos[id];
    IError[id] = IError[id] + (Error[id]*0.02);
    //DError[id] = (Error[id] - LastError[id])/0.02;
    filterCoefficient[id] = (kd*Error[id] - filter_DSTATE[id]) * N;
    LastError[id] = Error[id];
   
    //data.outMimo[id] = setPoint[id] - (kp*Error[id] + ki*IError[id] + kd*DError[id]);
    data.outMimo[id] = setPoint[id] + (kp*Error[id] + ki*IError[id] + filterCoefficient[id]);
    filter_DSTATE[id] = filter_DSTATE[id] + (filterCoefficient[id]*0.02); 
    //data.outMimo[id] = setPoint[id] + (kp*Error[id] + ki*IError[id] + kd*DError[id]);
  
    //ID, data
    //motors.moveMotor_deg(id, data.outMimo[id]);
}*/

//control = PID
void control::singlePID(int id, float kp, float ki, float kd, float N){
    lastPos[id] = data.pos[id];
  
    Error[id]  = setPoint[id]-data.pos[id];
    IError[id] = IError[id] + (Error[id]*0.02);
    //DError[id] = (Error[id] - LastError[id])/0.02;
    filterCoefficient[id] = (kd*Error[id] - filter_DSTATE[id]) * N;

    LastError[id] = Error[id];
   
    //data.outMimo[id] = setPoint[id] - (kp*Error[id] + ki*IError[id] + kd*DError[id]);
    data.outMimo[id] = (kp*Error[id] + ki*IError[id] + filterCoefficient[id]);
    filter_DSTATE[id] = filter_DSTATE[id] + (filterCoefficient[id]*0.02); 
    //data.outMimo[id] = (kp*Error[id] + ki*IError[id] + kd*DError[id]);
  
    //ID, data
    //motors.moveMotor_deg(id, data.outMimo[id]);
}

//control = PID Kinematics
/*void control::singlePID(float x, float y, float z, float Nx, float Ny, float Nz){

    //Forward Kinematics

    Errorx  = x-dataForwardx;
    IErrorx = IErrorx + (Errorx*0.02);
    //DError[id] = (Error[id] - LastError[id])/0.02;
    filterCoefficientx = (kd*Errorx - filter_DSTATEx * Nx;
    filter_DSTATEx = filter_DSTATEx + (filterCoefficientx*0.02); 

    Errory  = y-dataForwardy;
    IErrory = IErrory + (Errory*0.02);
    //DError[id] = (Error[id] - LastError[id])/0.02;
    filterCoefficienty = (kd*Errory - filter_DSTATEy * Ny;
    filter_DSTATEz = filter_DSTATEz + (filterCoefficientz*0.02);

    Errorz  = z-dataForwardz;
    IErrorz = IErrorz + (Errorz*0.02);
    //DError[id] = (Error[id] - LastError[id])/0.02;
    filterCoefficientz = (kd*Errorz - filter_DSTATEz * Nz;
    filter_DSTATEz = filter_DSTATEz + (filterCoefficientz*0.02);


    //outx,outy,outz
    
    //Invers Kinematics
    //x,y,z to servo's angle    
  
}*/

//Mimo control = SP + PID
/*void control::mimoPID(int id, int id2, float kp,float ki,float kd,float N, float kp2,float ki2,float kd2,float N2){
    
    if(data.pos[id]>9999||data.pos[id]<-9999)data.pos[id]=0;
    if(data.pos[id2]>9999||data.pos[id2]<-9999)data.pos[id2]=0;

    //if(lastPos[id]-data.pos[id]<-80||lastPos[id]-data.pos[id]>80)data.pos[id]=lastPos[id];
    //if(lastPos[id2]-data.pos[id2]<-80||lastPos[id2]-data.pos[id2]>80)data.pos[id2]=lastPos[id2];

    //lastPos[id] = data.pos[id];
    //lastPos[id2] = data.pos[id2];
    //data.pos[id] = filter(data.pos[id]);
    //data.pos[id2] = filter(data.pos[id2]);

    Error[id]  = setPoint[id] - data.pos[id];
    IError[id] = IError[id] + (Error[id]*0.02);
    //DError[id] = (Error[id] - LastError[id])/0.02;
    filterCoefficient[id] = (kd*Error[id] - filter_DSTATE[id]) * N;
    LastError[id] = Error[id];

    Error[id2]  = setPoint[id2] - data.pos[id2];
    IError[id2] = IError[id2] + (Error[id2]*0.02);
    //DError[id2] = (Error[id2] - LastError[id2])/0.02;
    filterCoefficient[id2] = (kd*Error[id2] - filter_DSTATE[id2]) * N2;
    LastError[id2] = Error[id2];

    PID1 = (kp*Error[id] + ki*IError[id] + filterCoefficient[id]);
    PID2 = (kp*Error[id2] + ki*IError[id2] + filterCoefficient[id2]);

    data.outMimo[id]  = setPoint[id] + PID1 - PID2;
    data.outMimo[id2] = setPoint[id2]+ PID2 - PID1; 
   

    filter_DSTATE[id] = filter_DSTATE[id] + (filterCoefficient[id]*0.02);
    filter_DSTATE[id2] = filter_DSTATE[id2] + (filterCoefficient[id2]*0.02);

    //data.outMimo[id] = setPoint[id] + (kp*Error[id] + ki*IError[id] + filterCoefficient[id])
		       - (kp*Error[id2] + ki*IError[id2] + filterCoefficient[id2]);
    //filter_DSTATE[id] = filter_DSTATE[id] + (filterCoefficient[id]*0.02); 

    //data.outMimo[id2] = setPoint[id2] + (kp*Error[id2] + ki*IError[id2] + filterCoefficient[id2])
		       - (kp*Error[id] + ki*IError[id] + filterCoefficient[id]);
    //filter_DSTATE[id2] = filter_DSTATE[id2] + (filterCoefficient[id2]*0.02); 
   
    //data.outMimo[id] = setPoint[id] - (kp*Error[id] + ki*IError[id] + kd*DError[id]) 
    //	                 - (kp*Error[id2] + ki*IError[id2] + kd*DError[id2]);
    //data.outMimo[id2] = setPoint[id2] - (kp*Error[id] + ki*IError[id] + kd*DError[id]) 
    //                   - (kp*Error[id2] + ki*IError[id2] + kd*DError[id2]);
  
    //ID, data
    //motors.moveMotor_deg(id, data.outMimo[id]);
    //motors.moveMotor_deg(id2, data.outMimo[id2]);
}*/

//Mimo control = PID
void control::mimoPID(int id, int id2, float kp,float ki,float kd,float N, float kp2,float ki2,float kd2,float N2){
    
    if(data.pos[id]>9999||data.pos[id]<-9999)data.pos[id]=0;
    if(data.pos[id2]>9999||data.pos[id2]<-9999)data.pos[id2]=0;

    if(lastPos[id]-data.pos[id]<-20||lastPos[id]-data.pos[id]>20)data.pos[id]=lastPos[id];
    if(lastPos[id2]-data.pos[id2]<-20||lastPos[id2]-data.pos[id2]>20)data.pos[id2]=lastPos[id2];

    lastPos[id] = data.pos[id];
    lastPos[id2] = data.pos[id2];

    Error[id]  = setPoint[id]-data.pos[id];
    IError[id] = IError[id] + (Error[id]*0.02);
    //DError[id] = (Error[id] - LastError[id])/0.02;
    filterCoefficient[id] = (kd*Error[id] - filter_DSTATE[id]) * N;
    LastError[id] = Error[id];

    Error[id2]  = setPoint[id2]-data.pos[id2];
    IError[id2] = IError[id2] + (Error[id2]*0.02);
    //DError[id2] = (Error[id2] - LastError[id2])/0.02;
    filterCoefficient[id2] = (kd*Error[id2] - filter_DSTATE[id2]) * N2;
    //LastError[id2] = Error[id2];

    p11 = (kp*Error[id] + ki*IError[id] + filterCoefficient[id]);
    p22 = (kp*Error[id2] + ki*IError[id2] + filterCoefficient[id2]);

    p12 = p22*0.08;
    p21 = p11*0.1;
    
    //engkle kanan
    //data.outMimo[id] = p11 - p12 ;
    //data.outMimo[id2] = p22 - p21;

    //engkel kiri
    data.outMimo[id] = p11 + p12 ;
    data.outMimo[id2] = p22 + p21;

    //data.outMimo[id] = setPoint[id];
    //data.outMimo[id2] = setPoint[id2];

    //data.outMimo[id] = setPoint[id];
    //data.outMimo[id2] = setPoint[id2];

    filter_DSTATE[id] = filter_DSTATE[id] + (filterCoefficient[id]*0.02); 
    filter_DSTATE[id2] = filter_DSTATE[id2] + (filterCoefficient[id2]*0.02);

    //printf("p12 = %.2f, p21 = %.2f, o1 = %.2f, o2 = %.2f\n", p21, p12, data.outMimo[id], data.outMimo[id2]);

   
    //data.outMimo[id] = setPoint[id] - data.outMimo[id];
    //data.outMimo[id2] = setPoint[id2] - data.outMimo[id2];
    //data.outMimo[id2] = setPoint[id2] - (kp*Error[id] + ki*IError[id] + kd*DError[id]) 
    //                   - (kp*Error[id2] + ki*IError[id2] + kd*DError[id2]);
  
    //ID, data
    //motors.moveMotor_deg(id, data.outMimo[id]);
    //motors.moveMotor_deg(id2, data.outMimo[id2]);
}

//Mimo control = PID
void control::mimo3x3(int id,int id2,int id3, float kp,float ki,float kd,float N, float kp2,float ki2,float kd2,float N2, float kp3,float ki3,float kd3,float N3){
    
    if(data.pos[id]>9999||data.pos[id]<-9999)data.pos[id]=0;
    if(data.pos[id2]>9999||data.pos[id2]<-9999)data.pos[id2]=0;
    if(data.pos[id3]>9999||data.pos[id3]<-9999)data.pos[id3]=0;

    if(lastPos[id]-data.pos[id]<-20||lastPos[id]-data.pos[id]>20)data.pos[id]=lastPos[id];
    if(lastPos[id2]-data.pos[id2]<-20||lastPos[id2]-data.pos[id2]>20)data.pos[id2]=lastPos[id2];
    if(lastPos[id3]-data.pos[id3]<-20||lastPos[id3]-data.pos[id3]>20)data.pos[id3]=lastPos[id3];

    lastPos[id] = data.pos[id];
    lastPos[id2] = data.pos[id2];
    lastPos[id3] = data.pos[id3];

    Error[id]  = setPoint[id]-data.pos[id];
    IError[id] = IError[id] + (Error[id]*0.02);
    //DError[id] = (Error[id] - LastError[id])/0.02;
    filterCoefficient[id] = (kd*Error[id] - filter_DSTATE[id]) * N;
    //LastError[id] = Error[id];

    Error[id2]  = setPoint[id2]-data.pos[id2];
    IError[id2] = IError[id2] + (Error[id2]*0.02);
    //DError[id2] = (Error[id2] - LastError[id2])/0.02;
    filterCoefficient[id2] = (kd*Error[id2] - filter_DSTATE[id2]) * N2;
    //LastError[id2] = Error[id2];

    Error[id3]  = setPoint[id3]-data.pos[id3];
    IError[id3] = IError[id3] + (Error[id3]*0.02);
    //DError[id2] = (Error[id2] - LastError[id2])/0.02;
    filterCoefficient[id3] = (kd*Error[id3] - filter_DSTATE[id3]) * N3;
    //LastError[id2] = Error[id2];


    tf11 = (kp*Error[id] + ki*IError[id] + filterCoefficient[id]);
    tf22 = (kp*Error[id2] + ki*IError[id2] + filterCoefficient[id2]);
    tf33 = (kp*Error[id3] + ki*IError[id3] + filterCoefficient[id3]);

    tf12 = tf22*0.1;
    tf13 = tf33*0.1;

    tf21 = tf11*0.1;
    tf23 = tf33*0.1;

    tf31 = tf11*0.1;
    tf32 = tf22*0.1;
    

    //engkel kiri
    data.outMimo[id] = tf11 + tf12 + tf13;
    data.outMimo[id2] = tf21 + tf22 + tf23;
    data.outMimo[id3] = tf31 + tf32 + tf33;

    filter_DSTATE[id] = filter_DSTATE[id] + (filterCoefficient[id]*0.02); 
    filter_DSTATE[id2] = filter_DSTATE[id2] + (filterCoefficient[id2]*0.02);
    filter_DSTATE[id3] = filter_DSTATE[id3] + (filterCoefficient[id3]*0.02);

    //printf("p12 = %.2f, p21 = %.2f, o1 = %.2f, o2 = %.2f\n", p21, p12, data.outMimo[id], data.outMimo[id2]);

   
    //data.outMimo[id] = setPoint[id] - (kp*Error[id] + ki*IError[id] + kd*DError[id]) 
    //	                 - (kp*Error[id2] + ki*IError[id2] + kd*DError[id2]);
    //data.outMimo[id2] = setPoint[id2] - (kp*Error[id] + ki*IError[id] + kd*DError[id]) 
    //                   - (kp*Error[id2] + ki*IError[id2] + kd*DError[id2]);
  
    //ID, data
    //motors.moveMotor_deg(id, data.outMimo[id]);
    //motors.moveMotor_deg(id2, data.outMimo[id2]);*/
}

float control::filter(float data){
  //float Alpha = 0.201;
  //dataLPF = data*Alpha + ((1-Alpha)*dataLPF);
  dataLPF = data*1024;

  total = total - readings[readIndex];
  // read from the sensor:
  readings[readIndex] = dataLPF;
  // add the reading to the total:
  total = total + readings[readIndex];
  // advance to the next position in the array:
  readIndex = readIndex + 1;

  // if we're at the end of the array...
  if (readIndex >= 10) {
    // ...wrap around to the beginning:
    readIndex = 0;
  }

  // calculate the average:
  average = total / 10;

  //average = dataLPF;
  return average;
}



#endif // CONTROL_H
