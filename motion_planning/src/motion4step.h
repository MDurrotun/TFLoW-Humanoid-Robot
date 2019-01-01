#ifndef MOTION_H
#define MOTION_H

#include "loadparameter.h"
#include "definition.h"
#include "globalvars.h"
#include "dynamixel.h"
#include "kinematics.h"
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/UInt32.h>

class Motion4step{
    private :
        Parameter parameter;
        ros::NodeHandle node[4];
        ros::Publisher pub[4];
        unsigned char step;
        int Sx, Sy, Sz, Sh, Sxg, Syg, Szg, Seh, aBody, t;        
        float xFoot1, yFoot1, zFoot1, xFoot2, yFoot2, zFoot2, hFoot1, hFoot2;
	      float yRight2=200, yLeft2=200, flag1=0, flag2=0;
        int motions;
        int t_action[20];
        int PatternMotion[8][8] = {
            {0, 200, 0, 0, 0, 200, 0, 0},
            {0, 200, 0, 0, 0, 200, 0, 0},
            {0, 200, 0, 0, 0, 200, 0, 0},
            {0, 200, 0, 0, 0, 200, 0, 0},
            {0, 200, 0, 0, 0, 200, 0, 0},
            {0, 200, 0, 0, 0, 200, 0, 0},
            {0, 200, 0, 0, 0, 200, 0, 0},
            {0, 200, 0, 0, 0, 200, 0, 0},
        };
        short int DefaultStepTrajectoryWalking[8][8] = {
            {0, 200, 0, 0, 0, 200, 0, 0},
            {0, 200, 0, 0, 0, 200, 0, 0},
            {0, 200, 0, 0, 0, 200, 0, 0},
            {0, 200, 0, 0, 0, 200, 0, 0},
            {0, 200, 0, 0, 0, 200, 0, 0},
            {0, 200, 0, 0, 0, 200, 0, 0},
            {0, 200, 0, 0, 0, 200, 0, 0},
            {0, 200, 0, 0, 0, 200, 0, 0},
        };
        float sMotion[34][20];
        float AnglePosition[34];
        int max_step;

    public :
        Dynamixel motor;
        Kinematics kinematics;
        Motion4step();
        void ChangeDataMotion(int fMth);
        void MotionCounter(int fMth);
        void StepTrajectoryWalking(int fMth, Dynamixel& motor);
        void BasicMotionFSM(Dynamixel& motor);
        void ChangeDataAction(int fMth);
        void StepTrajectoryAction(int fMth, Dynamixel& motor);
        void motion_publish();
	      void StepJumping(Dynamixel& motor);
        void StepStartUp(Dynamixel& motor);
};

Motion4step::Motion4step()
{
    Motion4step::pub[0] = Motion4step::node[0].advertise<geometry_msgs::Quaternion>("/motion/koordinat/kaki_kanan",1);
    Motion4step::pub[1] = Motion4step::node[1].advertise<geometry_msgs::Quaternion>("/motion/koordinat/kaki_kiri",1);
    Motion4step::pub[2] = Motion4step::node[2].advertise<std_msgs::UInt32>("/motion/step",1);
    Motion4step::pub[3] = Motion4step::node[3].advertise<std_msgs::UInt32>("/motion/motion",1);
}

void Motion4step::motion_publish()
{
    geometry_msgs::Quaternion koordinat_kaki_kanan_p;
    geometry_msgs::Quaternion koordinat_kaki_kiri_p;
    std_msgs::UInt32 step_p;
    std_msgs::UInt32 motion_p;

    koordinat_kaki_kanan_p.x = zFoot1;
    koordinat_kaki_kanan_p.y = xFoot1;
    koordinat_kaki_kanan_p.z = yFoot1;
    koordinat_kaki_kanan_p.w = hFoot1;

    koordinat_kaki_kiri_p.x = zFoot2;
    koordinat_kaki_kiri_p.y = xFoot2;
    koordinat_kaki_kiri_p.z = yFoot2;
    koordinat_kaki_kiri_p.w = hFoot2;

    step_p.data = step;
    motion_p.data = motions;

    Motion4step::pub[0].publish(koordinat_kaki_kanan_p);
    Motion4step::pub[1].publish(koordinat_kaki_kiri_p);
    Motion4step::pub[2].publish(step_p);
    Motion4step::pub[3].publish(motion_p);
}

void Motion4step::ChangeDataMotion(int fMth){

    parameter.getparam(fMth,Sx,Sy,Sz,Sh,Sxg,Syg,Szg,Seh,aBody,t);
    //ROS_INFO("SX: %d, SY: %d, SZ: %d", Sx, Sy, Sz);

    for(j=0 ; j<8 ; j++){
        for(i=0 ; i<5 ; i++){
            PatternMotion[i][j] = DefaultStepTrajectoryWalking[i][j];
        }
    }

    PatternMotion[2][0] = PatternMotion[4][4] = Sx + Sxg;
    PatternMotion[2][4] = PatternMotion[4][0] = -Sx + Sxg;

    PatternMotion[1][1] = PatternMotion[3][5] =  Sy;

    PatternMotion[1][2] = PatternMotion[1][6] = -Sz;
    PatternMotion[3][2] = PatternMotion[3][6] = Sz;

    if(Seh <= 0){
        PatternMotion[1][3] = PatternMotion[2][3] = PatternMotion[3][7] = PatternMotion[4][7] = -Sh - Seh;
        PatternMotion[1][7] = PatternMotion[2][7] = PatternMotion[3][3] = PatternMotion[4][3] = Sh;
    } else {
        PatternMotion[1][3] = PatternMotion[2][3] = PatternMotion[3][7] = PatternMotion[4][7] = -Sh;
        PatternMotion[1][7] = PatternMotion[2][7] = PatternMotion[3][3] = PatternMotion[4][3] = Sh + Seh;
    }

    if(Szg < 0){
        PatternMotion[4][2] = Szg - 3;
        PatternMotion[4][6] = -Szg;
    } else if(Szg > 0){
        PatternMotion[2][2] = -Szg;
        PatternMotion[2][6] = Szg + 3;
    }
}

//MOTION COUNTER
void Motion4step::MotionCounter(int fMth){
  //________________________________Acceleration___________________________//
  if(fMth==50){
    if(counter<4)fMth=10;
    else if(counter>=4 && counter<8)fMth=20;
    else if(counter>=8 && counter<12)fMth=30;
    else if(counter>=12 && counter<16)fMth=40;
  }
  else if(fMth==51){
    if(counter<4)fMth=11;
    else if(counter>=4 && counter<8)fMth=21;
    else if(counter>=8 && counter<12)fMth=31;
    else if(counter>=12 && counter<16)fMth=41;
  }
  else if(fMth==52){
    if(counter<4)fMth=12;
    else if(counter>=4 && counter<8)fMth=22;
    else if(counter>=8 && counter<12)fMth=32;
    else if(counter>=12 && counter<16)fMth=42;
  }
  else if(fMth==40){
    if(counter<4)fMth=10;
    else if(counter>=4 && counter<8)fMth=20;
    else if(counter>=8 && counter<12)fMth=30;
  }
  else if(fMth==41){
    if(counter<4)fMth=11;
    else if(counter>=4 && counter<8)fMth=21;
    else if(counter>=8 && counter<12)fMth=31;
  }
  else if(fMth==42){
    if(counter<4)fMth=12;
    else if(counter>=4 && counter<8)fMth=22;
    else if(counter>=8 && counter<12)fMth=32;
  }
  else if(fMth>=30 && fMth<40){
    if(counter<4)fMth=10;
    else if(counter>=4 && counter<8)fMth=20;
  }
  else if(fMth>=20 && fMth<30){
    if(counter<4)fMth=10;
  }
  //________________________________DeAcceleration___________________________//
  if(fMth>=30 && fMth<40){
    if(dcount<4)fMth=40;
  }
  else if(fMth>=20 && fMth<30){
      if(dcount<4)fMth=40;
      else if(dcount>=4 && dcount<8)fMth=30;
  }
  else if(fMth>200 || (fMth>=10 && fMth<19)){
      if(fMth == 11){
          if(dcount<4)fMth=41;
          else if(dcount>=4 && dcount<8)fMth=31;
          else if(dcount>=8 && dcount<12)fMth=21;
      }else if(fMth == 12){
          if(dcount<4)fMth=42;
          else if(dcount>=4 && dcount<8)fMth=32;
          else if(dcount>=8 && dcount<12)fMth=22;
      }else{
          if(dcount<4)fMth=40;
          else if(dcount>=4 && dcount<8)fMth=30;
          else if(dcount>=8 && dcount<12)fMth=20;
      }
  }
  else if(fMth == 19 || fMth < 10){
    if(dcount<4)fMth=40;
    else if(dcount>=4 && dcount<8)fMth=30;
    else if(dcount>=8 && dcount<12)fMth=20;
    else if(dcount>=12 && dcount<16)fMth=10;
  }

  //if(fMth!=lastfMth){
    ChangeDataMotion(fMth);
    //lastfMth=fMth;
  //}

  if(fMth==19 || fMth<=9){counter=0;if(dcount>=18)dcount=18;}
  else if(fMth>=10 && fMth<19){if(counter>=4)counter=4;if(dcount>=16)dcount=16;}
  else if(fMth>=20 && fMth<25){if(counter>=8)counter=8;if(dcount>=12)dcount=12;}
  else if(fMth>=30 && fMth<40){if(counter>=12)counter=12;if(dcount>=8)dcount=8;}
  else if(fMth>=40 && fMth<50){if(counter>=16)counter=16;if(dcount>=4)dcount=4;}
  else if(fMth>=50 && fMth<60){if(counter>=18)counter=18; dcount=0;}

  if(counter >= 18) counter = 18;
  if(dcount >= 18)dcount = 18;
  counter++;dcount++;
}

//STEP TRAJECTORY WALKING
void Motion4step::StepTrajectoryWalking(int fMth, Dynamixel& motor){

  motions = fMth;
  //t = 6;
  if(step == 0) t = 1;

  if(theta > 180){
    for(i=0; i<8; i++){ lastPatternMotion[i] = PatternMotion[step][i]; } //take motion pattern matrix
    lastAngleBody = aBody;
    step ++;
    if(step > 4){
      step = 1;
      CountSTART++;
      if(CountSTART>=100) CountSTART=100;
    } //back to step 1
    if(step==1||step==3){ MotionCounter(fMth); } // motion acc & decc
    theta = t;
    if(fMth == 210 && step == 4){ //sit
      state = RELAX;
    }
  }

  LengthX1 = (float)(PatternMotion[step][0] - lastPatternMotion[0]);
  LengthY1 = (float)(PatternMotion[step][1] - lastPatternMotion[1]);
  LengthZ1 = (float)(PatternMotion[step][2] - lastPatternMotion[2]);
  Heading1 = (float)(PatternMotion[step][3] - lastPatternMotion[3]);
  LengthX2 = (float)(PatternMotion[step][4] - lastPatternMotion[4]);
  LengthY2 = (float)(PatternMotion[step][5] - lastPatternMotion[5]);
  LengthZ2 = (float)(PatternMotion[step][6] - lastPatternMotion[6]);
  Heading2 = (float)(PatternMotion[step][7] - lastPatternMotion[7]);

  if(step == 1 || step ==3){
    yFoot1 = sin((theta/2) / PHI) * LengthY1 + (float)lastPatternMotion[1];
    yFoot2 = sin((theta/2) / PHI) * LengthY2 + (float)lastPatternMotion[5];
    zFoot2 = sin((theta/2) / PHI) * LengthZ2 + (float)lastPatternMotion[6];
    zFoot1 = sin((theta/2) / PHI) * LengthZ1 + (float)lastPatternMotion[2];
  }else{
    yFoot1 = (1 - sin(((theta/2)+90) / PHI)) * LengthY1 + (float)lastPatternMotion[1];
    yFoot2 = (1 - sin(((theta/2)+90) / PHI)) * LengthY2 + (float)lastPatternMotion[5];
    zFoot2 = (1 - sin(((theta/2)+90) / PHI)) * LengthZ2 + (float)lastPatternMotion[6];
    zFoot1 = (1 - sin(((theta/2)+90) / PHI)) * LengthZ1 + (float)lastPatternMotion[2];
  }
  xFoot1 = ((0 - cos(theta / PHI)+ 1)/2) * LengthX1 + (float)lastPatternMotion[0];
  xFoot2 = ((0 - cos(theta / PHI)+ 1)/2) * LengthX2 + (float)lastPatternMotion[4];
  hFoot2 = (theta / 180) * Heading2 + (float)lastPatternMotion[7];
  hFoot1 = (theta / 180) * Heading1 + (float)lastPatternMotion[3];

  if (step == 2 || step == 3 || step == 4){
    HipH = (((0 - cos(theta / PHI)+ 1)/2) * LengthX1 + (float)lastPatternMotion[0])/4;
  } else{
    HipH = -(((0 - cos(theta / PHI)+ 1)/2) * LengthX2 + (float)lastPatternMotion[4])/4;
  }
  HipZ = (zFoot1+zFoot2)/-2;
  AngleBody = (theta / 180) * (aBody - lastAngleBody) + lastAngleBody;

  kinematics.Coordinate_Kaki(xFoot1, zFoot1, yFoot1, hFoot1, xFoot2, zFoot2, yFoot2, hFoot2);
  //ROS_INFO("Kanan Y : %g | Kiri Y : %g || Kanan Z : %g | Kiri Z : %g", yFoot1,yFoot2,zFoot1,zFoot2);

  //Coba Tambahan
  if(yFoot1>=200){
    kinematics.AngleJointAll[13]=(kinematics.AngleJointAll[1]*0.5);
  }
  else if(yFoot2>=200){
    kinematics.AngleJointAll[13]=(kinematics.AngleJointAll[2]*0.5);
  }

  if(step == 1 || step == 3){
    theta+=t;
  } else {
      theta+=t;
    //if(theta > 160 && theta <= 175) theta+=(5);
    //else if(theta > 175 && theta <= 180) theta+=(2);
    //else theta+=t;
  }

  //motor.moveMotor_degAll(kinematics.AngleJointAll,1,20);
}


void Motion4step::BasicMotionFSM(Dynamixel& motor){
  if(state==RELAX){
    //HeadManual=ON;
  }else if(state==STANCE){
    //FallDetect();
  }else if(state==WALK){
    //parsingData();
    if(motion <= 9 && motion > 0){
      Action = motion;
      motion = 0;

    } else {
      Action = motion;
    }
    StepTrajectoryWalking(motion,motor);
    sleep(8);
    //collectDXLkaki();
    //sprintf(kata,"%d	%d	%d	%d	%d	%d	%d	%d	%d	%d	%d	%d\n",dataPosDXL[1],dataPosDXL[2],dataPosDXL[3],dataPosDXL[4],dataPosDXL[5],dataPosDXL[6],dataPosDXL[7],dataPosDXL[8],dataPosDXL[9],dataPosDXL[10],dataPosDXL[11],dataPosDXL[12]);
    //USART_puts1(kata);
    //FallDetect();
  }else if(state==SIT){
  }else if(state==KICK){
    //StepTrajectoryKicking(KickMode);
  }else if(state==FALL){
    //Falling();
  }

}

void Motion4step::ChangeDataAction(int fMth)
{
    parameter.getparam_action(fMth,sMotion,max_step,t_action);
}

void Motion4step::StepTrajectoryAction(int fMth, Dynamixel& motor)
{
    motions = fMth;
    if(fMth!=lastfMth){
      ChangeDataAction(fMth);
      //m=0;
    }   
    for(step=m;step<max_step;step++){
      T=0;
      t = t_action[step];
      if(t == 0) t = 1;
      //ROS_INFO("Time Step %d : %d -> t : %d", step, t_action[step], t);
      while(T<=180)
      {
        for(i=0;i<=id_max;i++){
          Selisih=sMotion[i][step]-sMotion[i][step-1];
          AnglePosition[i]=(int)((((float)VCos[T]/1000)*Selisih)/2+sMotion[i][step-1]);
        }
        ROS_INFO("%g %g",AnglePosition[18],AnglePosition[19]);
        motor.moveMotor_degAll(AnglePosition,1,20);
        T+=t;
        //ROS_INFO("t = %d",t);
        //ROS_INFO("T = %d",T);
        //ROS_INFO("fmth = %d",fMth);
        //ROS_INFO("STEP = %d",step);
        //ROS_INFO("MAX_STEP = %d", max_step);
        usleep(13000);
      }
      m=1;
      /*

      if(fMth==100){//Give Flag
        if(step==1){t=4;}
        else if(step==2){t=4;usleep(500000);}
        else if(step==3){t=4;usleep(100000);}
        else if(step==4){t=2;usleep(30000000);}
        else if(step==5){t=4;usleep(100000);}
      } else if(fMth==101){//Give Flag
        if(step==1){t=1;}
        else if(step==2){t=1;}
      } else if(fMth==102){//Give Flag
        if(step==1){t=1;}
        else if(step==9){t=1;}
      }*/
    }
    l=max_step;
    //if(max_step==1||max_step==5){m=2;max_step=3;}
    //else if(max_step==3){max_step=4;max_step=5;}
    lastfMth=fMth;
}

void Motion4step::StepJumping(Dynamixel& motor){
  /*xFoot1 = 0;  xFoot2 = 0;
  zFoot1 = 0;  zFoot2 = 0;
  hFoot1 = 0;  hFoot2 = 0;

  for(float i=20; i<=195; i+=0.8){
    yFoot1 = i;
    yFoot2 = i;
    kinematics.Coordinate_Kaki(xFoot1, zFoot1, yFoot1, hFoot1, xFoot2, zFoot2, yFoot2, hFoot2);
    printf("Kanan Y : %.2f | Kiri Y : %.2f\n", yFoot1,yFoot2);

    usleep(10000);
    motor.moveMotor_degAll(kinematics.AngleJointAll,1,20);
  }

  usleep(5000000);

  for(float i=195; i>=20; i-=0.8){
    yFoot1 = i;
    yFoot2 = i;
    kinematics.Coordinate_Kaki(xFoot1, zFoot1, yFoot1, hFoot1, xFoot2, zFoot2, yFoot2, hFoot2);
    printf("Kanan Y : %.2f | Kiri Y : %.2f\n", yFoot1,yFoot2);

    usleep(10000);
    motor.moveMotor_degAll(kinematics.AngleJointAll,1,20);
  }
  usleep(5000000);*/

  //step jongkok
  motor.moveMotor_deg(3, -60); 
  motor.moveMotor_deg(4, 60);

  motor.moveMotor_deg(5, -100); 
  motor.moveMotor_deg(6, 100);

  motor.moveMotor_deg(7, 60); 
  motor.moveMotor_deg(8, -60);

  //motor.moveMotor_deg(14, -10);
  usleep(4000000);

  //step lompat
  motor.moveMotor_deg(3, 20); 
  motor.moveMotor_deg(4, -20);

  motor.moveMotor_deg(5, 0); 
  motor.moveMotor_deg(6, 0);

  motor.moveMotor_deg(7, 0); 
  motor.moveMotor_deg(8, 0);

  //motor.moveMotor_deg(14, 0);
  usleep(1000000);

  //step mendarat
  motor.moveMotor_deg(3, -20); 
  motor.moveMotor_deg(4, 20);

  motor.moveMotor_deg(5, -50); 
  motor.moveMotor_deg(6, 50);

  motor.moveMotor_deg(7, 30); 
  motor.moveMotor_deg(8, -30);

  //motor.moveMotor_deg(14, -5);
  usleep(4000000);

  //step kembali ke posisi semula
  motor.moveMotor_deg(3, 0); 
  motor.moveMotor_deg(4, 0);

  motor.moveMotor_deg(5, 0); 
  motor.moveMotor_deg(6, 0);

  motor.moveMotor_deg(7, 0); 
  motor.moveMotor_deg(8, 0);

  //motor.moveMotor_deg(14, 0);
  usleep(4000000);

  while(1);
  
}

void Motion4step::StepStartUp(Dynamixel& motor){
  xFoot1 = 0;  xFoot2 = 0;
  zFoot1 = 0;  zFoot2 = 0;
  hFoot1 = 0;  hFoot2 = 0;

  for(int i=1; i<=20; i++)
  {
        motor.setspeed(i,1);
        usleep(500);
  }

  for(float i=0; i<=10; i+=0.8)
  {
    yFoot1 = i;
    yFoot2 = i;
    kinematics.Coordinate_Kaki(xFoot1, zFoot1, yFoot1, hFoot1, xFoot2, zFoot2, yFoot2, hFoot2);

    usleep(100000);
    motor.moveMotor_degAll(kinematics.AngleJointAll,1,20);
  }

  //change hand to forward position
  for(int i=1; i<=10; i++)
  {
  motor.moveMotor_deg(15,30);
  motor.moveMotor_deg(16,-30);
  usleep(300000);
  }
  

  //change speed motor to high speed
  for(int i=1; i<=20; i++)
  {
        motor.setspeed(i,2);
        usleep(500);
  }

  for(float i=0; i<=195; i+=0.8)
  {
    yFoot1 = i;
    yFoot2 = i;
    kinematics.Coordinate_Kaki(xFoot1, zFoot1, yFoot1, hFoot1, xFoot2, zFoot2, yFoot2, hFoot2);
    printf("Kanan Y : %.2f | Kiri Y : %.2f\n", yFoot1,yFoot2);

    usleep(10000);
    motor.moveMotor_degAll(kinematics.AngleJointAll,1,20);
  }

}



#endif // MOTION_H
