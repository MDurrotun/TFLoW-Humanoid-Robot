#ifndef MOTION_H
#define MOTION_H

#include "loadparameter.h"
#include "definition.h"
#include "globalvars.h"
#include "dynamixel.h"
#include "kinematics.h"
class Motion6step{
    private :
        Parameter parameter;
        Kinematics kinematics;
        unsigned char step;
        int Sx, Sy, Sz, Sh, Sxg, Syg, Szg, Seh, aBody, t;
        float xFoot1, yFoot1, zFoot1, xFoot2, yFoot2, zFoot2, hFoot1, hFoot2;
        int PatternMotion[8][8];
        short int DefaultStepTrajectoryWalking[8][8];

        //Dynamixel motor;

    public :
        void ChangeDataMotion(int fMth);
        void MotionCounter(int fMth);
        void StepTrajectoryWalking(int fMth, Dynamixel& motor);
        void BasicMotionFSM2(Dynamixel& motor);

};

void Motion6step::ChangeDataMotion(int fMth){

    parameter.getparam(fMth,Sx,Sy,Sz,Sh,Sxg,Syg,Szg,Seh,aBody,t);

    for(j=0 ; j<8 ; j++){
        for(i=0 ; i<7 ; i++){
            PatternMotion[i][j] = DefaultStepTrajectoryWalking[i][j];
        }
    }

    if(fMth != 0){
        PatternMotion[3][0] = PatternMotion[6][4] = Sx + Sxg;
        PatternMotion[3][4] = PatternMotion[6][0] = -Sx + Sxg;

        PatternMotion[4][0] = PatternMotion[1][4] = 0;
        PatternMotion[4][4] = PatternMotion[1][0] = PatternMotion[3][4] - PatternMotion[3][0];


        PatternMotion[2][1] = PatternMotion[5][5] =  Sy;
        PatternMotion[3][1] = PatternMotion[6][5] =  190;

        PatternMotion[1][2] = PatternMotion[1][6] = PatternMotion[2][2] = PatternMotion[2][6] = PatternMotion[3][6] = -Sz;
        PatternMotion[3][2] = -Sz - 10;
        PatternMotion[4][2] = PatternMotion[4][6] = PatternMotion[5][2] = PatternMotion[5][6] = PatternMotion[6][6] = Sz;
        PatternMotion[6][2] = Sz + 10;

        //PatternMotion[1][2] = PatternMotion[1][6] = PatternMotion[2][2] = PatternMotion[2][6] = PatternMotion[3][2] = PatternMotion[3][6] = -Sz;
        //PatternMotion[4][2] = PatternMotion[4][6] = PatternMotion[5][2] = PatternMotion[5][6] = PatternMotion[6][2] = PatternMotion[6][6] = Sz;


        if(Seh <= 0){
            PatternMotion[2][3] = PatternMotion[3][3] = PatternMotion[4][3] = PatternMotion[5][7] = PatternMotion[6][7] = PatternMotion[1][7] = -Sh - Seh;
            PatternMotion[2][7] = PatternMotion[3][7] = PatternMotion[4][7] = PatternMotion[5][3] = PatternMotion[6][3] = PatternMotion[1][3] = Sh;
        } else {
            PatternMotion[2][3] = PatternMotion[3][3] = PatternMotion[4][3] = PatternMotion[5][7] = PatternMotion[6][7] = PatternMotion[1][7] = -Sh;
            PatternMotion[2][7] = PatternMotion[3][7] = PatternMotion[4][7] = PatternMotion[5][3] = PatternMotion[6][3] = PatternMotion[1][3] = Sh + Seh;
        }

        if(Szg < 0){
            PatternMotion[6][2] = Szg - 3;
            PatternMotion[6][6] = -Szg;
            PatternMotion[1][2] = Szg + Szg - 3;
            PatternMotion[1][6] = 0;
        } else if(Szg > 0){
            PatternMotion[3][2] = -Szg;
            PatternMotion[3][6] = Szg + 3;
            PatternMotion[4][2] = 0;
            PatternMotion[4][6] = Szg + Szg + 3;
        }
    }
}

void Motion6step::MotionCounter(int fMth){
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

  if(fMth!=lastfMth){
    ChangeDataMotion(fMth);
    lastfMth=fMth;
  }

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

void Motion6step::StepTrajectoryWalking(int fMth, Dynamixel& motor){
  t = 10;
  if(theta > 180){
    for(i=0; i<8; i++){ lastPatternMotion[i] = PatternMotion[step][i]; } //take motion pattern matrix
    lastAngleBody = aBody;
    step ++;
    if(step > 6){
      step = 1;
      CountSTART++;
      if(CountSTART>=100) CountSTART=100;
    } //back to step 1
    if(step==1||step==4){ MotionCounter2(fMth); } // motion acc & decc
    theta = t;
    if(fMth == 210 && step == 6){ //sit
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

  if(step == 2 || step == 5){
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
  AngleBody = (theta / 180) * (aBody - lastAngleBody) + lastAngleBody;

  kinematics.Coordinate_Kaki(xFoot1, zFoot1, yFoot1, hFoot1, xFoot2, zFoot2, yFoot2, hFoot2);

  //Tangan();
  if( step == 2 || step == 3 || step == 5 || step == 6){
    theta+=t;
  } else {
    //if(theta > 170 && theta <= 180) theta+=(1);
    //else theta+=t;
    theta+=(3);
  }
  motor.moveMotor_degAll(kinematics.AngleJointAll,1,34);
}

void Motion6step::BasicMotionFSM(Dynamixel& motor){
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


#endif // MOTION_H
