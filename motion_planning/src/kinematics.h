#ifndef KINEMATICS_H
#define KINEMATICS_H
#define id_max 20

class Kinematics{
    private:
        float Angle_Kaki[6] = {0,0,0,0,0,0};
        float Angle_Tangan[8] = {0,0,0,0,0,0,0,0};
        float Angle_Kepala[3] = {0,0,0};
        float Angle_Waist[3] = {0,0,0};
        float total_theta[4] = {0,0,0,0};
    public:
        float AngleJointAll[id_max+1];
        int step=0, time=0, tmp=0;
        float tmp_theta[4]={0,0,0,0};
        void InversKinematic_Kaki(float x, float y, float z, int Heading);
        void Coordinate_Kaki(float x1, float y1, float z1, int Heading1, float x2, float y2, float z2, int Heading2);
        void Angle_Jumping(float theta1, float theta2, float theta3, float theta4, int t, int path);
        void resetKinematics();
        void initialPosition(float theta1, float theta2, float theta3, float theta4);
};

/*void Kinematics::InversKinematic_Kaki(float x, float y, float z, int Heading){
  float Resultan0, Resultan1, Resultan2, aX, aY, Alfa;
  float sA = 0, sB = 0, sC = 0;
  float sBA, sBB, sCA, sCB, sD;

  Angle_Kaki[0] = Heading;//servo no 7-8
  Resultan0 = sqrt((y*y) + (x*x));

  Alfa = 	(atan2(y,x) * PHI) - Heading;
  aY = y + (sin(Alfa / PHI) * Resultan0);
  aX = x - (cos(Alfa / PHI) * Resultan0);

  sD = atan2(aY,z) * PHI;

  Resultan1 = sqrt(aY*aY + z*z);
  if(Resultan1 >= Frame3)Resultan1 = Frame3;
  Resultan2 = sqrt((Resultan1*Resultan1) + (aX*aX));
  if(Resultan2 >= Frame3)Resultan2 = Frame3;

  sCA = (float)((Frame1*Frame1) + (Frame2*Frame2)) - (Resultan2*Resultan2);
  sCB = (float)(2 * Frame1 * Frame2);
  sC = (float)acos(sCA / sCB) * PHI ;
  sA = (float)atan2(Resultan1,-x) * PHI;
  sBA = (float)Frame2 * sin(sC / PHI);
  sBB = (float)Frame1 + (Frame2 * cos(sC / PHI));
  sB = (float)atan2(sBA,sBB) * PHI;

  Angle_Kaki[1] = -sD;
  Angle_Kaki[2] = 0 - (sA - sB);
  Angle_Kaki[3] = (180 - sC);
  Angle_Kaki[4] = 0 - (Angle_Kaki[2] + Angle_Kaki[3]);
  Angle_Kaki[5] = -sD;
}*/

void Kinematics::InversKinematic_Kaki(float x, float y, float z, int Heading){
  float Resultan0, Resultan1, Resultan2, aX, aY, Alfa;
  float sA = 0, sB = 0, sC = 0;
  float sBA, sBB, sCA, sCB, sD;
  
  Angle_Kaki[0] = (Heading);//servo no 7-8
  Resultan0 = sqrt((y*y) + (x*x));
  
  Alfa =  (atan2(y,x) * PHI) - (Heading);
  aY = (sin(Alfa / PHI) * Resultan0);
  aX = (cos(Alfa / PHI) * Resultan0);
  
  sD = atan2(aY,z) * PHI;
  
  Resultan1 = sqrt(aY*aY + z*z);
  Resultan2 = sqrt((Resultan1*Resultan1) + (aX*aX));
  
  sCA = (float)((Frame1*Frame1) + (Frame2*Frame2)) - (Resultan2*Resultan2);
  sCB = (float)(2 * Frame1 * Frame2);
  sC = (float)acos(sCA / sCB) * PHI ;
  sA = (float)atan2(Resultan1,-x) * PHI;
  sBA = (float)Frame2 * sin(sC / PHI);
  sBB = (float)Frame1 + (Frame2 * cos(sC / PHI));
  sB = (float)atan2(sBA,sBB) * PHI;
  
  Angle_Kaki[1] = sD;
  Angle_Kaki[2] = 0 - (sA - sB);
  Angle_Kaki[3] = (180 - sC);
  Angle_Kaki[4] = Angle_Kaki[2] + Angle_Kaki[3];
  Angle_Kaki[5] = -sD;
}

void Kinematics::Coordinate_Kaki(float x1, float y1, float z1, int Heading1, float x2, float y2, float z2, int Heading2){
    int dataMiring;
    float estimasi = 0;
    if(state != 20)dataMiring = 0; //err_y with IMU
    else dataMiring = 0;

    Kinematics::InversKinematic_Kaki(x1, y1, z1, Heading1);
    AngleJointAll[11] = Angle_Kaki[0];
    AngleJointAll[9] = -Angle_Kaki[1];
    AngleJointAll[7] = -(float)Angle_Kaki[2] + estimasi;//- AngleBody * cos(Angle[0] / PHI);
    AngleJointAll[5] = -(float)Angle_Kaki[3] - estimasi - estimasi;
    AngleJointAll[3] = (Angle_Kaki[4] - estimasi)*-1;
    AngleJointAll[1] = -Angle_Kaki[5]*-1;

    Kinematics::InversKinematic_Kaki(x2, y2, z2, Heading2);
    AngleJointAll[12] = Angle_Kaki[0];
    AngleJointAll[10] = -Angle_Kaki[1];
    AngleJointAll[8] = (float)Angle_Kaki[2] + estimasi;//- AngleBody * cos(Angle[0] / PHI);
    AngleJointAll[6] = (float)Angle_Kaki[3] - estimasi - estimasi;
    AngleJointAll[4] = -(Angle_Kaki[4] - estimasi)*-1;
    AngleJointAll[2] = -Angle_Kaki[5]*-1;

    AngleJointAll[13] = 0;
    AngleJointAll[14] = 0;
    AngleJointAll[15] = 0;
    AngleJointAll[16] = 0;
    AngleJointAll[17] = 0;
    AngleJointAll[18] = 0;
    AngleJointAll[19] = 0;
    AngleJointAll[20] = 0;
  }

void Kinematics::Angle_Jumping(float theta1, float theta2, float theta3, float theta4, int t, int path){
    float a1=3, a2=7, a3=7, a4=10;

    float Qx = a1*cos(theta1)+a2*cos(theta1+theta2)+a3*cos(theta1+theta2+theta3)+a4*cos(theta1+theta2+theta3+theta4);
    float Qy = a1*sin(theta1)+a2*sin(theta1+theta2)+a3*sin(theta1+theta2+theta3)+a4*sin(theta1+theta2+theta3+theta4);

    //theta1/=path;
    //theta2/=path;
    //theta3/=path;
    //theta4/=path;

    if(time==0 && step==0){
      time++;
      step++;

      AngleJointAll[4] = tmp_theta[0] + (((theta1 - tmp_theta[0])/path)*step);
      AngleJointAll[3] = -AngleJointAll[4];
      AngleJointAll[6] = tmp_theta[1] + ((theta2 - tmp_theta[1])/path*step);
      AngleJointAll[5] = -AngleJointAll[6];
      AngleJointAll[7] = tmp_theta[2] + ((theta3 - tmp_theta[2])/path*step);
      AngleJointAll[8] = -AngleJointAll[7];
      AngleJointAll[14] = tmp_theta[3] + ((theta4 - tmp_theta[3])/path*step);
    }
    else if(time>t && step<path){
      time=0;
      step++;

      AngleJointAll[4] = tmp_theta[0] + (((theta1 - tmp_theta[0])/path)*step);
      AngleJointAll[3] = -AngleJointAll[4];
      AngleJointAll[6] = tmp_theta[1] + ((theta2 - tmp_theta[1])/path*step);
      AngleJointAll[5] = -AngleJointAll[6];
      AngleJointAll[7] = tmp_theta[2] + ((theta3 - tmp_theta[2])/path*step);
      AngleJointAll[8] = -AngleJointAll[7];
      AngleJointAll[14] = tmp_theta[3] + ((theta4 - tmp_theta[3])/path*step);
    }
    else if(time>t && step>=path){
      time=0;
      step++;
    }
    else time++;

    //printf("step=%d time=%d tmp_theta1=%.2f theta1=%.2f joint4=%.2f \n",step,time,tmp_theta[0],theta1,AngleJointAll[4]);
    //printf("step=%d time=%d tmp_theta2=%.2f theta2=%.2f joint6=%.2f \n",step,time,tmp_theta[1],theta2,AngleJointAll[6]);
    //printf("step=%d time=%d tmp_theta3=%.2f theta3=%.2f joint4=%.2f \n",step,time,tmp_theta[2],theta3,AngleJointAll[7]);
    //printf("step=%d time=%d tmp_theta4=%.2f theta4=%.2f joint4=%.2f \n",step,time,tmp_theta[3],theta4,AngleJointAll[14]);


    AngleJointAll[1] = 0;
    AngleJointAll[2] = 0;
    AngleJointAll[9] = 0;
    AngleJointAll[10] = 0;
    AngleJointAll[11] = 0;
    AngleJointAll[12] = 0;
    AngleJointAll[13] = 0;
    AngleJointAll[15] = 0;
    AngleJointAll[16] = 0;
    AngleJointAll[17] = 0;
    AngleJointAll[18] = 0;
    AngleJointAll[19] = 0;
    AngleJointAll[20] = 0;
}

void Kinematics::resetKinematics(){
    for(int i=1; i<=20; i++)AngleJointAll[i]=0;
    step=0;
}

void Kinematics::initialPosition(float theta1, float theta2, float theta3, float theta4){
    tmp_theta[0]=theta1;
    tmp_theta[1]=theta2;
    tmp_theta[2]=theta3;
    tmp_theta[3]=theta4;
}

/*void Kinematics::Coordinate_Kaki(float x1, float y1, float z1, int Heading1, float x2, float y2, float z2, int Heading2){
  int dataMiring;
  float AngleJ = 0;
  //if(state != 20) dataMiring = err_y[1];//err_y; //err_y with IMU
  if(state != 20) dataMiring = 0;
  else dataMiring = 0;
  
  Kinematics::InversKinematic_Kaki(x1, y1, z1, Heading1);
  AngleJointAll[1] = Angle_Kaki[0];  
  if(z1 < z2){    
    AngleJointAll[3] = (Angle_Kaki[1]  - AngleBody * sin(Angle_Kaki[0] / PHI))*1.5 + (float)dataMiring/6;
    AngleJointAll[11] = (Angle_Kaki[5]*1.5) - (float)dataMiring/6;
  }
  else{ 
    AngleJointAll[3] = (Angle_Kaki[1]  - AngleBody * sin(Angle_Kaki[0] / PHI)) + (float)dataMiring / 6; 
    AngleJointAll[11] = Angle_Kaki[5] - (float)dataMiring/6;
  }   
  AngleJointAll[5] = Angle_Kaki[2] + AngleJ - AngleBody * cos(Angle_Kaki[0] / PHI);
  AngleJointAll[7] = Angle_Kaki[3] - (AngleJ + AngleJ);
  AngleJointAll[9] = (Angle_Kaki[4] - AngleJ);
//  if(z1 > z2){ AngleJointServo[8] = ((Angle[4] - AngleJ) + (z1 - z2)/20);}
//  else{ AngleJointServo[8] = (Angle[4] - AngleJ); }
  
  Kinematics::InversKinematic_Kaki(x2, y2, z2, Heading2);
  AngleJointAll[2] = Angle_Kaki[0];  
  if(z1 > z2){   
    AngleJointAll[4] = (Angle_Kaki[1] - AngleBody * sin(Angle_Kaki[0] / PHI))*1.5 + (float)dataMiring/6; 
    AngleJointAll[12] = (Angle_Kaki[5]*1.5) - (float)dataMiring/6;
  }
  else{  
    AngleJointAll[4] = (Angle_Kaki[1]  - AngleBody * sin(Angle_Kaki[0] / PHI)) + (float)dataMiring / 6;
    AngleJointAll[12] = Angle_Kaki[5] - (float)dataMiring/6;
  } 
  AngleJointAll[6] = Angle_Kaki[2] + AngleJ - AngleBody * cos(Angle_Kaki[0] / PHI);
  AngleJointAll[8] = Angle_Kaki[3] - (AngleJ + AngleJ);
  AngleJointAll[10] = (Angle_Kaki[4] - AngleJ);

    AngleJointAll[13] = 0;
    AngleJointAll[14] = 0;
    AngleJointAll[15] = 0;
    AngleJointAll[16] = 0;
    AngleJointAll[17] = 0;
    AngleJointAll[18] = 0;
    AngleJointAll[19] = 0;
    AngleJointAll[20] = 0;
//  if(z1 < z2){  AngleJointServo[9] = ((Angle[4] - AngleJ) + (z2 - z1)/20);}
//  else{   AngleJointServo[9] = (Angle[4] - AngleJ);}
}*/

#endif // KINEMATICS_H
