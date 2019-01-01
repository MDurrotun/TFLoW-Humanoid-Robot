#ifndef GLOBALVARS_H
#define GLOBALVARS_H

//****************************************** GLOBAL VARIABEL *************************************************//

float AngleJointServo[12] = { 0,0,0,0,0,0,0,0,0,0,0,0 };
float Frame1 = 100;
float Frame2 = 100;
float Frame3 = 200;
float PHI = (float)57.295779513082320876798154814105;
float Angle[6] = {0,0,0,0,0,0};
float theta=0;
float LengthX1,LengthY1,LengthZ1,Heading1,LengthX2,LengthY2,LengthZ2,Heading2,HipZ,HipH;

unsigned char m=0,n=1,i=0,l=0,j=0;
unsigned char state=WALK;
unsigned char START = 0, CountSTART = 0;
unsigned char iii,CS1=0, CS2=0, CS3=0, CS6=0;
unsigned char dat1, dat2, dat3, dat4, dat5, dat6;
float AngleBody = 0, lastAngleBody = 0;
unsigned char Mode, Action, DataTX, DataTY, ready, it, DataPC[8], Servo[2];

short int GoalPosition[34];
short int sMotion[32][20];
short int Selisih;
short int T;
short int VCos[181]=
{0,0,1,1,2,4,5,7,10,12,15,18,22,26,30,34,39,44,49,54,60,66,73,79,86,94,101,109,117,125,134,143,152,161,171,181,191,201,212,223,234,245,257,269,281,293,305,318,331,344,357,371,384,398,412,426,441,455,470,485,500,515,531,546,562,577,593,609,625,642,658,674,691,708,724,741,758,775,792,809,826,844,861,878,895,913,930,948,965,983,1000,1017,1035,1052,1070,1087,1105,1122,1139,1156,1174,1191,1208,1225,1242,1259,1276,1292,1309,1326,1342,1358,1375,1391,1407,1423,1438,1454,1469,1485,1500,1515,1530,1545,1559,1574,1588,1602,1616,1629,1643,1656,1669,1682,1695,1707,1719,1731,1743,1755,1766,1777,1788,1799,1809,1819,1829,1839,1848,1857,1866,1875,1883,1891,1899,1906,1914,1921,1927,1934,1940,1946,1951,1956,1961,1966,1970,1974,1978,1982,1985,1988,1990,1993,1995,1996,1998,1999,1999,2000,2000 };

int lastfMth=0;
int counter=0,dcount=18;
int motion=0;
int lastPatternMotion[8]={0,200,0,0,0,200,0,0};
int lastTangan;

#endif // GLOBALVARS_H
