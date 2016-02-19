/*
project_Flying_WingF3_V1  
by: tinnakon kheowree  
tinnakon_za@hotmail.com
tinnakonza@gmail.com
http://quad3d-tin.lnwshop.com
https://www.facebook.com/tinnakonza
*/
//Parameter system Quadrotor
#define m_quad 1.1 //kg
#define L_quad 0.225 //m quad

//////////////////RC//////////////////////////////////
//#define tarremote 0.025 // fast
#define tarremote 0.062  //0.092 slow 0.12 0.02 0.08 remote 
//#define tar 0.011 //0.012 0.015
#define tar 0.01

#define MINTHROTTLE 1064 
#define MAXTHROTTLE 1860
#define MINCOMMAND 1020
#define MAXCOMMAND 1980
#define MIDRC 1500
#define MINCHECK 1100
#define MAXCHECK 1900

//PID-------------Rate
float K_Rate = 0.952;//0.65 0.25

float Kp_rateRoll = 0.89;//1.05 1.12
float Ki_rateRoll = 0.65;//0.85 0.45
float Kd_rateRoll = 0.0013;//0.026 0.028 0.041 0.051

float Kp_ratePitch = 0.93;//1.12
float Ki_ratePitch = 0.65;//0.45
float Kd_ratePitch = 0.0013;//0.041

int SET_Roll1 = 1500;
int SET_PITCH1 = 1500;

#define TASK_100HZ 2
#define TASK_50HZ 3 //4
#define TASK_20HZ 10
#define TASK_10HZ 20
#define TASK_5HZ 40
#define TASK_2HZ 100
#define TASK_1HZ 200
#define TASK_noHZ 220
#define RAD_TO_DEG 57.295779513082320876798154814105

float sensorValue1 = 1.0;
float sensorValue2 = 1.0;
// Main loop variables
unsigned long currentTime = 0;
unsigned long previousTime = 0;
unsigned long sensorPreviousTime = 0;
uint8_t frameCounter = 0;
byte armed = 0;
float G_Dt = 0.01; 
long Dt_sensor = 1000;
long Dt_roop = 10000;
int Status_LED = LOW;
int ESC_calibra = 0;
