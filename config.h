//#include "config.h"
///////////////The type of multicopter////////////////////////   
//#define HEX_X
#define Quad_X
//#define Quad_P
//#define PPM
/////////////////////////////////////////////////////////////
//PID-------------Rate
float Kp_rateRoll = 1.18;//2.78 1.18 5.28
float Kpi_rateRoll = 0.82;//2.12 1.02 1.32
float Ki_rateRoll = 2.12;//2.75
float Kd_rateRoll = 0.062;//0.085 0.025 - 0.045

float Kp_ratePitch = 1.18;//1.18 5.28
float Kpi_ratePitch = 0.82;//1.02 1.32
float Ki_ratePitch = 2.12;//2.75 0.5 - 2.8
float Kd_ratePitch = 0.062;//0.078 0.025 - 0.045

float Kp_rateYaw = 3.75;//3.75 5.75 1.75 - 3.450  350.0
float Kpi_rateYaw = 0.0;//0.0
float Ki_rateYaw = 3.65;//3.65  2.95
float Kd_rateYaw = 0.015;//0.035 0.065

//PID--------------Stable
float Kp_levelRoll= 7.9;// 6.2 7.8 9.2 
float Ki_levelRoll= 0.00;//0.0
float Kd_levelRoll= 0.00;//0.0

float Kp_levelPitch= 7.9;//6.2 9.2 
float Ki_levelPitch= 0.00;
float Kd_levelPitch= 0.00;

float Kp_levelyaw= 0.0;

//stat feedback--------------Altitude
float Kp_altitude = 385.0;//265 165.0
float Ki_altitude = 32.5;//32.5,0.0
float Kd_altitude = 160.0;//120 160
float Ka_altitude = 28.5;//35 25 - 37
//////////////////RC//////////////////////////////////
//#define tarremote 0.025 // fast
#define tarremote 0.065  //slow 0.095 0.12 0.02 0.08 remote 
//////////////////////////////////////////////////
//PID GPS
float Kp_gps = 0.15;//0.15 0.101 2.101 5.101
float Kd_gps = 3.9;//4.3 0.35 1.35 3.35
float Kp_speed = 1.45;//1.1

//GPS //สตาร์ท
float GPS_LAT_HOME = 13.867000579 ;//13.867000579  13.867021560  13.867017745      14.907173156
float GPS_LON_HOME = 100.483291625; //100.483291625 100.483261108  100.483276367  100.206214904
//ลงจอด
float waypoint1_LAT = 13.867000579;//F, 13.875096, 100.484546     ,R     13.867492, 100.501004
float waypoint1_LON = 100.483291625;//B, 13.857347, 100.483344   ,L     13.866868, 100.473152
//
float waypoint2_LAT = 13.867048263;
float waypoint2_LON = 100.483268737;

float GPS_LAT1 = 0.0;
float GPS_LON1 = 0.0;
// Automatic take-off and landing 
#define h_control 0.7  //0.6 0.9 meter
float Altitude_Hold = 0.0;

#define tar 0.011 //0.012 0.015
//Parameter system Quadrotor
#define m_quad 1.1 //kg
#define L_quad 0.25 //m

//magnetometer calibration constants; use the Calibrate example from
// the Pololu library to find the right values for your board
int M_X_MIN = -490;    //-654  -693   -688
int M_X_MAX = 310;     //185   209    170
int M_Y_MIN = -369;    //-319  -311   -310
int M_Y_MAX = 397;     //513   563    546
int M_Z_MIN = -392;    //-363  -374   -377
int M_Z_MAX = 346;     //386   429    502

//Observer hz
float Altitude_hat=0.0;//Observer hx
float vz_hat=0.0;
float h=0.0;
float seth=0.0;//set control
float uthrottle=0.0;
float uAltitude = 1000.0;
float accrX_Earth = 0.0;
float accrY_Earth = 0.0;
float accrZ_Earth = 0.0;
float accrZ_Earthf = 0.0;
//float vz = 0.0;

//GPS
float GPS_speed = 0.0;
int GPS_FIX = 0;
float actual_speedX = 0.0;
float actual_speedY = 0.0;
float actual_speedXf = 0.0;
float actual_speedYf = 0.0;
float actual_speedXff = 0.0;
float actual_speedYff = 0.0;
float GPS_LAT1_old = GPS_LAT_HOME;
float GPS_LON1_old = GPS_LON_HOME;
float Control_XEf = 0.0;
float Control_YEf = 0.0;
float Control_XBf = 0.0;
float Control_YBf = 0.0;
float target_LAT = 0.0;
float target_LON = 0.0;
byte currentCommand[23];
byte Read_command = 0;
float GPS_hz = 0.0;
float GPS_vz = 0.0;
float GPS_ground_course = 0.0;
float GPS_Distance = 0.0;
float error_LAT = 0.0;
float error_LON = 0.0;

#define TASK_100HZ 1
#define TASK_50HZ 2
#define TASK_20HZ 5
#define TASK_10HZ 10
#define TASK_5HZ 20
#define TASK_1HZ 100
#define RAD_TO_DEG 57.295779513082320876798154814105

  //direction cosine matrix (DCM)   Rotated Frame to Stationary Frame ZYX
  float DCM00 = 0.0;
  float DCM01 = 1.0;
  float DCM02 = 0.0;
  float DCM10 = -1.0;
  float DCM11 = 0.0;
  float DCM12 = 0.0;
  float DCM20 = 0.0;
  float DCM21 = 0.0;
  float DCM22 = 1.0;
  float cos_rollcos_pitch = 1.0;
  
// Main loop variables
unsigned long currentTime = 0;
unsigned long previousTime = 0;
unsigned long sensorPreviousTime = 0;
unsigned long GPS_loopTimer = 0;

uint8_t frameCounter = 0;
uint8_t timeLanding = 0;
uint8_t timeOff = 0;
byte armed = 0;
float G_Dt = 0.01; 
float Dt_GPS = 0.2;

long Dt_sensor = 1000;
long Dt_roop = 10000;
int Status_LED = LOW;
int ESC_calibra = 0;
