/*
project_Quad_rotor v1.1  
1. Automatic  Takeoff 
2. 1 waypoint navigation
3. Landing
by: tinnakon kheowree  
tinnakon_za@hotmail.com
tinnakonza@gmail.com

date: 13-05-2557(2014)  V.1 Quad-X
date: 22-05-2557(2014)  V.3 , ahrs, pid, Moving Average Filters
date: 25-05-2557(2014)  V.4 ,filter remote , nois gyro +-0.1 ,tuning Altitude Hold

Automatic  Takeoff Landing waypoint navigation Remote Trim Acc
Altitude Hold, Fail_Safe()

support:  Board MEGAWAP_V2_AIO
• Atmega2560
• ITG3205 Triple Axis Gyro  //400kHz
• BMA180 Accelerometer //400kHz
• BMP085 Barometer
• HMC5883L Magnetometer //400kHz

Quad-X
       
pin 2 FRONTL  M1CW        M2CCW  FRONTR pin 5
              \         / 
                \ --- /
                 |   |
                / --- \
              /         \ 
pin 6 REARL  M4 CCW      M3 CW  REARR  pin 3 

Quad=> +
              F
             D2 >>          
              /
L  D6 << ----    -----  D5 <<   R      
              /
             D3 >>
              B
              
---------motor---------
Front  => D2
Right => D5
Left  => D6
Back => D3

----------rx-----------           
Throttle  => A8
Aileron   => A9
Elevator  => A10
Ruder     => A11
Aux1       => A12
Aux2       => A13
*/
#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "multi_rx2560.h"
#include "sensor.h"
#include "ahrs.h"
//#include "bmp085.h"
//#include "Kalmantin.h"
#include "Control_PID.h"
#include "motorX4.h"

void setup()
{
  Serial.begin(38400);//38400
  Serial1.begin(115200);//CRIUS Bluetooth Module pin code 0000
  Serial2.begin(115200);//read gps from 1280
  Serial3.begin(38400);//3DR Radio Telemetry Kit 433Mhz
  pinMode(13, OUTPUT);pinMode (30, OUTPUT);pinMode (31, OUTPUT);//(13=A=M),(31=B=STABLEPIN),(30=C,GPS FIG LEDPIN)
  configureReceiver();//find multi_rx.h
  motor_initialize();//find motor.h
  ESC_calibration();//find motor.h
  Wire.begin();
  digitalWrite(13, HIGH);
  delay(10);
  AccelerometerInit();//sensor.h
  delay(10);
  GyroITGInit();//sensor.h
  delay(10);  
  MagHMCInt();//sensor.h
  delay(10); 
  TWBR = ((F_CPU / 400000L) - 16) / 2; // change the I2C clock rate to 400kHz 
      for (uint8_t i=0; i<10; i++) 
    {
     AccelerometerRead();
     GyroRead();
     MagRead();
     delay(20);
    }
    digitalWrite(13, LOW);
  sensor_Calibrate();//sensor.h
  ahrs_initialize();//ahrs.h
  //baroInitialize();//bmp085.h
  RC_Calibrate();
  sensorPreviousTime = micros();
  previousTime = micros();
  GPS_loopTimer = millis();
}

void loop()
{
  while(Serial2.available() >= 22)//data 22 byte
   {
     byte byteGPS=Serial2.read();//Read header
     if(byteGPS == 0x24)////The header byte is represented by value 36 decimal or 0x24 hex.
     {
       for(uint8_t i=0; i<21; i++)    // Build Command From Serial Buffer //data 21 byte
       {
        currentCommand[i] = Serial2.read();       
       }
union ua1_tag 
{
    byte b[4];
    float fval;
} ua1;
ua1.b[0] = currentCommand[0];
ua1.b[1] = currentCommand[1];
ua1.b[2] = currentCommand[2];
ua1.b[3] = currentCommand[3];
GPS_LAT1 = ua1.fval;
ua1.b[0] = currentCommand[4];
ua1.b[1] = currentCommand[5];
ua1.b[2] = currentCommand[6];
ua1.b[3] = currentCommand[7];
GPS_LON1 = ua1.fval;
ua1.b[0] = currentCommand[8];
ua1.b[1] = currentCommand[9];
ua1.b[2] = currentCommand[10];
ua1.b[3] = currentCommand[11];
GPS_speed = ua1.fval - 3.5;//15.5 cm/s
ua1.b[0] = currentCommand[12];
ua1.b[1] = currentCommand[13];
ua1.b[2] = currentCommand[14];
ua1.b[3] = currentCommand[15];
GPS_hz = ua1.fval;
ua1.b[0] = currentCommand[16];
ua1.b[1] = currentCommand[17];
ua1.b[2] = currentCommand[18];
ua1.b[3] = currentCommand[19];
GPS_vz = ua1.fval;
GPS_FIX = currentCommand[20];
  //Diff speed
  Dt_GPS = (float)(millis() - GPS_loopTimer)/ 1000.0;
  GPS_loopTimer = millis();
  if(Dt_GPS <= 0 || Dt_GPS > 10.0)
  {
    Dt_GPS = 0.2;
  }
  actual_speedX = (GPS_LAT1 - GPS_LAT1_old)*1000000000.0*Dt_GPS;//cm/s  10000000.0
  actual_speedY = (GPS_LON1 - GPS_LON1_old)*1000000000.0*Dt_GPS;//cm/s
  actual_speedX = constrain(actual_speedX, -200, 200);//+-200 cm/s
  actual_speedY = constrain(actual_speedY, -200, 200);//+-200 cm/s
     }//end if The header
 }//end while roop data rs232  
  Dt_sensor = micros() - sensorPreviousTime;///////////Roop sensor/////////
  if(Dt_sensor <= 0)
  {
    Dt_sensor = 1001;
  }
    if(Dt_sensor >= 1000 && sensorSamples < 4)////Collect 3 samples = 2760 us  && sensorSamples < 3
    {  
        sensorPreviousTime = micros();
        sensor_readSum();
    }
   Dt_roop = micros() - previousTime;// 100 Hz task loop (10 ms)  , 5000 = 0.02626 ms
   if(Dt_roop <= 0)
   {
    Dt_roop = 10001; 
   }   
    if (Dt_roop >= 10000) 
    {
      previousTime = micros();
      G_Dt = Dt_roop*0.000001;
      frameCounter++;
      sensor_Get();//Read sensor
////////////////Moving Average Filters 4///////////////////////////
      GyroXf2 = (GyroX + GyroX2)/2.0;
      GyroYf2 = (GyroY + GyroY2)/2.0;
      GyroZf2 = (GyroZ + GyroZ2)/2.0;
      GyroXf = (GyroXf2 + GyroX3)/2.0;
      GyroYf = (GyroYf2 + GyroY3)/2.0;
      GyroZf = (GyroZf2 + GyroZ3)/2.0;
      AccXf2 = (AccX + AccX2)/2.0;
      AccYf2 = (AccY + AccY2)/2.0;
      AccZf2 = (AccZ + AccZ2)/2.0;
      AccXf = (AccXf2 + AccX2)/2.0;
      AccYf = (AccYf2 + AccY2)/2.0;
      AccZf = (AccZf2 + AccZ2)/2.0;
      AccX2 = AccX;AccY2 = AccY;AccZ2 = AccZ;//acc Old1
      AccX3 = AccXf2;AccY3 = AccYf2;AccZ3 = AccZf2;//acc Old2
      GyroX2 = GyroX;GyroY2 = GyroY;GyroZ2 = GyroZ;//gyro Old1
      GyroX3 = GyroXf2;GyroY3 = GyroYf2;GyroZ3 = GyroZf2;//gyro Old2      
////////////////Low pass filter/////////////////////////////////
      //GyroXf = GyroXf + (GyroX - GyroXf)*49.6*G_Dt; //29 - 49.4
      //GyroYf = GyroYf + (GyroY - GyroYf)*49.6*G_Dt;
      //GyroZf = GyroZf + (GyroZ - GyroZf)*49.6*G_Dt;
      //AccXf2 = AccXf2 + (AccX - AccXf2)*49.6*G_Dt;//15.4  //Low pass filter ,smoothing factor  α := dt / (RC + dt)
      //AccYf = AccYf + (AccY - AccYf)*49.6*G_Dt;//15.4
      //AccZf = AccZf + (AccZ - AccZf)*49.6*G_Dt;//15.4
//////////////////////////////////////////////////////////
      ahrs_updateMARG(GyroXf, GyroYf, GyroZf, -AccXf, -AccYf, AccZf, c_magnetom_x, c_magnetom_y, c_magnetom_z, G_Dt);//quaternion ,direction cosine matrix ,Euler angles
      //x_angle = kalmanCalculateX(ahrs_r*RAD_TO_DEG, GyroX*RAD_TO_DEG, G_Dt);
      //y_angle = kalmanCalculateY(ahrs_p*RAD_TO_DEG, GyroY*RAD_TO_DEG, G_Dt);

      //Observer hz kalman , GPS_hz , GPS_vz
      //baro_vzfilter = baro_vzfilter + (baro_vz - baro_vzfilter)*1.17*G_Dt;//2.5 Low Pass Filter vz baro
 
      float temp_vz = accrZ_Earth + 8.72*(GPS_vz - vz_hat);//4.7 15.5
      vz_hat = vz_hat + temp_vz*G_Dt;
      vz_hat = constrain(vz_hat, -2, 2);//+-2 m/s
      float temp_hz = vz_hat + 12.54*(GPS_hz - Altitude_hat);//12.54 4.5
      Altitude_hat = Altitude_hat + temp_hz*G_Dt;
      Altitude_hat = constrain(Altitude_hat, 0.0, 1.2);//1.5 m By Ultrasonic
      //vz = vz + accrZ_Earth*G_Dt;
/*
      else//By Baro
      {
       float temp_vz = accrZ_cutg + 0.85*(baro_vzfilter - vz_hat);//0.85
       vz_hat = vz_hat + temp_vz*G_Dt;
       vz_hat = constrain(vz_hat, -2, 2);//+-2 m/s
       float temp_hz = vz_hat + 12.54*(baroAltitudeRunning - Altitude_hat);//12.54 4.5
       Altitude_hat = Altitude_hat + temp_hz*G_Dt;
      }
 */     
      //GPS Speed Low Pass Filter
     actual_speedXf = actual_speedXf + (actual_speedX - actual_speedXf)*10.17*G_Dt;//8.4  //cm/s
     actual_speedYf = actual_speedYf + (actual_speedY - actual_speedYf)*10.17*G_Dt;//1.17
     actual_speedXf = constrain(actual_speedXf, -200, 200);//+-200 cm/s
     actual_speedYf = constrain(actual_speedYf, -200, 200);//+-200 cm/s
//PID Control///////////
     Control_PIDRate();//Control_PID.h
     
         if (time_auto > 4 && endAuto == 0) //End waypoint quadrotor
        {
          timeOff++;
          if(timeOff >= 30)//relay 0.3 s timeOff
          {
           armed = 0;
          }
        }     
//////Out motor///////////
     motor_Mix();//"motor.h"
/////////////////////////
     motor_command(); ////////end Out motor//////

 if (frameCounter % TASK_50HZ == 0)// 50 Hz tak (20 ms)
 {
         computeRC();//multi_rx.h
         //Fail_Safe();
       if (CH_THR < MINCHECK)  //////ARM and DISARM your Quadrotor///////////////
        {
            if (CH_RUD > MAXCHECK && armed == 0) 
            {
                armed = 1;
                digitalWrite(30, HIGH);
            }
            if (CH_RUD < MINCHECK && armed == 1) 
            {
                armed = 0;
                countFail_Safe = 0;
                digitalWrite(30, LOW);
            }
            if (CH_RUD < MINCHECK && armed == 0 && CH_ELE > MAXCHECK) //Mag_Calibrate
            {
              Mag_Calibrate();//#include "mpu6050.h"
            }
        }//end  ARM and DISARM your helicopter///////////////       
   
}//end roop 50 Hz 
         //if (frameCounter % TASK_20HZ == 0)// 20 Hz task (50 ms)
        //{
            //measureBaro();
            //getBaroAltitude();
            //baro_vz = (baroAltitudeRunning - baroAltitudeold)/0.08;
            //baro_vz = constrain(baro_vz, -2, 2);//2 m/s
       // }//end roop 20 Hz
         if (frameCounter % TASK_10HZ == 0)// 10 Hz task (100 ms)
        {
            //sensorBaroTime = millis();
            //baroAltitudeold = baroAltitudeRunning;
            Automatictakeland();
            MagRead();  
            Chack_Command();//Control pid
            Fail_Safe();//"multi_rx.h"
        }//end roop 10 Hz
         if (frameCounter % TASK_5HZ == 0)//GPS_calc TASK_5HZ
        {
         //baroAltitudeold = baroAltitudeRunning;
         GPS_calc_positionhold();
        }
         if (frameCounter % TASK_20HZ == 0)//roop print  ,TASK_5HZ  TASK_10HZ
        {
            //Serial.print(CH_THR);Serial.print("\t");
            //Serial.print(CH_AIL);Serial.print("\t");  
            //Serial.print(CH_ELE);Serial.print("\t");
            //Serial.print(CH_RUD);Serial.print("\t");  
            //Serial.print(AUX_1);Serial.print("\t"); 
            //Serial.print(AUX_2);Serial.print("\t"); 

            //Serial.print(CH_AILf);Serial.print("\t");  
            //Serial.print(CH_ELEf);Serial.print("\t");
            //Serial.print(CH_RUDf);Serial.print("\t"); 
            
            //Serial.print(setpoint_rate_roll);Serial.print("\t");
            
            //Serial.print(MagX);Serial.print("\t");
            //Serial.print(MagY);Serial.print("\t");
            //Serial.print(MagZ);Serial.print("\t");  
            //Serial3.print(MagX);Serial3.print("\t");
            //Serial3.print(MagY);Serial3.print("\t");
            //Serial3.print(MagZ);Serial3.print("\t"); 
            //Serial.print(c_magnetom_x);Serial.print("\t");
            //Serial.print(c_magnetom_y);Serial.print("\t");
            //Serial.print(c_magnetom_z);Serial.print("\t"); 
            
            //Serial.print(GPS_FIX1);Serial.print("\t");
            //Serial3.print(GPS_LAT1,9);Serial3.print("\t"); 
            //Serial3.print(GPS_LON1,9);Serial3.print("\t");
            //Serial3.print(GPS_speed1);Serial3.print("\t");//cm/s
            //Serial3.print(actual_speedXff);Serial3.print("\t");
            //Serial3.print(actual_speedXf);Serial3.print("\t");
            //Serial3.print(actual_speedYff);Serial3.print("\t");
            //Serial3.print(actual_speedYf);Serial3.print("\t");
            //Serial3.print(GPS_Distance);Serial3.print("\t");
            //Serial3.print(GPS_ground_course);Serial3.print("\t");
            //Serial3.print(Control_XEf);Serial3.print("\t");
            //Serial3.print(Control_YEf);Serial3.print("\t");
            //Serial3.print(Control_XBf);Serial3.print("\t");
            //Serial3.print(Control_YBf);Serial3.print("\t");
            
            //Serial.print(Control_XEf);Serial.print("\t");
            //Serial.print(Control_YEf);Serial.print("\t");
            //Serial.print(Control_XBf);Serial.print("\t");
            //Serial.print(Control_YBf);Serial.print("\t");
            
            //Serial.print(uthrottle);Serial.print("\t");
            //Serial.print(baroAltitudeRunning);Serial.print("\t");
            //Serial3.print(baroAltitudeRunning);Serial3.print("\t");
            //Serial.print(h_counter);Serial.print("\t");
            //Serial.print(GPS_hz);Serial.print("\t"); 
            //Serial.print(Altitude_hat);Serial.print("\t");
            //Serial3.print(Altitude_hat);Serial3.print("\t");
            //Serial.print(GPS_vz*10);Serial.print("\t");
            //Serial.print(vz_hat*10);Serial.print("\t");
            //Serial3.print(vz_hat*10);Serial3.print("\t");
            //Serial.print(baro_vzfilter*10);Serial.print("\t");
            
            //Serial.print(motor_Front);Serial.print("\t");
            
            //Serial.print(set_pitch_rate);Serial.print("\t");
            
            //Serial.print(AccX);Serial.print("\t");
            //Serial.print(AccXf);Serial.print("\t");
            //Serial.print(AccY);Serial.print("\t");  
            //Serial.print(AccYf);Serial.print("\t"); 
            //Serial.print(AccZ,3);Serial.print("\t");
            //Serial.print(AccZf);Serial.print("\t");       
            //Serial.print(accrX_Earth);Serial.print("\t");
            //Serial.print(accrY_Earth);Serial.print("\t");
            //Serial.print(accrZ_Earth);Serial.print("\t");
            //Serial.print(vz);Serial.print("\t"); 
            
            //Serial.print(GyroX*RAD_TO_DEG);Serial.print("\t");
            //Serial.print(GyroXf*RAD_TO_DEG);Serial.print("\t");
            //Serial.print(GyroY*RAD_TO_DEG);Serial.print("\t");
            //Serial.print(GyroYf*RAD_TO_DEG);Serial.print("\t");
            //Serial.print(GyroZ*RAD_TO_DEG);Serial.print("\t");
            //Serial.print(GyroZf*RAD_TO_DEG);Serial.print("\t");
   
            Serial.print(ahrs_r*RAD_TO_DEG);Serial.print("\t");
            Serial.print(ahrs_p*RAD_TO_DEG);Serial.print("\t");  
            //Serial.print(ahrs_y*RAD_TO_DEG);Serial.print("\t");  
            //Serial3.print(ahrs_y*RAD_TO_DEG);Serial3.print("\t"); 
            //Serial.print(cos_rollcos_pitch);Serial.print("\t"); 
             
            //Serial.print(x_angle,3);Serial.print("\t");
            
            //Serial.print(err_pitch_rate);Serial.print("\t");
            
            Serial.print(sensorSamples2);Serial.print("\t");
            //Serial.print(Dt_sensor);Serial.print("\t");
            Serial.print(G_Dt*1000);Serial.print("\t");
            //Serial.print(millis()/1000.0);//millis() micros()
            Serial.print("\n"); 
            Serial3.print("\n");
        }//end roop 5 Hz 
        if (frameCounter >= TASK_1HZ) { // Reset frameCounter back to 0 after reaching 100 (1s)
            frameCounter = 0;
            time_auto++;
            Remote_TrimACC();//motor.h
              if(Status_LED == LOW)
            {
            Status_LED = HIGH;
            }
            else
            {
            Status_LED = LOW;
            }
            digitalWrite(13, Status_LED);
            if(GPS_FIX == 1)
            {
              digitalWrite(31, !Status_LED);
            }
  
        }//end roop 1 Hz
    }//end roop 100 HZ 
}
