//#include "sensor.h"//sensor************************************************
#define ITG3200_Address 0x68
#define BMA180_Address 0x40 
#define HMC5883_Address 0x1E
float GyroX,GyroY,GyroZ,GyroTemp,AccX,AccY,AccZ;
float GyroX2,GyroY2,GyroZ2;
float GyroX3,GyroY3,GyroZ3;
float GyroXf,GyroYf,GyroZf;
float GyroXf2,GyroYf2,GyroZf2;
float AccXf,AccYf,AccZf;
float AccXf2,AccYf2,AccZf2;
float AccX2,AccY2,AccZ2;
float AccX3,AccY3,AccZ3;

int MagX,MagY,MagZ;
float gyroSumX,gyroSumY,gyroSumZ,AccSumX,AccSumY,AccSumZ;
float gyro_offsetX,gyro_offsetY,gyro_offsetZ,acc_offsetX,acc_offsetY,acc_offsetZ;
float sensorSamples = 0.0;
int sensorSamples2 = 0;

float c_magnetom_x;
float c_magnetom_y;
float c_magnetom_z;

#define applyDeadband(value, deadband)  \
  if(abs(value) < deadband) {           \
    value = 0;                          \
  } else if(value > 0){                 \
    value -= deadband;                  \
  } else if(value < 0){                 \
    value += deadband;                  \
  }
 
void AccelerometerInit()
{
  Wire.beginTransmission(BMA180_Address);//address of the accelerometer
  Wire.write(0x10);//reset the accelerometer
  Wire.write(0xB6);//default range 2G: 1G = 4096 unit.
  Wire.endTransmission();
  delay(5);
  Wire.beginTransmission(BMA180_Address); // address of the accelerometer
  Wire.write(0x0D); //ctrl_reg0 // low pass filter, range settings
  Wire.write(1<<4);//0x10  1<<4 value: set bit ee_w to 1 to enable writing
  Wire.endTransmission();
  delay(5);
  Wire.beginTransmission(BMA180_Address); // address of the accelerometer
  Wire.write(0x20);//bw_tcs
  Wire.write(0x0A);// low pass filter to 10 Hz = 00001010
  Wire.endTransmission();
  delay(5);
  Wire.beginTransmission(BMA180_Address);// address of the accelerometer
  Wire.write(0x35);
  Wire.write(0x05);// range +/- 2g =  0000 010 1=0x05  //data & 0xF1
  Wire.endTransmission();
}
void AccelerometerRead()
{
  Wire.beginTransmission(BMA180_Address); // address of the accelerometer
  Wire.write(0x02); // set read pointer to data
  Wire.endTransmission();
  Wire.requestFrom(BMA180_Address, 6);
  
  // read in the 3 axis data, each one is 16 bits
  int i = 0;
  byte result[6];
  while(Wire.available())    
  { 
    result[i] = Wire.read(); 
    i++;
  }
  Wire.endTransmission();   
  AccY = (((result[1] << 8) | result[0]) + 1420)*9.81/16593.46;//*9.81/16384  + 1120 /16693.46
  AccX = (((result[3] << 8) | result[2]) + 5)*9.81/16429.48;//*9.81/16384     + 145 /16529.48
  AccZ = (((result[5] << 8) | result[4]) - 459)*9.81/16124.26;//*9.81/16384   - 359 - 439 /16124.26

  //AccY = ((result[1] << 8) | result[0]);//-17873.46 15513.43
  //AccX = ((result[3] << 8) | result[2]);//-16629.48 16429.96
  //AccZ = ((result[5] << 8) | result[4]);//-15865.41 16943.26
  //GyroTemp = (buff[0] << 8) | buff[1]; // temperature     
}
void GyroITGInit()
{
   Wire.beginTransmission(ITG3200_Address); 
   Wire.write(0x3E);  //PWR_MGM register: Power Management
   Wire.write(0x80);   //0x80 send a reset to the device
   Wire.endTransmission(); 
   delay(5);
   Wire.beginTransmission(ITG3200_Address);// default value = 0: OK
   Wire.write(0x15);  //SMPLRT_DIV
   Wire.write(0x00);    //sample rate divider 0x00=1kHz, 0x07 = 125Hz  Default settings LPF 256Hz/8000Hz sample
   Wire.endTransmission(); 
   delay(5);
   Wire.beginTransmission(ITG3200_Address); //Digital Low Pass Filter/ Full Scale range 
   Wire.write(0x16);  //DLPF_FS- FS_SEL- DLPF_CFG ,low pass filter configuration
   Wire.write(0x18);   // +/- 2000 dgrs/sec, Low Pass Filter Bandwidth (10 HZ 1KHz=1D), 1E, 19 //0x18 =max
   Wire.endTransmission(); 
   delay(5);
   //Wire.beginTransmission(ITG3200_Address); 
   //Wire.write(0x17);  //Interrupt: Configuration
   //Wire.write(0x00);    // enable send raw values
   //Wire.endTransmission(); 
  // i2c_writeReg(ITG3200_ADDRESS, 0x3E, 0x03); //register: Power Management  --  value: PLL with Z Gyro reference
   Wire.beginTransmission(ITG3200_Address); 
   Wire.write(0x3E);  //register: Power Management
   Wire.write(0x03);    // value: PLL with Z Gyro reference
   Wire.endTransmission(); 
   delay(5);
}
void GyroRead()
{
  Wire.beginTransmission(0x68); // address of the gyro
  Wire.write(0x1B); // set read pointer
  Wire.endTransmission();
  Wire.requestFrom(0x68, 8);
  int i = 0;
  byte buff[8];
  while(Wire.available())    
  { 
    buff[i] = Wire.read(); 
    i++;
  }
  Wire.endTransmission();   
  GyroY = (((buff[2] << 8) | buff[3])/14.175)/RAD_TO_DEG;//*1/14.375
  GyroX = (((buff[4] << 8) | buff[5])/14.175)/RAD_TO_DEG;//13.375
  GyroZ = (((buff[6] << 8) | buff[7])*-1.0/14.175)/RAD_TO_DEG;
  GyroTemp = (((buff[0] << 8) | buff[1]) + 22000)/280.0; // temperature -13200  280 LSB/ºC
}

void MagHMCInt()
{
  Wire.beginTransmission(HMC5883_Address); //open communication with HMC5883
  Wire.write(0x00); //Configuration Register A
  Wire.write(0x70); //num samples: 8 ; output rate: 15Hz ; normal measurement mode
  Wire.endTransmission();
  Wire.beginTransmission(HMC5883_Address); //open communication with HMC5883
  Wire.write(0x01); //Configuration Register B
  Wire.write(0x20); //configuration gain 1.3Ga
  Wire.endTransmission();
  //Put the HMC5883 IC into the correct operating mode
  Wire.beginTransmission(HMC5883_Address); //open communication with HMC5883
  Wire.write(0x02); //select mode register
  Wire.write(0x00); //continuous measurement mode
  Wire.endTransmission();
}

void MagRead()
{
  //Tell the HMC5883 where to begin reading data
  Wire.beginTransmission(HMC5883_Address);
  Wire.write(0x03); //select register 3, X MSB register
  Wire.endTransmission();
 //Read data from each axis, 2 registers per axis
  Wire.requestFrom(HMC5883_Address, 6);
 int i = 0;
  byte result[6];
  while(Wire.available())    
  { 
    result[i] = Wire.read(); 
    i++;
  }
  Wire.endTransmission();   
  MagY = (((result[0] << 8) | result[1]));//offset + 1.05
  MagZ = (((result[2] << 8) | result[3])*-1);// + 0.05
  MagX = (((result[4] << 8) | result[5]));// - 0.55
  
 // adjust for  compass axis offsets/sensitivity differences by scaling to +/-5 range
  c_magnetom_x = ((float)(MagX - M_X_MIN) / (M_X_MAX - M_X_MIN))*10.0 - 5.0;
  c_magnetom_y = ((float)(MagY - M_Y_MIN) / (M_Y_MAX - M_Y_MIN))*10.0 - 5.0;
  c_magnetom_z = ((float)(MagZ - M_Z_MIN) / (M_Z_MAX - M_Z_MIN))*10.0 - 5.0;
}
//average filter///
void sensor_readSum() {
      sensorSamples++;
      AccelerometerRead();
      GyroRead();
      //MagRead();
    gyroSumX += GyroX;
    gyroSumY += GyroY;
    gyroSumZ += GyroZ;
    AccSumX += AccX;
    AccSumY += AccY;
    AccSumZ += AccZ; 
}
void sensor_Get()
{  
  GyroX = (gyroSumX/sensorSamples) - gyro_offsetX;
  GyroY = (gyroSumY/sensorSamples) - gyro_offsetY;
  GyroZ = (gyroSumZ/sensorSamples) - gyro_offsetZ;
  //applyDeadband(GyroX, 0.01);//0.02 deg/s
  
  AccX = (AccSumX/sensorSamples) - acc_offsetX;
  AccY = (AccSumY/sensorSamples) - acc_offsetY;
  AccZ = (AccSumZ/sensorSamples);// - acc_offsetZ
  
  gyroSumX = 0.0;
  gyroSumY = 0.0;
  gyroSumZ = 0.0;
  AccSumX = 0.0;
  AccSumY = 0.0;
  AccSumZ = 0.0;
  sensorSamples2 = sensorSamples;
  sensorSamples=0.0;
}
void sensor_Calibrate()
{
  Serial.print("Sensor_Calibrate");Serial.println("\t");
    for (uint8_t i=0; i<40; i++) //Collect 40 samples
    {
        Serial.print("- ");
        sensor_readSum();        
        digitalWrite(30, HIGH);
        delay(20);
        digitalWrite(30, LOW);
        delay(20);
    }
    Serial.println("- ");
    gyro_offsetX = gyroSumX/sensorSamples;
    gyro_offsetY = gyroSumY/sensorSamples;
    gyro_offsetZ = gyroSumZ/sensorSamples;
    acc_offsetX = AccSumX/sensorSamples;
    acc_offsetY = AccSumY/sensorSamples;
    acc_offsetZ = AccSumZ/sensorSamples;
    
    sensorSamples = 0.0;
    Serial.print("GYRO_Calibrate");Serial.print("\t");
    Serial.print(gyro_offsetX);Serial.print("\t");//-0.13
    Serial.print(gyro_offsetY);Serial.print("\t");//-0.10
    Serial.print(gyro_offsetZ);Serial.println("\t");//0.03 
    Serial.print("ACC_Calibrate");Serial.print("\t");
    Serial.print(acc_offsetX);Serial.print("\t");
    Serial.print(acc_offsetY);Serial.print("\t");
    Serial.print(acc_offsetZ);Serial.println("\t"); 
    //gyro_offsetX = -0.03;//-0.03
    //gyro_offsetY = -0.10;//-0.10
    //gyro_offsetZ = 0.01;//0.00
    acc_offsetX = -0.14;//-0.14     Trim PITCH CONTROL   -10.07	-10.55	-9.82
    acc_offsetY = 0.38;//0.38   -0.08 Trim ROLL CONTROL     10.39	9.74	11
    //acc_offsetZ = 0.0;//0.245 0.235 10.2
}
void Mag_Calibrate()//Calibration_sensor Magnetometer
{
  Serial.print("Mag_Calibrate");Serial.println("\t");
    // Output MIN/MAX values
    M_X_MIN = 0;
    M_Y_MIN = 0;
    M_Z_MIN = 0;
    M_X_MAX = 0;
    M_Y_MAX = 0;
    M_Z_MAX = 0; 
    Serial1.print("magn x,y,z (min/max) = ");
    for (int i = 0; i < 600; i++) {//Calibration 30 s
      digitalWrite(30, HIGH);
      Serial.print("- ");
      MagRead();
      if (MagX < M_X_MIN) M_X_MIN = MagX;
      if (MagX > M_X_MAX) M_X_MAX = MagX;
      if (MagY < M_Y_MIN) M_Y_MIN = MagY;
      if (MagY > M_Y_MAX) M_Y_MAX = MagY;
      if (MagZ < M_Z_MIN) M_Z_MIN = MagZ;
      if (MagZ > M_Z_MAX) M_Z_MAX = MagZ;
      delay(25);
      digitalWrite(30, LOW);
      delay(25);
    }
      Serial.println("- ");
      Serial1.print(M_X_MIN);Serial1.print("/");
      Serial1.print(M_X_MAX);Serial1.print("\t");
      Serial1.print(M_Y_MIN);Serial1.print("/");
      Serial1.print(M_Y_MAX);Serial1.print("\t");
      Serial1.print(M_Z_MIN);Serial1.print("/");
      Serial1.print(M_Z_MAX);
      Serial1.print("\n");
}
  /********************************************************************/
  /****      Accelerometers trim     Remote Trim By tinnakon   ****/
//With the help of your roll and pitch stick you could now trim the ACC mode.
//You must first put the throttle stick in maximal position. (obviously with motors disarmed)
//full PITCH forward/backward and full ROLL left/right (2 axis possibilities) will trim the level 
//mode according to the neutral angle you want to change.
//The status LED will blink to confirm each ticks.
  void Remote_TrimACC() {
     if(CH_THR > MAXCHECK && armed == 0)
    {
 ////Trim ROLL CONTROL/////////////
      if(CH_AIL > MAXCHECK)
      {
        acc_offsetY = acc_offsetY - 0.02;
           for (int i = 0; i < 5; i++)
           {
            digitalWrite(13, HIGH);
            delay(20);
            digitalWrite(13, LOW);
            delay(20);
           }
      }
      if(CH_AIL < MINCHECK)
      {
        acc_offsetY = acc_offsetY + 0.02;
           for (int i = 0; i < 5; i++)
           {
            digitalWrite(13, HIGH);
            delay(20);
            digitalWrite(13, LOW);
            delay(20);
           }
      }
 ///////Trim PITCH CONTROL//////////////////////
         if(CH_ELE > MAXCHECK)
      {
        acc_offsetX = acc_offsetX - 0.02;
           for(int i = 0; i < 5; i++)
           {
            digitalWrite(13, HIGH);
            delay(20);
            digitalWrite(13, LOW);
            delay(20);
           }
      }
      if(CH_ELE < MINCHECK)
      {
        acc_offsetX = acc_offsetX + 0.02;
           for(int i = 0; i < 5; i++)
           {
            digitalWrite(13, HIGH);
            delay(20);
            digitalWrite(13, LOW);
            delay(20);
           }
      }
    }//end CH_THR > MAXCHECK && armed == 0
  }
  
