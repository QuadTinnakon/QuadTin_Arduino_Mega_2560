//#include "ahrs.h" By tinnakon 5_03_2557(2014)
#define Kp 0.12	   //0.45 0.2 0.7	// proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.0093  //0.00053 0.0005	// integral gain governs rate of convergence of gyroscope biases

float exInt , eyInt , ezInt ;	// scaled integral error
float q0, q1 , q2 , q3 ;	// quaternion elements representing the estimated orientation
float ahrs_p,ahrs_r,ahrs_y;

float previousEx = 0.0;
float previousEy = 0.0;
float previousEz = 0.0;

void ahrs_initialize()
{
  exInt=0.0;
  eyInt=0.0;
  ezInt=0.0;
  q0=1.0;
  q1=0.0;
  q2=0.0;
  q3=0.0;
}
boolean isSwitched(float previousError, float currentError) {
  if ( (previousError > 0 &&  currentError < 0) ||
	   (previousError < 0 &&  currentError > 0)) {
    return true;
  }
  return false;
}
void ahrs_updateMARG(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float G_Dt){

  float norm1,halfT;
  float hx, hy, hz, bx, bz;
  float vx, vy, vz, wx, wy, wz;
  float ex, ey, ez;
  halfT=G_Dt/2.0;
  // auxiliary variables to reduce number of repeated operations
  float q0q0 = q0*q0;
  float q0q1 = q0*q1;
  float q0q2 = q0*q2;
  float q0q3 = q0*q3;
  float q1q1 = q1*q1;
  float q1q2 = q1*q2;
  float q1q3 = q1*q3;
  float q2q2 = q2*q2;   
  float q2q3 = q2*q3;
  float q3q3 = q3*q3;          
  // normalise the measurements
  norm1 = sqrt(ax*ax + ay*ay + az*az);       
  ax = ax / norm1;
  ay = ay / norm1;
  az = az / norm1;
  norm1 = sqrt(mx*mx + my*my + mz*mz);          
  mx = mx / norm1;
  my = my / norm1;
  mz = mz / norm1;         
  // compute reference direction of flux
  hx = 2*mx*(0.5 - q2q2 - q3q3) + 2*my*(q1q2 - q0q3) + 2*mz*(q1q3 + q0q2);
  hy = 2*mx*(q1q2 + q0q3) + 2*my*(0.5 - q1q1 - q3q3) + 2*mz*(q2q3 - q0q1);
  hz = 2*mx*(q1q3 - q0q2) + 2*my*(q2q3 + q0q1) + 2*mz*(0.5 - q1q1 - q2q2);         
  bx = sqrt((hx*hx) + (hy*hy));
  bz = hz; 
  // estimated direction of gravity and flux (v and w)
  vx = 2*(q1q3 - q0q2);
  vy = 2*(q0q1 + q2q3);
  vz = q0q0 - q1q1 - q2q2 + q3q3;
  wx = 2*bx*(0.5 - q2q2 - q3q3) + 2*bz*(q1q3 - q0q2);
  wy = 2*bx*(q1q2 - q0q3) + 2*bz*(q0q1 + q2q3);
  wz = 2*bx*(q0q2 + q1q3) + 2*bz*(0.5 - q1q1 - q2q2);  
  // error is sum of cross product between reference direction of fields and direction measured by sensors
  ex = (ay*vz - az*vy);
  ey = (az*vx - ax*vz);
  //ez = (ax*vy - ay*vx);
  ez = (ax*vy - ay*vx) + (mx*wy - my*wx);

  // integral error scaled integral gain
  exInt += ex*Ki;
    if (isSwitched(previousEx,ex)) {
    exInt = 0.0;
  }
  previousEx = ex;
  eyInt += ey*Ki;
    if (isSwitched(previousEy,ey)) {
    eyInt = 0.0;
  }
  previousEy = ey;
    ezInt += ez*Ki;
    if (isSwitched(previousEz,ez)) {
    ezInt = 0.0;
  }
  previousEz = ez;
  
  // adjusted gyroscope measurements
  gx +=Kp*ex + exInt;
  gy +=Kp*ey + eyInt;
  gz +=Kp*ez + ezInt;

  // integrate quaternion rate and normalise
  q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
  q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
  q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
  q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;  
  // normalise quaternion
  norm1 = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  q0 = q0 / norm1;
  q1 = q1 / norm1;
  q2 = q2 / norm1;
  q3 = q3 / norm1;
  //direction cosine matrix (DCM), Rotation matrix , Rotated Frame to Stationary Frame ZYX
  DCM00 = 2*(0.5 - q2q2 - q3q3);//q0q0 - q1q1 - q2q2 + q3q3
  DCM01 = 2*(q1q2 - q0q3);//2*(q0q1 + q2q3)
  DCM02 = 2*(q1q3 + q0q2);//2*(q1q3 - q0q2); 2*(q0q2 - q1q3)
  DCM10 = 2*(q1q2 + q0q3);
  DCM11 = 2*(0.5 - q1q1 - q3q3);
  DCM12 = 2*(q2q3 - q0q1);
  DCM20 = 2*(q1q3 - q0q2);//-sin pitch
  DCM21 = 2*(q2q3 + q0q1);
  DCM22 = 2*(0.5 - q1q1 - q2q2);
  //ahrs_toEuler();
  ahrs_y=atan2(DCM10, DCM00);//2*q0*q0+2*q1*q1-1)
  ahrs_p=-asin(DCM20); // theta
  //ahrs_p=acos(22);//http://en.wikipedia.org/wiki/Rotation_formalisms_in_three_dimensions
  ahrs_r=atan2(DCM21, DCM22); // phi
  
    cos_rollcos_pitch = DCM22;
    if(cos_rollcos_pitch <= 0.5)//45 deg = 0.5
    {
      cos_rollcos_pitch = 0.5;
    }
   
//accrX_cutg = AccXf*vx + AccYf*vy - AccZf*vz;
accrX_Earth = (AccXf*DCM00 + AccYf*DCM01 + AccZf*DCM02);
//accrX_cutg = (ax*DCM00 + ay*DCM01 + az*DCM02)*9.81;
//accrX_cutg = DCM00;
//accrY_cutg = -AccXf*vx - AccYf*vy + AccZf*vz;
accrY_Earth = (AccXf*DCM10 + AccYf*DCM11 + AccZf*DCM12);
//accrY_cutg = (ax*DCM10 + ay*DCM11 + az*DCM12)*9.81;
//accrY_cutg = DCM11;
//accrZ_cutg = (-AccXf*vx - AccYf*vy + AccZf*vz) - 9.78;//- 9.37
accrZ_Earthf = (AccXf*DCM20 + AccYf*DCM21 + AccZf*DCM22) - acc_offsetZ;
//accrZ_cutg = (ax*DCM20 + ay*DCM21 + az*DCM22)*9.81;
//accrZ_cutg = DCM22;
accrZ_Earth = accrZ_Earth + (accrZ_Earthf - accrZ_Earth)*G_Dt*32.5;//18.5 Low pass filter ,smoothing factor  α := dt / (RC + dt)
applyDeadband(accrZ_Earth, 0.08);//+-0.02 m/s^2
}
