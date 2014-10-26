//#include "motorX4.h"
int MOTOR_FrontL_PIN = 2;
int MOTOR_FrontR_PIN = 5;
int MOTOR_BackL_PIN = 6;
int MOTOR_BackR_PIN = 3;

#define PWM_FREQUENCY 400   //400 in Hz
#define PWM_PRESCALER 8
#define PWM_COUNTER_PERIOD (F_CPU/PWM_PRESCALER/PWM_FREQUENCY)

int motor_FrontL = 1000;
int motor_FrontR = 1000;
int motor_BackL = 1000;
int motor_BackR = 1000;

void motor_command_all() 
{
  for (int j = 0 ; j <= 50 ; j++)
  {
    motor_FrontL = 1000;
    motor_FrontR = 1000;
    motor_BackL = 1000;
    motor_BackR = 1000;
  //analogWrite(MOTOR_Front_PIN, 125);//125 = 1000 ms , 490 Hz
  //analogWrite(MOTOR_Right_PIN, 125);//250 = 2000 ms
  //analogWrite(MOTOR_Left_PIN, 125);
  //analogWrite(MOTOR_Back_PIN, 125);
  OCR3B = motor_FrontL*2; //  pin 2
  OCR3C = motor_BackR*2; //  pin 3  <<3
  OCR3A = motor_FrontR*2; //  pin 5
  OCR4A = motor_BackL*2; //  pin 6
  delay(20);
}
}
void motor_initialize() 
{
  pinMode(MOTOR_FrontL_PIN,OUTPUT);  
  pinMode(MOTOR_FrontR_PIN,OUTPUT); 
  pinMode(MOTOR_BackL_PIN,OUTPUT); 
  pinMode(MOTOR_BackR_PIN,OUTPUT); 
// init 16bit timer 3 //connect pin 3 to timer 3 channel C // connect pin 5 to timer 3 channel A
                      // connect pin 2 to timer 3 channel B
    // Init PWM Timer 3                                       // WGMn1 WGMn2 WGMn3  = Mode 14 Fast PWM, TOP = ICRn ,Update of OCRnx at BOTTOM
    TCCR3A = (1<<WGM31)|(1<<COM3A1)|(1<<COM3B1)|(1<<COM3C1);  // Clear OCnA/OCnB/OCnC on compare match, set OCnA/OCnB/OCnC at BOTTOM (non-inverting mode)
    TCCR3B = (1<<WGM33)|(1<<WGM32)|(1<<CS31);                 // Prescaler set to 8, that gives us a resolution of 0.5us
    ICR3 = PWM_COUNTER_PERIOD;                                // Clock_speed / ( Prescaler * desired_PWM_Frequency) #defined above.
          
       // init 16bit timer 4  // connect pin 6 to timer 4 channel A
       // Init PWM Timer 4
      TCCR4A = (1<<WGM41)|(1<<COM4A1);
      TCCR4B = (1<<WGM43)|(1<<WGM42)|(1<<CS41);
      ICR4 = PWM_COUNTER_PERIOD;
            // Init PWM Timer 4
      //TCCR4A = (1<<WGM41)|(1<<COM4A1)|(1<<COM4B1)|(1<<COM4C1);
      //TCCR4B = (1<<WGM43)|(1<<WGM42)|(1<<CS41);
      //ICR4 = PWM_COUNTER_PERIOD;
  motor_command_all();
}

//motor command
void motor_command() 
{
  //analogWrite(MOTOR_Front_PIN, motor_Front/8);
  //analogWrite(MOTOR_Right_PIN, motor_Right/8);
  //analogWrite(MOTOR_Left_PIN, motor_Left/8);
  //analogWrite(MOTOR_Back_PIN, motor_Back/8);
  OCR3B = motor_FrontL*2; //  pin 2
  OCR3C = motor_BackR*2; //  pin 3  <<3
  OCR3A = motor_FrontR*2; //  pin 5
  OCR4A = motor_BackL*2; //  pin 6
}
  /********************************************************************/
  /****           ESCs calibration                                 ****/
  /********************************************************************/
void ESC_calibration () {
   for (int i = 0; i < 5; i++)
  {
    computeRC();
    if(CH_THR > MAXCHECK)
    {
     ESC_calibra = 1; 
    }
    else
    {
     ESC_calibra = 0;
    }
   delay(20);
  }
  int jprint = 0;
  while(ESC_calibra == 1){
   computeRC();
   motor_FrontL = (CH_THR - 500)*1.5;
   motor_FrontR = (CH_THR - 500)*1.5;
   motor_BackL = (CH_THR - 500)*1.5;
   motor_BackR = (CH_THR - 500)*1.5;
   
   motor_FrontL = constrain(motor_FrontL, MINCOMMAND, MAXCOMMAND);
   motor_FrontR = constrain(motor_FrontR, MINCOMMAND, MAXCOMMAND);
   motor_BackL = constrain(motor_BackL, MINCOMMAND, MAXCOMMAND);
   motor_BackR = constrain(motor_BackR, MINCOMMAND, MAXCOMMAND);
   
  OCR3B = motor_FrontL*2; //  pin 2
  OCR3C = motor_BackR*2; //  pin 3  <<3
  OCR3A = motor_FrontR*2; //  pin 5
  OCR4A = motor_BackL*2; //  pin 6
   //analogWrite(MOTOR_Front_PIN, motor_Front/8);
   //analogWrite(MOTOR_Right_PIN, motor_Right/8);
   //analogWrite(MOTOR_Left_PIN, motor_Left/8);
   //analogWrite(MOTOR_Back_PIN, motor_Back/8);
   
   jprint++;
   if(jprint > 10)
   {
     jprint = 0;
   Serial.print(motor_FrontL);Serial.print("\t");
   Serial.print(motor_FrontR);Serial.print("\t");    
   Serial.print(motor_BackL);Serial.print("\t");     
   Serial.print(motor_BackR);Serial.println("\t");    
   //Serial1.println(motor_Back);
     if(Status_LED == LOW)
     Status_LED = HIGH;
     else
     Status_LED = LOW;
     digitalWrite(13, Status_LED);
     digitalWrite(30, Status_LED);
     digitalWrite(31, Status_LED);
   }
   delay(20);
  }
}

void motor_Mix(){
#ifdef Quad_X
      motor_FrontL = uAltitude + u_pitch*0.7 + u_roll*0.7 - u_yaw;//Front L
      motor_FrontR = uAltitude + u_pitch*0.7 - u_roll*0.7 + u_yaw;//Front R
      motor_BackL = uAltitude - u_pitch*0.7 + u_roll*0.7 + u_yaw;//Back L
      motor_BackR = uAltitude - u_pitch*0.7 - u_roll*0.7 - u_yaw;////Back R
#endif
#ifdef Quad_P
      motor_Front = uAltitude + u_pitch - u_yaw;
      motor_Right = uAltitude - u_roll + u_yaw;
      motor_Left = uAltitude + u_roll + u_yaw;
      motor_Back = uAltitude - u_pitch - u_yaw;
#endif
       if (CH_THR < MINCHECK) 
        {
          roll_I_rate = 0;
          pitch_I_rate = 0;
          yaw_I_rate = 0;
          motor_FrontL = 1000;
          motor_FrontR = 1000;
          motor_BackL = 1000;
          motor_BackR = 1000;
        }
        if(armed == 1)
        {
         motor_FrontL = constrain(motor_FrontL, MINTHROTTLE, MAXCOMMAND);
         motor_FrontR = constrain(motor_FrontR, MINTHROTTLE, MAXCOMMAND);
         motor_BackL = constrain(motor_BackL, MINTHROTTLE, MAXCOMMAND);
         motor_BackR = constrain(motor_BackR, MINTHROTTLE, MAXCOMMAND);
        }
        else
        {
          motor_FrontL = 1000;
          motor_FrontR = 1000;
          motor_BackL = 1000;
          motor_BackR = 1000;
        }
}
