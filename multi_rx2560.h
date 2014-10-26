//#include "multi_rx2560.h"
int CH_THR;
int CH_AIL;
int CH_ELE;
int CH_RUD;
float CH_AILf = 1500;
float CH_ELEf = 1500;
float CH_RUDf = 1500;
int CH_AIL_Cal = 1500;
int CH_ELE_Cal = 1500;
int CH_RUD_Cal = 1500;
int AUX_1;
int AUX_2;
byte Fail_SafeON = 0;
unsigned int countFail_Safe = 0;

#define MINTHROTTLE 1050 
//#define MAXTHROTTLE 1850
#define MINCOMMAND 1000
#define MAXCOMMAND 1900

#define MIDRC 1500
#define MINCHECK 1100
#define MAXCHECK 1900

//RX PIN assignment inside the port //for PORTK
#define THROTTLEPIN                0  //PIN 62 =  PIN A8
#define ROLLPIN                    1  //PIN 63 =  PIN A9
#define PITCHPIN                   2  //PIN 64 =  PIN A10
#define YAWPIN                     3  //PIN 65 =  PIN A11
#define AUX1PIN                    4  //PIN 66 =  PIN A12
#define AUX2PIN                    5  //PIN 67 =  PIN A13
#define CAM1PIN                    6  //PIN 68 =  PIN A14
#define CAM2PIN                    7  //PIN 69 =  PIN A15

#define ROLL       0
#define PITCH      1
#define YAW        2
#define THROTTLE   3
#define AUX1       4
#define AUX2       5
#define CAMPITCH   6
#define CAMROLL    7      

static uint8_t pinRcChannel[8] = {ROLLPIN, PITCHPIN, YAWPIN, THROTTLEPIN, AUX1PIN,AUX2PIN,CAM1PIN,CAM2PIN};
volatile uint16_t rcPinValue[8] = {1500,1500,1500,1500,1500,1500,1500,1500};
static int16_t rcData[8] ;
static int16_t rcHysteresis[8] ;
static int16_t rcData4Values[8][4];

void configureReceiver() {
    for (uint8_t chan = 0; chan < 8; chan++){
      for (uint8_t a = 0; a < 4; a++){
        rcData4Values[chan][a] = 1500;
      }
    }
      DDRK = 0; 
      PORTK   = (1<<0) | (1<<1) | (1<<2) | (1<<3) | (1<<4) | (1<<5) | (1<<6) | (1<<7); 
      PCMSK2 |= (1<<0) | (1<<1) | (1<<2) | (1<<3) | (1<<4) | (1<<5) | (1<<6) | (1<<7);
      PCICR   = 1<<2;
}

ISR(PCINT2_vect) {
  uint8_t mask;
  uint8_t pin;
  uint16_t cTime,dTime;
  static uint16_t edgeTime[8];
  static uint8_t PCintLast;
  pin = PINK;            
  mask = pin ^ PCintLast;   
  sei();                   
  PCintLast = pin;     
  cTime = micros();        
  if (mask & 1<<2){          
    if (!(pin & 1<<2)) {    
      dTime = cTime-edgeTime[2]; if (900<dTime && dTime<2200) rcPinValue[2] = dTime; 
    } else edgeTime[2] = cTime; 
  }  
  if (mask & 1<<4){   
    if (!(pin & 1<<4)) {
      dTime = cTime-edgeTime[4]; if (900<dTime && dTime<2200) rcPinValue[4] = dTime;
    } else edgeTime[4] = cTime;
  }
  if (mask & 1<<5){
    if (!(pin & 1<<5)) {
      dTime = cTime-edgeTime[5]; if (900<dTime && dTime<2200) rcPinValue[5] = dTime;
    } else edgeTime[5] = cTime;
  }
  if (mask & 1<<6){
    if (!(pin & 1<<6)) {
      dTime = cTime-edgeTime[6]; if (900<dTime && dTime<2200) rcPinValue[6] = dTime;
    } else edgeTime[6] = cTime;
  }
  if (mask & 1<<7){
    if (!(pin & 1<<7)) {
      dTime = cTime-edgeTime[7]; if (900<dTime && dTime<2200) rcPinValue[7] = dTime;
    } else edgeTime[7] = cTime;
  }
  if (mask & 1<<0){    
    if (!(pin & 1<<0)) {
      dTime = cTime-edgeTime[0]; if (900<dTime && dTime<2200) rcPinValue[0] = dTime; 
    } else edgeTime[0] = cTime; 
  }
  if (mask & 1<<1){      
    if (!(pin & 1<<1)) {
      dTime = cTime-edgeTime[1]; if (900<dTime && dTime<2200) rcPinValue[1] = dTime; 
    } else edgeTime[1] = cTime;
  }
  if (mask & 1<<3){
    if (!(pin & 1<<3)) {
      dTime = cTime-edgeTime[3]; if (900<dTime && dTime<2200) rcPinValue[3] = dTime;
    } else edgeTime[3] = cTime;
  }
 countFail_Safe = 0;
}

uint16_t readRawRC(uint8_t chan) {
  uint16_t data;
  uint8_t oldSREG;
  oldSREG = SREG;
  data = rcPinValue[pinRcChannel[chan]];
  SREG = oldSREG;
  return data; 
}
  
void computeRC() {
  static uint8_t rc4ValuesIndex = 0;
  uint8_t chan,a;
  rc4ValuesIndex++;
  for (chan = 0; chan < 8; chan++) {
    rcData4Values[chan][rc4ValuesIndex%4] = readRawRC(chan);
    rcData[chan] = 0;
    for (a = 0; a < 4; a++){
      rcData[chan] += rcData4Values[chan][a];
    }
    rcData[chan]= (rcData[chan]+2)/4;
    if ( rcData[chan] < rcHysteresis[chan] -3)  rcHysteresis[chan] = rcData[chan]+2;
    if ( rcData[chan] > rcHysteresis[chan] +3)  rcHysteresis[chan] = rcData[chan]-2;
  }
    CH_THR = rcHysteresis[THROTTLE];
    CH_AIL = rcHysteresis[ROLL];
    CH_ELE = rcHysteresis[PITCH];
    CH_RUD = rcHysteresis[YAW];
    AUX_1 = rcHysteresis[AUX1];
    AUX_2 = rcHysteresis[AUX2];
    CH_AILf = CH_AILf + (CH_AIL - CH_AILf)*0.02/tarremote;
    CH_ELEf = CH_ELEf + (CH_ELE - CH_ELEf)*0.02/tarremote;
    CH_RUDf = CH_RUDf + (CH_RUD - CH_RUDf)*0.02/tarremote;
}
//By tinnakon
void Fail_Safe(){
    countFail_Safe++;
    if(countFail_Safe > 3)
    {
     countFail_Safe = 3;
     Fail_SafeON = 1;
     if(Status_LED == LOW)
     Status_LED = HIGH;
     else
     Status_LED = LOW;
     digitalWrite(13, Status_LED);
     digitalWrite(30, Status_LED);
     digitalWrite(31, Status_LED);
    }
    else
    {
      Fail_SafeON = 0;
    }
  }
  void RC_Calibrate(){
  Serial.print("RC_Calibrate");Serial.println("\t");
  for (int i = 0; i < 10; i++) {
    computeRC();
    delay(20);
  }
  CH_AIL_Cal = CH_AIL;
  CH_ELE_Cal = CH_ELE;
  CH_RUD_Cal = CH_RUD;
    Serial.print(CH_AIL_Cal);Serial.print("\t");//-0.13
    Serial.print(CH_ELE_Cal);Serial.print("\t");//-0.10
    Serial.print(CH_RUD_Cal);Serial.println("\t");//0.03 
}
