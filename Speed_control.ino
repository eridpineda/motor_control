#include "DueTimer.h"

#define COUNTS_PER_ROTATION 211.2
#define pi 3.1416

//**********************************
int count_q = 0;      // quadrature decoder hardware counts
int spd = 10; // speed
int dir = 8;  // direction 'High' - negative counts (CCW), "LOW" - positive Counts (CW)
int pos = LOW;
//*****************Intializing*****************
float count_q_prev = 0;
float V_prev = 0;
float V_prevprev = 0;
float rad_err_prev = 0;
float Prev_rad_rev;
float Spd_Err_prev = 0;
float Spd_Err_prevprev = 0;

float rad_Input;
float rad_Output;
float rad_Err;
float V_req_pwm;
float rad_rev;
float Spd;
float Spd_Input;
float Spd_Output;
float Spd_Err;

//*****************Voltage*****************
//Maximum motor voltage
float V_max = 12;     
//Minimum motor voltage
float V_min = 0;
float V = 0;         // set initial voltage to zero

void setup() {

  Serial.begin(9600); //115200 or 9600
  analogWriteResolution(12);    // set DAC output to maximum 12-bits
  Timer3.attachInterrupt(update).start(10000); // start ISR timer3 (not used by quad decoder and PWM) at 1 ms, which is 1kHz
  pinMode(spd, OUTPUT);
  pinMode(dir, OUTPUT);
  
  // setup for encoder position measurement (Digital pins 2 and 13)
  // This is described in Chapter 36 of the SAM3X8E datasheet
  
    REG_PMC_PCER0 = PMC_PCER0_PID27;   
    REG_TC0_CMR0 = TC_CMR_TCCLKS_XC0;  

    REG_TC0_BMR = TC_BMR_QDEN
                | TC_BMR_POSEN
                | TC_BMR_EDGPHA; 

    REG_TC0_CCR0 = TC_CCR_CLKEN
                 | TC_CCR_SWTRG;
}

void loop() {
 count_q = REG_TC0_CV0; 
 rad_rev = (2*pi*(count_q))/COUNTS_PER_ROTATION; // [rad]
 Spd_Output = (rad_rev - Prev_rad_rev)/(0.01); //[rad/s]
 Prev_rad_rev = rad_rev;

 Serial.println(Spd_Output);
 // analogWrite(DAC0, Spd_Output)
 
}

void update() {

  count_q = REG_TC0_CV0; 
  rad_rev = (2*pi*(count_q))/COUNTS_PER_ROTATION; // [rad]

  Spd_Input = 50; //[rad/s]
  Spd_Output = (rad_rev - Prev_rad_rev)/(0.01); //[rad/s]
  
  Spd_Err = Spd_Input - Spd_Output;

  // Controller/Difference Equation
 // V = Spd_Err - 0.9999*Spd_Err_prev + 1*V_prev; // Difference equation kp =1, ki=0.1
 // V = Spd_Err - 0.9995*Spd_Err_prev + 1*V_prev; // Difference equation kp =1, ki=0.1
 
 // Used tau - pole after design 
  V = 0.8337*Spd_Err + 0.0008333*Spd_Err_prev - 0.8329*Spd_Err_prevprev + 0.3333*V_prev + 0.6667*V_prevprev; // Difference equation kp =1, ki=0.1

  Spd_Err_prev = Spd_Err;
  Spd_Err_prevprev = Spd_Err_prev;
  Prev_rad_rev = rad_rev;
  V_prev = V; 
  V_prevprev = V_prev;

  pos  = LOW; //  Positive counter values

if (V < V_min) {
      pos  = HIGH; // if input voltage is negative flip direction
    }

  digitalWrite(dir, pos); // direction can change dependent on V 

  V_req_pwm = fabs(V)*(4095/12); // Voltage must remain postive for pwm 

  if (V_req_pwm> 4095) 
  {
    V_req_pwm = 4095;
  }

  analogWrite(spd, V_req_pwm);
 
  
  //Serial.println(Spd_Output);

}
