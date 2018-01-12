#include <PID_v1.h> #uses PID implementation by br3ttb, found here: https://github.com/br3ttb/Arduino-PID-Library/blob/master/PID_v1.h

#define InA1            30                      // INA motor pin
#define InB1            33                      // INB motor pin 
#define PWM1            8                       // PWM motor pin
#define encodPinA1      2                       // encoder A pin
#define encodPinB1      3                       // encoder B pin

#define LOOPTIME        100                     // PID loop time

unsigned long lastMilli = 0;                    // loop timing 
unsigned long lastMilliPrint = 0;               // loop timing
double degree_req = 360;                            // degrees (Set Point)
double degree_act = 0;                               // degrees (actual value)
double step_req = degree_req*0.556;
double step_act = degree_act*0.556;
double PWM_val = 0;                                
volatile long count = 0;                        // rev counter

double consKp=1, consKi=0.08, consKd=0.25;

PID myPID(&step_act, &PWM_val, &step_req, consKp, consKi, consKd, DIRECT);

//200 steps correspond to 360 degrees
//1 degree corresponds to 0.556 steps

void setup() {
 Serial.begin(115200);
 pinMode(InA1, OUTPUT);
 pinMode(InB1, OUTPUT);
 pinMode(PWM1, OUTPUT);
 pinMode(encodPinA1, INPUT); 
 pinMode(encodPinB1, INPUT); 
 attachInterrupt(0, doEncoder, CHANGE);
 TCCR4B = TCCR4B & B11111000 | B00000101;    // set timer 4 divisor to  1024 for PWM frequency of    30.64 Hz

 
   digitalWrite(InA1, LOW);
   digitalWrite(InB1, HIGH);
   
 analogWrite(PWM1, PWM_val);
 myPID.SetMode(AUTOMATIC);

}

void loop() {
 double step_req = degree_req*0.556;
 double step_act = degree_act*0.556;
 if((millis()-lastMilli) >= LOOPTIME)   {                                    // enter timed loop
   lastMilli = millis();
   getMotorData();                                                           // calculate angle
   myPID.Compute();
   analogWrite(PWM1, PWM_val);
 }   
 printMotorInfo();                                                           // display data
}

void getMotorData()  {                                                        // calculate degrees
static long countAnt = 0;                                                   // last count
 step_act = count;                  
}

void printMotorInfo()  {                                                      // display data
 if((millis()-lastMilliPrint) >= 500)   {  
   double degree_act = step_act/0.556;   
   lastMilliPrint = millis();
   Serial.print("Set point:");             Serial.println(degree_req);  
   Serial.print("Current degree:");        Serial.println(degree_act);
   Serial.print("PWM:");                   Serial.println(PWM_val);
   Serial.print("\n");   
 }
}

void doEncoder() {
  /* If pinA and pinB are both high or both low, it is spinning
   * forward. If they're different, it's going backward.
   */
  if (digitalRead(encodPinA1) == digitalRead(encodPinB1)) {
    count++;
  } else {
    count--;
  }
}

void halt()
{
  while(1);
}
