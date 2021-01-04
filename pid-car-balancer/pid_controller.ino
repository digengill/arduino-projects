#include <AFMotor.h>
#include <Servo.h>
#include <NewPing.h>
Servo s1; 
NewPing sonar(22,24,35);

float kp = 4, kd = 0.4, ki = 0.8; //PID constants
float target = 17; //center of balance
float error, errorOld, errorD, errorI, totalError;
float t1, t2, dt;
float dist;

void setup() {
  // setup
  s1.attach(9); //attach servo
  s1.write(70); //balanced value
  dist = sonar.ping_cm();
  errorOld = 0;
  error = 0;
  t1 = 0;
  Serial.begin(9600);
}

void loop() {
  dist = sonar.ping_cm();
  t2 = millis();
  if (dist != 0 && dist < 31)
  {
    error = target - dist; 
    if (abs(error) > 0.5)
    {
    dt = t2 - t1;
    dt = dt / 1000;
    t1 = t2;

    errorD =  kd * ((error - errorOld) / dt); //derivative    
    errorI += ki *error * dt; // integral
    errorOld = error;
   
    totalError = kp*error + errorD + errorI + 80;
    //totalError = errorD +90;
    if (totalError < 10)
    {
        totalError = 10;
      }
      else if (totalError > 150)
      {
        totalError = 150;
        }
    s1.write(int(totalError)); //write to servo
    }
  }
  delay(100);
  
}
