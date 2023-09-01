#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Depending on your servo make, the pulse width min and max may vary, you 
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!
#define SERVOMIN  102 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  512 // this is the 'maximum' pulse length count (out of 4096)
#define FREQ 50

// servos number
uint8_t servo1 = 0;
uint8_t servo2 = 1;
uint8_t servo3 = 2;
uint8_t servo4 = 3;
uint8_t servo5 = 4;


// functions declaration
int angleToPulse(int ang);
void motor_move(int8_t motor, int angle); // angle from -90 to +90
void motors_init(void);
void motors_home(void);

// global variables:
 String msg;
 int angle;

// setup
void setup() {
  Serial.begin(115200);
  motors_init();
  motors_home();
}

// infinite loop
void loop() {

  if (Serial.available() > 0) {
    String msg = Serial.readStringUntil(';');
    Serial.flush();
    // Split the message into four angle values separated by commas
    int angles[5];
    int angleIndex = 0;
    char *msgBuffer = new char[msg.length() + 1];
    msg.toCharArray(msgBuffer, msg.length() + 1);

    char *angleStr = strtok(msgBuffer, ",");
    while (angleStr != NULL && angleIndex < 5) {
      angles[angleIndex] = atoi(angleStr);
      angleIndex++;
      angleStr = strtok(NULL, ",");
    }
    delete[] msgBuffer;


    // Control all four servos
    motor_move(servo1, angles[0]);
    motor_move(servo2, angles[1]);
    motor_move(servo3, angles[2]);
    motor_move(servo4, angles[3]);
    motor_move(servo5, angles[4]);
    delay(12);
  }
 
}

/*
 * angleToPulse(int ang)
 * gets angle in degree and returns the pulse width
 * also prints the value on seial monitor
 */
int angleToPulse(int ang){
   int pulse = map(ang,0, 180, SERVOMIN,SERVOMAX);// map angle of 0 to 180 to Servo min and Servo max 
   return pulse;
}

void motor_move(int8_t motor, int angle)  // angle from -90 to +90
{
  if(angle>90) angle = 90;
  else if(angle<-90) angle = -90;

  if(motor == servo3 || motor == servo4)
    angle *= -1; // reverse direction

    
  // map
  int pulse = map(angle, -90, +90, SERVOMIN, SERVOMAX);

  pwm.setPWM(motor, 0, pulse);
  return;
}

void motors_init(void)
{
  pwm.begin();
  pwm.setPWMFreq(FREQ);  // Analog servos run at ~50 Hz updates
}

void motors_home(void)
{
  // homing position (straight up)
  motor_move(servo1, -3);  // + is ccw
  motor_move(servo2, -58);   // + forward
  motor_move(servo3, 90);  
  motor_move(servo4, 47); 
  delay(1000);
}