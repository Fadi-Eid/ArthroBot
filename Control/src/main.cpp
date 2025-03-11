#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define NUM_SERVOS 6  // Adjust according to the number of joints used
#define SERVO_MIN  102 // Minimum pulse length count
#define SERVO_MAX  550 // Maximum pulse length count
#define FREQ 50

int servo_pins[NUM_SERVOS] = {0, 1, 2, 3, 4, 5};

// Map function to convert angles to PWM values
int angleToPulse(float angle) {
    return map(angle, 0, 180, SERVO_MIN, SERVO_MAX);
}

void setup() {
    Serial.begin(115200);
    pwm.begin();
    pwm.setPWMFreq(FREQ);
    
    // Initialize servos to minimum position
    for (int i = 0; i < NUM_SERVOS; i++) {
        pwm.setPWM(servo_pins[i], 0, (SERVO_MIN+SERVO_MAX)/2);
    }
    delay(500);
}

void loop() {
    if (Serial.available() > 0) {
        String data = Serial.readStringUntil('\n');
        data.trim();
        
        float angles[NUM_SERVOS];
        int index = 0;
        char *ptr = strtok((char*)data.c_str(), ",");
        
        while (ptr != NULL && index < NUM_SERVOS) {
            angles[index] = atof(ptr);  // Convert string to float
            ptr = strtok(NULL, ",");
            index++;
        }
        
        // Apply the parsed values to servos
        if (index == NUM_SERVOS) {
            for (int i = 0; i < NUM_SERVOS; i++) {
                int pulse = angleToPulse(angles[i]);
                pwm.setPWM(servo_pins[i], 0, pulse);
            }
        }
    }
}
