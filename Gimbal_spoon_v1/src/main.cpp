#include <Arduino.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>

// put function declarations here:
Adafruit_MPU6050 mpu;
Servo servoPitch; 
Servo servoRoll;

//---VALUES TO TUNE---

const int PITCH_PIN = D0;
const int ROLL_PIN = D1;

//sets the default position of the servo 
const int PITCH_CENTER = 90;
const int ROLL_CENTER = 90;

//offsets the default motor position to account for any misalignment 
const int PITCH_OFFSET = 0; 
const int ROLL_OFFSET = 28; //currently -15 b/c this motor is slightly askew 

const float PITCH_GAIN  = 3.1;
const float ROLL_GAIN = 3.1;

const float PITCH_CURVE = 0.75; 
const float ROLL_CURVE  = 0.75;

const float GYRO_PITCH_SIGN = 1.0; 
const float GYRO_ROLL_SIGN = 1.0; 

//==============================

float filteredPitch = 0.0; 
float filteredRoll = 0.0;
unsigned long lastTime = 0; 

//so that it's sensitive at small movements but doesn't overcompensate at larger movements
float applyCurve(float angle, float gain, float curve) {
    float absAngle = fabs(angle);
    float sign = (angle >= 0.0) ? 1.0 : -1.0;
    
    return sign * pow(absAngle, curve) * gain;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(1000);

  Serial.println("Starting setup...");
  
  //just in case
  if(!mpu.begin()){
    Serial.println("Error...");
    while(1) { 
      delay(10); 
    }
  }
  Serial.println("MPU6050 Found!");

  //mpu settings 
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_184_HZ);
  
  ESP32PWM::allocateTimer(0); 
  ESP32PWM::allocateTimer(1);

  servoPitch.setPeriodHertz(50);
  servoRoll.setPeriodHertz(50);

  servoPitch.attach(PITCH_PIN, 500, 2400);
  servoRoll.attach(ROLL_PIN, 500, 2400);

  servoPitch.write(PITCH_CENTER + PITCH_OFFSET);
  servoRoll.write(ROLL_CENTER + ROLL_OFFSET);

  lastTime = micros();
  Serial.println("--Ready!--");

}

void loop() {
  // put your main code here, to run repeatedly:

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  //convert to angles (in degrees)
  // float pitchAngle = (atan2(a.acceleration.y, a.acceleration.z) * 180.0) / PI; 
  // float rollAngle = (atan2(a.acceleration.x, a.acceleration.z) * 180.0) / PI;

  // int pitchTarget = constrain(PITCH_CENTER + pitchAngle, 20, 160);
  // int rollTarget = constrain(ROLL_CENTER - rollAngle, 20, 160);

  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;

  float accelPitch = atan2(a.acceleration.y, hypotf(a.acceleration.x, a.acceleration.z)) * 180.0 / PI;
  float accelRoll = atan2(-a.acceleration.x, hypotf(a.acceleration.y, a.acceleration.z)) * 180.0 / PI;

  float gyroPitchRate = g.gyro.x * 180.0 / PI * GYRO_PITCH_SIGN;
  float gyroRollRate = g.gyro.y * 180.0 / PI * GYRO_ROLL_SIGN;

  filteredPitch = 0.98 * (filteredPitch + gyroPitchRate * dt) + 0.02 * accelPitch;
  filteredRoll = 0.98 * (filteredRoll + gyroRollRate * dt) + 0.02 * accelRoll;

  float pitchCorrection = applyCurve(filteredPitch, PITCH_GAIN, PITCH_CURVE);
  float rollCorrection  = applyCurve(filteredRoll, ROLL_GAIN, ROLL_CURVE);

  int pitchTarget = PITCH_CENTER + PITCH_OFFSET + pitchCorrection;
  int rollTarget = ROLL_CENTER + ROLL_OFFSET - rollCorrection;

  pitchTarget = constrain(pitchTarget, 15, 165);
  rollTarget = constrain(rollTarget, 15, 165);

  servoPitch.write(pitchTarget);
  servoRoll.write(rollTarget);

  Serial.print("Filtered Pitch: ");
  Serial.print(filteredPitch);
  Serial.print(" | Filtered Roll: ");
  Serial.println(filteredRoll);

  delay(1);
}

