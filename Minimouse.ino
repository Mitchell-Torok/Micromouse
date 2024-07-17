#include "DualEncoder.hpp"
#include "EncoderOdometry.hpp"
#include "IMUOdometry.hpp"
#include "Wire.h"
#include <MPU6050_light.h>
#include "Motor.hpp"
#include "PIDController.hpp"
#include "Movement.hpp"
#include "Angle.hpp"


MPU6050 mpu(Wire);

#define MOT1PWM 11 // PIN 9 is a PWM pin
#define MOT1DIR 12

#define MOT2PWM 9 // PIN 9 is a PWM pin
#define MOT2DIR 10

#define WHEELRAD 16
#define AXLELENGTH 98

#define DISTANCE 300


mtrn3100::Motor motor1(MOT1PWM,MOT1DIR);
mtrn3100::Motor motor2(MOT2PWM,MOT2DIR);

#define EN_1_A 2 //These are the pins for the PCB encoder
#define EN_1_B 7 //These are the pins for the PCB encoder
#define EN_2_A 3 //These are the pins for the PCB encoder
#define EN_2_B 8 //These are the pins for the PCB encoder

mtrn3100::DualEncoder encoder(EN_1_A, EN_1_B, EN_2_A, EN_2_B);
mtrn3100::EncoderOdometry encoder_odometry(WHEELRAD,AXLELENGTH); //TASK1 TODO: IDENTIFY THE WHEEL RADIUS AND AXLE LENGTH
mtrn3100::IMUOdometry IMU_odometry;
mtrn3100::PIDController controllerLeft(20,0, 0);
mtrn3100::PIDController controllerRight(20,0, 0);
mtrn3100::PIDController controllerIMU (7, 0, 0);
mtrn3100::Angle imu;
mtrn3100::Movement movement(controllerLeft, controllerRight, encoder);

    int count = 0;
    bool start_count = false;
    const char* magicString = "flfrfrrf";
    int mag = 0;
    // 0 = off, 1 = forward, 2 = left, 3 = right 4 = stop
    int exe = 0;
    float rTuner = 0;
    float lTuner = 0;
    float mpuBearing = 0;
    bool speedChange = true;

void setup() {
  
    Serial.begin(115200);
    Serial.println("about to wire begin");
    Wire.begin();
    
    //Set up the IMU
    Serial.println("about to mpu begin");
    delay(100);
    byte status = mpu.begin();
    Serial.println("completed mpu begin");

    Serial.print(F("MPU6050 status: "));
    Serial.println(status);
    while(status!=0){ } // stop everything if could not connect to MPU6050
    
    Serial.println(F("Calculating offsets, do not move MPU6050"));
    delay(1000);
    mpu.calcOffsets(true,true);
    Serial.println("Done!\n");
    
}


void loop() {
    mpu.update();

    //Serial.println(mpu.getAngleZ());
    //Serial.println(lTuner);
    //Serial.println(rTuner);

    if (start_count == true) {
      count++;
    }
  
    if (magicString[mag] == 'f' && exe == 0) {
      mpuBearing = mpu.getAngleZ();
      exe = 1;
      movement.forward(250);
    }
    else if (magicString[mag] == 'l' && exe == 0) {
      //Serial.print("HERELLL");
      exe = 2;
      mpuBearing = mpu.getAngleZ() + 90;
    }
    else if (magicString[mag] == 'r' && exe == 0) {
      exe = 3;
      mpuBearing = mpu.getAngleZ() - 90;
    }
    else if (magicString[mag] == '\0' && exe == 0) {
      exe = 4;
      movement.stop();
    }
    
   
    lTuner = controllerIMU.compute(mpu.getAngleZ() - mpuBearing);
    
    // else if (mpu.getAngleZ() - mpuBearing <= 0 && speedChange == true) {
    //   lTuner = mpuBearing - mpu.getAngleZ();
    //   rTuner = 0; 
    // }
    if (exe == 1) {
      motor1.setPWM(controllerLeft.compute(encoder.getLeftRotation()), lTuner);
      motor2.setPWM(controllerRight.compute(encoder.getRightRotation()), -lTuner);

      if (abs(controllerLeft.getError()) <= 2 || abs(controllerRight.getError()) <= 2) {
        start_count = true;
        motor1.setPWM(0, 0);
        motor2.setPWM(0, 0);
        exe = 4;
      }
    }
    else if (exe == 2 || exe == 3) {
      //Serial.println(controllerIMU.compute(mpu.getAngleZ() - mpuBearing));
      motor1.setPWM(controllerIMU.compute(mpu.getAngleZ() - mpuBearing), 0);
      motor2.setPWM(-controllerIMU.compute(mpu.getAngleZ() - mpuBearing), 0);
      if (controllerIMU.getError() <= 5) {
        start_count = true;
      }
    }


    if (count >= 35) {
      exe = 2;
    }
    //Serial.println(count);
    if (count >= 70) {
      count = 0;
      start_count = false;
      mag++;
      exe = 0;
      speedChange = true;
    }

  delay(40);

    

//UNCOMMET FOR TASK 3:
//NOTE: IMU ODOMETRY IS REALLY BAD, THIS TASK EXISTS TO TEACH YOU WHY IMU ODOMETRY SUCKS, DO NOT SPEND TOO LONG ON IT
    //mpu.update();
    //IMU_odometry.update(mpu.getAccX(),mpu.getAccY());
    

//     Serial.print("ODOM:\t\t");
//     Serial.print(encoder_odometry.getX());
//     Serial.print(",\t\t");
//     Serial.print(encoder_odometry.getY());
//     Serial.print(",\t\t");
//     Serial.print(encoder_odometry.getH());
//     Serial.println();
}
