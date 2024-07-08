#include "DualEncoder.hpp"
#include "EncoderOdometry.hpp"
#include "IMUOdometry.hpp"
#include "Wire.h"
#include <MPU6050_light.h>
#include "Motor.hpp"
#include "PIDController.hpp"
#include "Movement.hpp"

//MPU6050 mpu(Wire);



#define MOT1PWM 9 // PIN 9 is a PWM pin
#define MOT1DIR 10

#define MOT2PWM 11 // PIN 9 is a PWM pin
#define MOT2DIR 12

#define WHEELRAD 16
#define AXLELENGTH 98

#define DISTANCE 300

mtrn3100::Motor motor1(MOT1PWM,MOT1DIR);
mtrn3100::Motor motor2(MOT2PWM,MOT2DIR);

#define EN_1_A 3 //These are the pins for the PCB encoder
#define EN_1_B 8 //These are the pins for the PCB encoder
#define EN_2_A 2 //These are the pins for the PCB encoder
#define EN_2_B 7 //These are the pins for the PCB encoder

mtrn3100::DualEncoder encoder(EN_1_A, EN_1_B,EN_2_A, EN_2_B);
mtrn3100::EncoderOdometry encoder_odometry(WHEELRAD,AXLELENGTH); //TASK1 TODO: IDENTIFY THE WHEEL RADIUS AND AXLE LENGTH
mtrn3100::IMUOdometry IMU_odometry;
mtrn3100::PIDController controllerLeft(200,5, 5);
mtrn3100::PIDController controllerRight(200,5, 5);
//mtrn3100::Angle imu()
mtrn3100::Movement movement(controllerLeft, controllerRight, encoder);

void setup() {
    Serial.begin(115200);
    //Wire.begin();

    //Set up the IMU
    // byte status = mpu.begin();
    // Serial.print(F("MPU6050 status: "));
    // Serial.println(status);
    // while(status!=0){ } // stop everything if could not connect to MPU6050
    
    // Serial.println(F("Calculating offsets, do not move MPU6050"));
    // delay(1000);
    // mpu.calcOffsets(true,true);
    // Serial.println("Done!\n");
    //controllerLeft.zeroAndSetTarget(encoder.getLeftRotation(), DISTANCE / WHEELRAD);
    //controllerRight.zeroAndSetTarget(encoder.getRightRotation(), -DISTANCE / WHEELRAD);
    movement.forward(300);
    string magicString = "fllrlflrfflrlflrlfllrlfllrl"
    // Serial.println(controllerLeft.compute(encoder.getLeftRotation()));
    // Serial.println(encoder.getLeftRotation());
}


void loop() {

//UNCOMMENT FOR TASK 2: 
//THE DELAY IS REQUIRED OTHERWISE THE ENCODER DIFFERENCE IS TOO SMALL
    //delay(50);
    //encoder_odometry.update(encoder.getLeftRotation(),encoder.getRightRotation());
   
    Serial.print("LEFT -->  ");
    //controllerLeft.compute(encoder.getLeftRotation());
    motor1.setPWM(controllerLeft.compute(encoder.getLeftRotation()));
    Serial.print("RIGHT -->  ");
    //controllerRight.compute(encoder.getRightRotation());
    motor2.setPWM(controllerRight.compute(encoder.getRightRotation()));


  

  delay(30);

    

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
