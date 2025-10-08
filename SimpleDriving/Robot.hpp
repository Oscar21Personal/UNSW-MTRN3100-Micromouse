#pragma once

#include <Arduino.h>
#include <MPU6050_light.h>

#include "Wire.h"
#include "DualEncoder.hpp"
#include "EncoderOdometry.hpp"
#include "PIDController.hpp"
#include "Motor.hpp"
#include "Lidar.hpp"

// These are the pins for the PCB encoder
constexpr int EN_1_A {2};
constexpr int EN_1_B {7};
constexpr int EN_2_A {3};
constexpr int EN_2_B {8};

// These are the pins for motors
constexpr int MOT1PWM {11};
constexpr int MOT1DIR {12};
constexpr int MOT2PWM {9};
constexpr int MOT2DIR {10};

MPU6050 mpu(Wire);

namespace mtrn3100 {

class Robot {
  public:
    Robot(float radius, float wheelBase) : 
      encoder{EN_1_A, EN_1_B, EN_2_A, EN_2_B}, encoder_odometry{radius, wheelBase}, 
      lMotor{MOT1PWM, MOT1DIR}, rMotor{MOT2PWM, MOT2DIR},
      odoForwardController{10, 0, 1}, odoRotateController{450, 1, 10},  // F: 5 0.5 1, 15 0 1, 20 0 1.5   R: 150 10 5, 100 6 1, 500 0 20
      imuRotateController{450, 1, 10} {}

    void setup() {
      Serial.begin(115200);
      Wire.begin();

      //Set up the IMU
      byte status = mpu.begin();
      Serial.print(F("MPU6050 status: "));
      Serial.println(status);
      while(status!=0){ } // stop everything if could not connect to MPU6050

      //Set up the Lidars
      Lidars.lidarsetup();
      
      Serial.println(F("Calculating offsets, do not move MPU6050"));
      delay(1000);
      mpu.calcOffsets(true,true);
      Serial.println("Done!\n");

    }

    void setCurrentState(int x, int y, char yaw) {
      currentX = x * 250 + 125;
      currentY = y * 250 + 125;
      if (yaw == 'N') currentYaw = -90;
      if (yaw == 'E') currentYaw = 0;
      if (yaw == 'S') currentYaw = 90;
      if (yaw == 'W') currentYaw = 180;
    }

    void move(float distance) {
      float speed{};
      float adjustSpeed{};
      int adjustScale{0.5};  // Use 1 for real test
      int lidarFlag{};
      switch(phase) {
        case 0:
          // Waiting phase
          break;
        case 1:
          // Starting: Set odometry target x
          encoder_odometry.reset();
          odoForwardController.zeroAndSetTarget(encoder_odometry.getX(), distance);
          imuRotateController.zeroAndSetTarget(mpu.getAngleZ()*PI/180, 0);
          phase = 2;
          break;
        case 2:
          // Motion phase
          speed = odoForwardController.compute(encoder_odometry.getX());
          // Capped at max speed
          if (speed > maxSpeed) speed = maxSpeed;
          if (speed < -maxSpeed) speed = -maxSpeed;

          adjustSpeed = imuRotateController.compute(mpu.getAngleZ()*PI/180);
          if (adjustSpeed > adjustScale*maxSpeed) adjustSpeed = adjustScale*maxSpeed;
          if (adjustSpeed < -adjustScale*maxSpeed) adjustSpeed = -adjustScale*maxSpeed;

          // Check distance using lidars
          if (abs(odoForwardController.getError()) < 40) {
            lidarFlag = 0;
          } else {
            lidarFlag = Lidars.check_distance();
          }
          
          if (lidarFlag == 1) {
            while (Lidars.check_distance_crash() <= 50) {
              lMotor.setPWM(-speed * hardEncoder);
              rMotor.setPWM(speed);
              phase = 0;
            }
          } else if (lidarFlag == 2) {
            lMotor.setPWM(speed * hardEncoder);
            rMotor.setPWM(-(speed + 30));
          } else if (lidarFlag == 3) {
            lMotor.setPWM((speed + 30) * hardEncoder);
            rMotor.setPWM(-speed);
          } else {
            lMotor.setPWM((speed - adjustSpeed) * hardEncoder);
            rMotor.setPWM(-speed - adjustSpeed);
          }

          if (abs(odoForwardController.getError()) < 3) {
            phase = 0;
          }
          break;
      }
    }

    void turn(float degree) {
      float speed{};
      float angleError{};
      switch(phase) {
        case 0:
          // Waiting phase
          break;
        case 1:
          // Starting: Set odometry target angle
          encoder_odometry.reset();
          odoRotateController.zeroAndSetTarget(encoder_odometry.getH(), degree*PI/180);
          previousAngle = mpu.getAngleZ();
          phase = 2;
          break;
        case 2:
          // Motion phase
          speed = odoRotateController.compute(encoder_odometry.getH());
          // Capped at max speed
          if (speed > maxSpeed) speed = maxSpeed;
          if (speed < -maxSpeed) speed = -maxSpeed;
          lMotor.setPWM(-speed);
          rMotor.setPWM(-speed);
          if (abs(odoRotateController.getError()) < 0.1) {
            phase = 3;
          }
          break;
        case 3:
          // Set IMU target angle (for adjusting error)
          angleError = degree - (mpu.getAngleZ() - previousAngle);
          imuRotateController.zeroAndSetTarget(mpu.getAngleZ()*PI/180, angleError*PI/180);
          phase = 4;
          break;
        case 4:
          speed = imuRotateController.compute(mpu.getAngleZ()*PI/180);
          // Capped at max speed
          if (speed > maxSpeed) speed = maxSpeed;
          if (speed < -maxSpeed) speed = -maxSpeed;
          lMotor.setPWM(-speed);
          rMotor.setPWM(-speed);
          if (abs(imuRotateController.getError()) < 0.03) {
            phase = 0;
          }
          break;
      }
    }

    void stop() {
      lMotor.setPWM(0);
      rMotor.setPWM(0);
    }

    void run(int nodeCoords[], int arraySize) {

      delay(50);
      encoder_odometry.update(encoder.getLeftRotation(), -1 * encoder.getRightRotation());
      mpu.update();

      // Move to next node
      if (actionCount < arraySize && phase != emergencyStopPhase) {
        // Find travel distance and angle
        int nextX {nodeCoords[actionCount]};
        int nextY {nodeCoords[actionCount + 1]};
        float targetDistance {0.9*sqrt(pow(nextX - currentX, 2) + pow(nextY - currentY, 2))};
        float targetAngle {};
        if (nextX == currentX && nextY == currentY) targetAngle = currentYaw;
        else if (nextY == currentY && nextX > currentX) targetAngle = 0; 
        else if (nextY == currentY && nextX < currentX) targetAngle = 180; 
        else if (nextX == currentX && nextY > currentY) targetAngle = 90; 
        else if (nextX == currentX && nextY < currentY) targetAngle = -90; 
        else targetAngle = atan2(nextY - currentY, nextX - currentX) * 180 / PI; 

        float angleDiff {targetAngle - currentYaw};
        // Limit angle between -180 to 180 deg
        if (angleDiff > 180) angleDiff -= 360;
        if (angleDiff <= -180) angleDiff += 360;

        // Rotate angle
        if (motionFlag == 0) {
          turn(-angleDiff);
          if (phase == 0) {
            motionFlag = 1;
            phase = 1;
            currentYaw = targetAngle;
            angleErrorAfterMove = mpu.getAngleZ();
          }
        }
        // Move forward
        if (motionFlag == 1) {
          move(targetDistance);
          if (phase == 0) {
            motionFlag = 2;
            phase = 1;
            currentX = nextX;
            currentY = nextY;
            angleErrorAfterMove -= mpu.getAngleZ();
          }
        }
        // Adjust the angle error
        if (motionFlag == 2) {
          turn(angleErrorAfterMove);
          if (phase == 0) {
            motionFlag = 3;
            phase = 1;
          }
        }
        // Start another motion
        if (motionFlag == 3) {
          actionCount += 2;
          motionFlag = 0;
        }

      } else {
        stop();
      }
      
    }

  private:
    // Class composition
    DualEncoder encoder;
    EncoderOdometry encoder_odometry;
    Motor lMotor;
    Motor rMotor;
    Lidar Lidars;
    PIDController odoForwardController;
    PIDController odoRotateController;
    PIDController imuRotateController;
    
    // Variables
    const int maxSpeed{130};
    const int emergencyStopPhase{999};
    int phase{1};   // 0 is waiting phase, 1 is starting phase
    int actionCount{0};
    float previousAngle{0};
    float hardEncoder{1.04}; // Encoder offset to combat linear drift

    float currentX{0};
    float currentY{0};
    float currentYaw{0};
    float angleErrorAfterMove{};
    int motionFlag{0};
};

}