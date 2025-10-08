#pragma once

#include "Wire.h"
#include <VL6180X.h>


VL6180X sensor1;
VL6180X sensor2;
VL6180X sensor3;

class Lidar {
private:
  int sensor1_pin = A0; // ENABLE PIN FOR SENSOR 1
  int sensor2_pin = A1; // ENABLE PIN FOR SENSOR 2
  int sensor3_pin = A2; // ENABLE PIN FOR SENSOR 3
  int movingAverage[7];
  int average;
  int arraySize = 5;



public:
  void lidarsetup() {
          // SET UP ENABLE PINS AND DISABLE SENSORS
        pinMode(sensor1_pin, OUTPUT);
        pinMode(sensor2_pin, OUTPUT);
        pinMode(sensor3_pin, OUTPUT);
        digitalWrite(sensor1_pin, LOW);
        digitalWrite(sensor2_pin, LOW);
        digitalWrite(sensor3_pin, LOW);
        // //ENABLE SENSORS AND CHANGE ADDRESSES
        digitalWrite(sensor1_pin, HIGH);
        delay(50);
        sensor1.init();
        sensor1.configureDefault();
        sensor1.setTimeout(250);
        sensor1.setAddress(0x54);
        delay(50);

        digitalWrite(sensor2_pin, HIGH);
        delay(50);
        sensor2.init();
        sensor2.configureDefault();
        sensor2.setTimeout(250);
        sensor2.setAddress(0x56);

        digitalWrite(sensor3_pin, HIGH);
        delay(50);
        sensor3.init();
        sensor3.configureDefault();
        sensor3.setTimeout(250);
        sensor3.setAddress(0x58);
        delay(500);
        // Initialise the first set of front lidar scans and average value
        for (int i = 0; i < arraySize; i++) {
          //print_lidar ();
          Serial.print("Initialising Lidar:");
          movingAverage[i] = sensor2.readRangeSingleMillimeters();
          Serial.println(sensor2.readRangeSingleMillimeters());
          delay(50);
        }
        avgDistance();
        // Serial.println("Calculated intiial average:");
        // Serial.println(average);
  }

  void print_lidar () {
    Serial.print(sensor1.readRangeSingleMillimeters());
    Serial.print(" | ");
    Serial.print(sensor2.readRangeSingleMillimeters());
    Serial.print(" | ");
    Serial.print(sensor3.readRangeSingleMillimeters());
    Serial.println();
  }

  int check_distance_crash () {
    return sensor2.readRangeSingleMillimeters();
  }

  int check_distance () {
    updateAverage();
    // if (average < 110 ) {
    if (sensor2.readRangeSingleMillimeters() < 50) {
      //Serial.print("STOP\n");
      return 1;
    } else if (sensor3.readRangeSingleMillimeters() < 50) {
      //Serial.print("RIGHT\n");
      return 2;
    } else if (sensor1.readRangeSingleMillimeters() < 50) {
      //Serial.print("LEFT\n");
      return 3;
    } else {
      return 0;
    }
  }
  
  int getLeftSensor() const { return sensor1.readRangeSingleMillimeters(); }
  int getFrontSensor() const { return sensor2.readRangeSingleMillimeters(); }
  int getRightSensor() const { return sensor3.readRangeSingleMillimeters(); }

  // Shifts the lidar scans in the array to the left and updates the end value 
  // with the most recent lidar scan
  void updateScans() {
    for (int i = 0; i < arraySize - 1; i++){
      movingAverage[i] = movingAverage[i+1];
      // Serial.print("Scan value : ");
      // Serial.println(movingAverage[i]);
    }
    int lastValue = movingAverage[arraySize - 1];
    int scanValue = sensor2.readRangeSingleMillimeters();
    if (scanValue != 0){
        movingAverage[arraySize - 1] = scanValue;
    }
    else {
       movingAverage[arraySize - 1] = lastValue;
    }
  }

  // Averages the lidar scans from the movingAverage array
  void avgDistance() {
    int sampleAverage = 0;
    for (int i = 0; i < arraySize; i++) {
      sampleAverage += movingAverage[i];
    }
    average = sampleAverage / arraySize;
  }

  // Automate updating average - called in check_distance
  void updateAverage() {
    updateScans();
    avgDistance();
    // Serial.print(" Average value: ");
    // Serial.println(average);
  }

};


