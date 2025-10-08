#include "DualEncoder.hpp"
#include "EncoderOdometry.hpp"
#include "Wire.h"
#include "PIDController.hpp"
#include "Motor.hpp"
#include "Lidar.hpp"
#include "Robot.hpp"
#include <MPU6050_light.h>
#include <math.h>

int nodeCoords[] = {2172,1125,2160,1095,2156,1023,2150,966,2145,903,2136,843,2125,791,2087,752,2073,730,2036,674,1987,620,1961,573,1950,544,1917,487,1870,454,1799,468,1728,478,1668,479,1604,465,1569,405,1533,345,1491,289,1459,227,1427,166,1406,125,1406,125};
  // int nodeCoords[] = {250,125,375,125};
const int arraySize = sizeof(nodeCoords) / sizeof(nodeCoords[0]);
mtrn3100::Robot robot(15, 100);   // Wheel radius and axial length in mm

void setup() {

  robot.setup();
  robot.setCurrentState(8, 4, 'N');  // Initial cell_x, cell_y, orientation

}


void loop() {

  robot.run(nodeCoords, arraySize);

}
