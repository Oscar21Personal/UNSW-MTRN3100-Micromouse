#include "DualEncoder.hpp"
#include "EncoderOdometry.hpp"
#include "PIDController.hpp"
#include "Motor.hpp"
#include "Lidar.hpp"
#include "Robot.hpp"
#include "AutoMap.hpp"
#include <Wire.h>
#include <MPU6050_light.h>
#include <SSD1306Ascii.h>
#include <SSD1306AsciiAvrI2c.h>
#include <math.h>

const int startCellX {6};
const int startCellY {4};
const int goalCellX {4};
const int goalCellY {2};
const char startCellYaw {'N'};
mtrn3100::Robot robot(15, 100);   // Wheel radius and axial length in mm
mtrn3100::AutoMap maze(startCellX, startCellY, goalCellX, goalCellY);

int nodeCoords[arraySize];

// pin definitions
#define I2C_ADDRESS 0x3C
SSD1306AsciiAvrI2c oled;

void displayCellCount(float cellCount) {
  // Display percentage
  oled.clear();
  oled.set2X();
  oled.setCursor(35, 3);
  oled.print(round((cellCount / 45.0) * 100.0));
  oled.print(" %");
}

void setup() {

  // Initialise nodeCoords array
  for (int i{0}; i < arraySize; i++) {
    nodeCoords[i] = -1;
  }

  robot.setup();
  robot.setCurrentState(startCellX, startCellY, startCellYaw);  // Initial cell_x, cell_y, orientation

  maze.updateMaze();

  // Initialise display
  oled.begin(&Adafruit128x64, I2C_ADDRESS);
  oled.setFont(System5x7);
  oled.clear();
  oled.set2X();
  oled.setCursor(35, 3);

}


void loop() {

  int currentCellX {startCellX};
  int currentCellY {startCellY};
  char currentCellYaw {startCellYaw};
  float cellCount{0};
  int isVisited[row][col];
  for (int i{0}; i < row; i++) {
    for (int j{0}; j < col; j++) {
      isVisited[i][j] = 0;
    }
  }

  // Explore cells until reached goal
  while (currentCellX != goalCellX || currentCellY != goalCellY) {

    if (isVisited[currentCellY][currentCellX] == 0) {
      cellCount++;
    }
    isVisited[currentCellY][currentCellX] = 1;
    displayCellCount(cellCount);

    if (currentCellYaw == 'N') {
      if (robot.isLeftWall() == 1) maze.setVerticalWall(currentCellX, currentCellY, 1);
      if (robot.isFrontWall() == 1) maze.setHorizontalWall(currentCellX, currentCellY, 1);
      if (robot.isRightWall() == 1) maze.setVerticalWall(currentCellX + 1, currentCellY, 1);
    }
    if (currentCellYaw == 'E') {
      if (robot.isLeftWall() == 1) maze.setHorizontalWall(currentCellX, currentCellY, 1);
      if (robot.isFrontWall() == 1) maze.setVerticalWall(currentCellX + 1, currentCellY, 1);
      if (robot.isRightWall() == 1) maze.setHorizontalWall(currentCellX, currentCellY + 1, 1);
    }
    if (currentCellYaw == 'S') {
      if (robot.isLeftWall() == 1) maze.setVerticalWall(currentCellX + 1, currentCellY, 1);
      if (robot.isFrontWall() == 1) maze.setHorizontalWall(currentCellX, currentCellY + 1, 1);
      if (robot.isRightWall() == 1) maze.setVerticalWall(currentCellX, currentCellY, 1);
    }
    if (currentCellYaw == 'W') {
      if (robot.isLeftWall() == 1) maze.setHorizontalWall(currentCellX, currentCellY + 1, 1);
      if (robot.isFrontWall() == 1) maze.setVerticalWall(currentCellX, currentCellY, 1);
      if (robot.isRightWall() == 1) maze.setHorizontalWall(currentCellX, currentCellY, 1);
    }

    maze.updateMaze();
    
    String nextCellStringX {maze.getCellPath(currentCellX, currentCellY)[0]};
    String nextCellStringY {maze.getCellPath(currentCellX, currentCellY)[1]};
    int nextCellX {nextCellStringX.toInt()};
    int nextCellY {nextCellStringY.toInt()};
    
    nodeCoords[0] = nextCellX * 250 + 125;
    nodeCoords[1] = nextCellY * 250 + 125;

    while (robot.run(nodeCoords) == 0) {
      // Execute the movement
    }
    
    if (nextCellX > currentCellX  && nextCellY == currentCellY) currentCellYaw = 'E';
    if (nextCellX < currentCellX  && nextCellY == currentCellY) currentCellYaw = 'W';
    if (nextCellX == currentCellX  && nextCellY > currentCellY) currentCellYaw = 'S';
    if (nextCellX == currentCellX  && nextCellY < currentCellY) currentCellYaw = 'N';
    currentCellX = nextCellX;
    currentCellY = nextCellY;

  }

  if (isVisited[currentCellY][currentCellX] == 0) {
    cellCount++;
  }
  isVisited[currentCellY][currentCellX] = 1;
  displayCellCount(cellCount);
  oled.println();
  oled.println("Find Path!");

  // Extract the whole path
  String path {maze.getCellPath(startCellX, startCellY)};
  // Return back to start
  for (int i{path.length() - 2}; i >= 0 ; i -= 2) {
    String cellX {path[i]};
    String cellY {path[i+1]};
    nodeCoords[path.length() - 2 - i] = cellX.toInt() * 250 + 125;
    nodeCoords[path.length() - 2 - i+1] = cellY.toInt() * 250 + 125;
  }
  nodeCoords[path.length()] = startCellX * 250 + 125;
  nodeCoords[path.length() + 1] = startCellY * 250 + 125;

  while (robot.run(nodeCoords) == 0) {
    // Execute the path
  }

  // Go to goal with the path found
  for (int i{0}; i < path.length(); i += 2) {
    String cellX {path[i]};
    String cellY {path[i+1]};
    nodeCoords[i] = cellX.toInt() * 250 + 125;
    nodeCoords[i+1] = cellY.toInt() * 250 + 125;
  }
  nodeCoords[path.length()] = -1;
  nodeCoords[path.length() + 1] = -1;
  while (robot.run(nodeCoords) == 0) {
    // Execute the path
  }
  
  while(1) {
    // Program finished
  }
}
