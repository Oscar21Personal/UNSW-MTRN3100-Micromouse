#pragma once

#include <Arduino.h>
#include <MPU6050_light.h>
#include <math.h>

const int row{5};
const int col{9};

namespace mtrn3100 {

class AutoMap {
  public:
    // AutoMap() = default;
      
    AutoMap(int x1, int y1, int x2, int y2): startX(x1), startY(y1), goalX(x2), goalY(y2) {
      // Initialize array
      for (int i{0}; i < row+1; i++) {
        for (int j{0}; j < col; j++) {
          if (i == 0 || i == row) {
            horizontalWalls[i][j] = 1;
          } else {
            horizontalWalls[i][j] = 0;
          }
        }
      }
      for (int i{0}; i < row; i++) {
        for (int j{0}; j < col+1; j++) {
          verticalWalls[i][j] = 0;
          if (j == 0 || j == col) {
            verticalWalls[i][j] = 1;
          } else {
            verticalWalls[i][j] = 0;
          }
        }
      }
      for (int i{0}; i < row; i++) {
        for (int j{0}; j < col; j++) {
          cellValues[i][j] = -1;
        }
      }
      cellValues[y2][x2] = 0;
    }

    void printArray() {
      for (int i{0}; i < row+1; i++) {
        for (int j{0}; j < col; j++) {
          Serial.print(horizontalWalls[i][j]);
          Serial.print(",");
        }
        Serial.println();
      }
      Serial.println();
      for (int i{0}; i < row; i++) {
        for (int j{0}; j < col+1; j++) {
          Serial.print(verticalWalls[i][j]);
          Serial.print(",");
        }
        Serial.println();
      }
      Serial.println();
      for (int i{0}; i < row; i++) {
        for (int j{0}; j < col; j++) {
          Serial.print(cellValues[i][j]);
          Serial.print(",");
        }
        Serial.println();
      }
    }

    void setHorizontalWall(int x, int y, int flag) {
      horizontalWalls[y][x] = flag;
    }

    void setVerticalWall(int x, int y, int flag) {
      verticalWalls[y][x] = flag;
    }

    int getCellValue(int x, int y) {
      return cellValues[y][x];
    }

    void initialiseCellValues() {
      // Initialise cell values
      for (int i{0}; i < row; i++) {
        for (int j{0}; j < col; j++) {
          if (j != goalX || i != goalY) {
            cellValues[i][j] = -1;
          }
        }
      }
      currentExploredValue = 0;
      mazeChanged = 1;
    }

    void updateMaze() {
      initialiseCellValues();
      // Flood Fill Algorithm
      while (mazeChanged == 1) {
        mazeChanged = 0;
        for (int i{0}; i < row; i++) {
          for (int j{0}; j < col; j++) {
            if (cellValues[i][j] == currentExploredValue) {
              // Explore all four directions
              int northWall = horizontalWalls[i][j];
              int southWall = horizontalWalls[i+1][j];
              int eastWall = verticalWalls[i][j+1];
              int westWall = verticalWalls[i][j];
              if (northWall == 0 && cellValues[i-1][j] == -1) {
                cellValues[i-1][j] = currentExploredValue + 1;
                mazeChanged = 1;
              }
              if (southWall == 0 && cellValues[i+1][j] == -1) {
                cellValues[i+1][j] = currentExploredValue + 1;
                mazeChanged = 1;
              }
              if (eastWall == 0 && cellValues[i][j+1] == -1) {
                cellValues[i][j+1] = currentExploredValue + 1;
                mazeChanged = 1;
              }
              if (westWall == 0 && cellValues[i][j-1] == -1) {
                cellValues[i][j-1] = currentExploredValue + 1;
                mazeChanged = 1;
              }
            }
          }
        }
        currentExploredValue++;
      }
    }

    String getCellPath(int startCellX, int startCellY) {
      String path{};
      int currentX {startCellX};
      int currentY {startCellY};
      while (currentX != goalX || currentY != goalY) {

        int currentCellValue {cellValues[currentY][currentX]};
        if (currentX != 0 && cellValues[currentY][currentX - 1] == currentCellValue - 1 && verticalWalls[currentY][currentX] == 0) {
          currentX--;
        } 
        else if (currentX != col - 1 && cellValues[currentY][currentX + 1] == currentCellValue - 1 && verticalWalls[currentY][currentX + 1] == 0) {
          currentX++;
        }
        else if (currentY != 0 && cellValues[currentY - 1][currentX] == currentCellValue - 1 && horizontalWalls[currentY][currentX] == 0) {
          currentY--;
        }
        else if (currentY != row - 1 && cellValues[currentY + 1][currentX] == currentCellValue - 1 && horizontalWalls[currentY + 1][currentX] == 0) {
          currentY++;
        }
        
        path = path + currentX + currentY;

      }

      return path;
    }

  private:
    int startX{};
    int startY{};
    int goalX{};
    int goalY{};
    int horizontalWalls[row+1][col];
    int verticalWalls[row][col+1]; 
    int cellValues[row][col]; 
    int currentExploredValue{0};
    int mazeChanged{1};
};

}  // namespace mtrn3100