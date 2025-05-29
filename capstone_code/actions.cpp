// actions.cpp
#include <Arduino.h>
#include <NewPing.h>
#include <LiquidCrystal_I2C.h>

extern LiquidCrystal_I2C lcd;

// Pin definitions 
#define TRIG_LEFT 13
#define ECHO_LEFT 14
#define TRIG_RIGHT 15
#define ECHO_RIGHT 16
#define TRIG_FRONT 17
#define ECHO_FRONT 18
#define IR_LEFT 32
#define IR_RIGHT 33
#define IR_FRONT 34
#define IR_BOTTOM 35

#define MOTOR_LEFT_FWD 19
#define MOTOR_LEFT_BCK 21
#define MOTOR_RIGHT_FWD 22
#define MOTOR_RIGHT_BCK 23
#define VACUUM_MOTOR 39
#define MOP_MOTOR 27

#define WALL_DISTANCE 15
#define FRONT_THRESHOLD 10
#define CLIFF_THRESHOLD 200
#define GRID_SIZE 4
#define CELL_SIZE_CM 20
#define MAX_PATH_LENGTH 64

// Variables
NewPing sonarLeft(TRIG_LEFT, ECHO_LEFT, 200);
NewPing sonarRight(TRIG_RIGHT, ECHO_RIGHT, 200);
NewPing sonarFront(TRIG_FRONT, ECHO_FRONT, 200);

int grid[GRID_SIZE][GRID_SIZE] = {0};
int posX = 0, posY = 0;
int direction = 0; // 0: up, 1: right, 2: down, 3: left

struct Position {
  int x;
  int y;
};

Position pathStack[MAX_PATH_LENGTH];
int pathTop = -1;

// Function declarations 
void pushPath(int x, int y);
Position popPath();
bool isGridComplete();
bool isTrapped();
void printGrid();
void moveForward();
void turnLeft();
void turnRight();
void stopRobot();
void goTo(int targetX, int targetY);
void backtrack();
void sweepGrid();
void mopGrid();
void runRobotLogic();

// --- Function definitions ---
void pushPath(int x, int y) {
  if (pathTop < MAX_PATH_LENGTH - 1) {
    pathTop++;
    pathStack[pathTop].x = x;
    pathStack[pathTop].y = y;
  }
}

Position popPath() {
  Position p = {posX, posY};
  if (pathTop >= 0) {
    p = pathStack[pathTop];
    pathTop--;
  }
  return p;
}

bool isGridComplete() {
  for (int i = 0; i < GRID_SIZE; i++) {
    for (int j = 0; j < GRID_SIZE; j++) {
      if (grid[i][j] == 0) return false;
    }
  }
  return true;
}

bool isTrapped() {
  for (int d = 0; d < 4; d++) {
    int nextX = posX, nextY = posY;
    if (d == 0 && posY > 0) nextY--;
    else if (d == 1 && posX < GRID_SIZE - 1) nextX++;
    else if (d == 2 && posY < GRID_SIZE - 1) nextY++;
    else if (d == 3 && posX > 0) nextX--;
    if (nextX >= 0 && nextX < GRID_SIZE && nextY >= 0 && nextY < GRID_SIZE) {
      if (grid[nextY][nextX] == 0) return false;
    }
  }
  return true;
}

void printGrid() {
  Serial.println("\nExplored Grid Map:");
  for (int i = 0; i < GRID_SIZE; i++) {
    for (int j = 0; j < GRID_SIZE; j++) {
      Serial.print(grid[i][j]);
      Serial.print(" ");
    }
    Serial.println();
  }
}

void moveForward() {
  pushPath(posX, posY);
  digitalWrite(MOTOR_LEFT_FWD, HIGH);
  digitalWrite(MOTOR_RIGHT_FWD, HIGH);
  digitalWrite(MOTOR_LEFT_BCK, HIGH);
  digitalWrite(MOTOR_RIGHT_BCK, HIGH);
  delay(2000);
  stopRobot();

  if (direction == 0 && posY > 0) posY--;
  else if (direction == 1 && posX < GRID_SIZE - 1) posX++;
  else if (direction == 2 && posY < GRID_SIZE - 1) posY++;
  else if (direction == 3 && posX > 0) posX--;

  grid[posY][posX] = 1;
}

void turnLeft() {
  digitalWrite(MOTOR_LEFT_FWD, LOW);
  digitalWrite(MOTOR_RIGHT_FWD, HIGH);
  digitalWrite(MOTOR_LEFT_BCK, LOW);
  digitalWrite(MOTOR_RIGHT_BCK, HIGH);
  delay(1500);
  stopRobot();
  direction = (direction + 3) % 4;
}

void turnRight() {
  digitalWrite(MOTOR_LEFT_FWD, HIGH);
  digitalWrite(MOTOR_RIGHT_FWD, LOW);
  digitalWrite(MOTOR_LEFT_BCK, HIGH);
  digitalWrite(MOTOR_RIGHT_BCK, LOW);
  delay(1500);
  stopRobot();
  direction = (direction + 1) % 4;
}

void stopRobot() {
  digitalWrite(MOTOR_LEFT_FWD, LOW);
  digitalWrite(MOTOR_RIGHT_FWD, LOW);
  digitalWrite(MOTOR_LEFT_BCK, LOW);
  digitalWrite(MOTOR_RIGHT_BCK, LOW);
  delay(200);
}

void goTo(int targetX, int targetY) {
  while (posX != targetX || posY != targetY) {
    if (posY > targetY) while (direction != 0) turnRight();
    if (posX < targetX) while (direction != 1) turnRight();
    if (posY < targetY) while (direction != 2) turnRight();
    if (posX > targetX) while (direction != 3) turnRight();
    moveForward();
  }
}

void backtrack() {
  while (pathTop >= 0) {
    Position prev = popPath();
    for (int d = 0; d < 4; d++) {
      int nx = prev.x, ny = prev.y;
      if (d == 0 && prev.y > 0) ny--;
      else if (d == 1 && prev.x < GRID_SIZE - 1) nx++;
      else if (d == 2 && prev.y < GRID_SIZE - 1) ny++;
      else if (d == 3 && prev.x > 0) nx--;
      if (nx >= 0 && nx < GRID_SIZE && ny >= 0 && ny < GRID_SIZE) {
        if (grid[ny][nx] == 0) {
          goTo(prev.x, prev.y);
          return;
        }
      }
    }
  }
  stopRobot();
  printGrid();
  while (true);
}

void sweepGrid() {
  digitalWrite(VACUUM_MOTOR, HIGH);
  for (int y = 0; y < GRID_SIZE; y++) {
    if (y % 2 == 0) {
      for (int x = 0; x < GRID_SIZE; x++) {
        goTo(x, y);
        delay(500);
      }
    } else {
      for (int x = GRID_SIZE - 1; x >= 0; x--) {
        goTo(x, y);
        delay(500);
      }
    }
  }
  digitalWrite(VACUUM_MOTOR, LOW);
  stopRobot();
  printGrid();
}

void mopGrid() {
  digitalWrite(MOP_MOTOR, HIGH);
  for (int y = 0; y < GRID_SIZE; y++) {
    if (y % 2 == 0) {
      for (int x = 0; x < GRID_SIZE; x++) {
        goTo(x, y);
        delay(500);
      }
    } else {
      for (int x = GRID_SIZE - 1; x >= 0; x--) {
        goTo(x, y);
        delay(500);
      }
    }
  }
  digitalWrite(MOP_MOTOR, LOW);
  stopRobot();
  printGrid();
}

void runRobotLogic() {
  if (isGridComplete()) {
    stopRobot();
    printGrid();
    delay(10000);
    goTo(0, 0);
    stopRobot();
    delay(1000);
    sweepGrid();
    delay(1000);
    mopGrid();
    while (true);
  }

  if (isTrapped()) {
    backtrack();
    return;
  }

  int leftDist = sonarLeft.ping_cm();
  int rightDist = sonarRight.ping_cm();
  int frontDist = sonarFront.ping_cm();
  int cliffSensor = analogRead(IR_BOTTOM);
  int frontIR = analogRead(IR_FRONT);

  if (cliffSensor < CLIFF_THRESHOLD) {
    stopRobot();
    return;
  }

  int nextX = posX, nextY = posY;
  if (direction == 0 && posY > 0) nextY--;
  else if (direction == 1 && posX < GRID_SIZE - 1) nextX++;
  else if (direction == 2 && posY < GRID_SIZE - 1) nextY++;
  else if (direction == 3 && posX > 0) nextX--;

  if ((frontDist > 0 && frontDist < FRONT_THRESHOLD) || (frontIR < CLIFF_THRESHOLD)) {
    if (nextX >= 0 && nextX < GRID_SIZE && nextY >= 0 && nextY < GRID_SIZE) {
      if (grid[nextY][nextX] == 0) {
        grid[nextY][nextX] = 2;
      }
    }
    turnRight();
  } else {
    if (nextX >= 0 && nextX < GRID_SIZE && nextY >= 0 && nextY < GRID_SIZE && grid[nextY][nextX] > 0) {
      turnRight();
    } else {
      moveForward();
    }
  }
}


void startScan() {
  Serial.println("Starting room scan...");
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Room scan started");


  // Initialize grid and starting position
  for (int i = 0; i < GRID_SIZE; i++)
    for (int j = 0; j < GRID_SIZE; j++)
      grid[i][j] = 0;

  posX = 0;
  posY = 0;
  direction = 0;
  grid[posY][posX] = 1;

  // Clear path stack
  pathTop = -1;

  // Main scanning loop: Run until grid is complete
  while (!isGridComplete()) {
    runRobotLogic();
  }

  // After grid is complete, do sweeping and mopping
  stopRobot();
  printGrid();
  delay(1000);
  goTo(0, 0);
  delay(1000);
  sweepGrid();
  delay(1000);
  mopGrid();

  Serial.println("Room scan complete.");
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Scan complete");
}
void startClean() {
  Serial.println("Starting cleaning phase...");
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Cleaning started");

  // Move robot to home position (0,0)
  goTo(0, 0);

  // Sweeping 
  Serial.println("Sweeping...");
  lcd.setCursor(0, 1);
  lcd.print("Sweeping...");
  digitalWrite(VACUUM_MOTOR, HIGH);  // Turn vacuum ON
  // Sweep grid in zigzag pattern
  for (int y = 0; y < GRID_SIZE; y++) {
    if (y % 2 == 0) {
      for (int x = 0; x < GRID_SIZE; x++) {
        goTo(x, y);
        delay(500); // wait while sweeping each cell
      }
    } else {
      for (int x = GRID_SIZE - 1; x >= 0; x--) {
        goTo(x, y);
        delay(500);
      }
    }
  }
  digitalWrite(VACUUM_MOTOR, LOW);  // Turn vacuum OFF
  stopRobot();

  //Mopping 
  Serial.println("Mopping...");
  lcd.setCursor(0, 1);
  lcd.print("Mopping...   ");
  digitalWrite(MOP_MOTOR, HIGH);  // Turn mop ON
  // Mop grid in zigzag pattern
  for (int y = 0; y < GRID_SIZE; y++) {
    if (y % 2 == 0) {
      for (int x = 0; x < GRID_SIZE; x++) {
        goTo(x, y);
        delay(500); // wait while mopping each cell
      }
    } else {
      for (int x = GRID_SIZE - 1; x >= 0; x--) {
        goTo(x, y);
        delay(500);
      }
    }
  }
  digitalWrite(MOP_MOTOR, LOW);  // Turn mop OFF
  stopRobot();

  Serial.println("Cleaning complete.");
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Cleaning done");
}

