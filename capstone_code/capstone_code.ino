#include <NewPing.h>

// Sensor Pins
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

// Motor Pins
#define MOTOR_LEFT_FWD 19
#define MOTOR_LEFT_BCK 21
#define MOTOR_RIGHT_FWD 22
#define MOTOR_RIGHT_BCK 23

// Constants
#define WALL_DISTANCE 15
#define FRONT_THRESHOLD 10
#define CLIFF_THRESHOLD 200
#define GRID_SIZE 4
#define CELL_SIZE_CM 20

NewPing sonarLeft(TRIG_LEFT, ECHO_LEFT, 200);
NewPing sonarRight(TRIG_RIGHT, ECHO_RIGHT, 200);
NewPing sonarFront(TRIG_FRONT, ECHO_FRONT, 200);

// Grid and robot state
int grid[GRID_SIZE][GRID_SIZE] = {0};
int posX = 0, posY = 0;
int direction = 0; // 0: up, 1: right, 2: down, 3: left

// Path stack for backtracking
#define MAX_PATH_LENGTH 64
struct Position {
  int x;
  int y;
};
Position pathStack[MAX_PATH_LENGTH];
int pathTop = -1;

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

// Check if all cells are explored
bool isGridComplete() {
  for (int i = 0; i < GRID_SIZE; i++) {
    for (int j = 0; j < GRID_SIZE; j++) {
      if (grid[i][j] == 0) return false;
    }
  }
  return true;
}

// Check if robot is trapped
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

// Movement
void moveForward() {
  pushPath(posX, posY); // Save current position

  digitalWrite(MOTOR_LEFT_FWD, HIGH);
  digitalWrite(MOTOR_RIGHT_FWD, HIGH);
  digitalWrite(MOTOR_LEFT_BCK, LOW);
  digitalWrite(MOTOR_RIGHT_BCK, LOW);
  delay(1000);
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
  digitalWrite(MOTOR_LEFT_BCK, HIGH);
  digitalWrite(MOTOR_RIGHT_BCK, LOW);
  delay(500);
  stopRobot();
  direction = (direction + 3) % 4;
}

void turnRight() {
  digitalWrite(MOTOR_LEFT_FWD, HIGH);
  digitalWrite(MOTOR_RIGHT_FWD, LOW);
  digitalWrite(MOTOR_LEFT_BCK, LOW);
  digitalWrite(MOTOR_RIGHT_BCK, HIGH);
  delay(500);
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

// Go to a previous cell
void goTo(int targetX, int targetY) {
  while (posX != targetX || posY != targetY) {
    if (posX < targetX) while (direction != 1) turnRight();
    else if (posX > targetX) while (direction != 3) turnRight();
    else if (posY < targetY) while (direction != 2) turnRight();
    else if (posY > targetY) while (direction != 0) turnRight();
    moveForward();
  }
}

// Backtracking logic
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

// Setup
void setup() {
  pinMode(MOTOR_LEFT_FWD, OUTPUT);
  pinMode(MOTOR_LEFT_BCK, OUTPUT);
  pinMode(MOTOR_RIGHT_FWD, OUTPUT);
  pinMode(MOTOR_RIGHT_BCK, OUTPUT);
  pinMode(IR_LEFT, INPUT);
  pinMode(IR_RIGHT, INPUT);
  pinMode(IR_FRONT, INPUT);
  pinMode(IR_BOTTOM, INPUT);
  Serial.begin(115200);
  grid[posY][posX] = 1;
}

// Loop
void loop() {
  if (isGridComplete()) {
    stopRobot();
    printGrid();
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

  if ((frontDist > 0 && frontDist < FRONT_THRESHOLD) || (frontIR > CLIFF_THRESHOLD)) {
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
