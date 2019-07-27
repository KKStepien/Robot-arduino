#define L_DIRECTION 4 //0-back, 1-front
#define L_SPEED 5
#define R_DIRECTION 7 //0-front, 1-back
#define R_SPEED 6
#define PWM_MAX 160

#define moveDetector 11

#define trigFront 8
#define echoFront 9
#define trigL 12
#define echoL 13
#define trigR 0
#define echoR 1

#define diodeBlueFront 2
#define diodeBlueL 19
#define diodeBlueR 18
#define diodeRed 3
#define buzzer 10

#define sensorSideL A0
#define sensorSideR A1
#define lightSensor A2
#define potentiometer A3

int limitLight = 0;
int light = 0;

int fenceFront = 0;
int fenceL = 0;
int fenceR = 0;

void setup() {
  pinMode(L_DIRECTION, OUTPUT);
  pinMode(L_SPEED, OUTPUT);
  pinMode(R_DIRECTION, OUTPUT);
  pinMode(R_SPEED, OUTPUT);

  pinMode(diodeBlueFront, OUTPUT);
  pinMode(diodeBlueL, OUTPUT);
  pinMode(diodeBlueR, OUTPUT);
  pinMode(diodeRed, OUTPUT);
  pinMode(buzzer, OUTPUT);

  pinMode(sensorSideL, INPUT_PULLUP);
  pinMode(sensorSideR, INPUT_PULLUP);
  pinMode(moveDetector, INPUT);

  pinMode(trigFront, OUTPUT);
  pinMode(echoFront, INPUT);
  pinMode(trigL, OUTPUT);
  pinMode(echoL, INPUT);
  pinMode(trigR, OUTPUT);
  pinMode(echoR, INPUT);
}

void loop() {

  startRobot();

  int standardSpeed = 100;
  int movementSpeed = 90;
  int delayBack = 700;
  int delayMovement = 550;
  int delaySideSensor = 450;
  int fenceDistance = 12;
  boolean motorLoop = true;

  while (motorLoop) {
    if (light <= limitLight) {
      startMotors(standardSpeed);
      calculateFence();
      
      if (fenceFront <= fenceDistance) {
        fenceFrontMovement(movementSpeed, delayBack, delayMovement);
      }
      if (fenceL <= fenceDistance) {
        turnLeft(movementSpeed, movementSpeed, delayBack, delayMovement);
      }
      if (fenceR <= fenceDistance) {
        turnRight(movementSpeed, movementSpeed, delayBack, delayMovement);
      }
      if (digitalRead(sensorSideL) == LOW) {
        turnLeft(standardSpeed, movementSpeed, delaySideSensor, delayMovement);
      }
      if (digitalRead(sensorSideR) == LOW) {
        turnRight(standardSpeed, movementSpeed, delaySideSensor, delayMovement);
      }
      if (digitalRead(sensorSideL) == LOW && digitalRead(sensorSideR) == LOW) {
        sensorSideLowBoth(movementSpeed, delayMovement, delaySideSensor);
      }
    }
    else if (light > limitLight) {
      stopRobot();
      motorLoop = false;
    }

    light = analogRead(lightSensor);
    limitLight = analogRead(potentiometer);
  }
}

//**function**//
void startRobot() {
  int start = digitalRead(moveDetector);
  while (start == LOW) {
    start = digitalRead(moveDetector);
  }

  digitalWrite(buzzer, HIGH);
  glimmerDiode(diodeBlueFront);
  glimmerDiode(diodeBlueL);
  glimmerDiode(diodeBlueR);
  digitalWrite(diodeBlueFront, HIGH);
  digitalWrite(diodeBlueL, HIGH);
  digitalWrite(diodeBlueR, HIGH);
  digitalWrite(buzzer, LOW);
}

void calculateFence() {
  fenceFront = calculateDistance(trigFront, echoFront);
  fenceL = calculateDistance(trigL, echoL);
  fenceR = calculateDistance(trigR, echoR);
}

int calculateDistance(int trig, int echo) { //ultradżwiękowe czujniki odległosći
  long times, distance;
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  times = pulseIn(echo, HIGH);
  distance = times / 58;

  return distance;
}

void startMotors(int speed) {
  leftMotor(speed);
  rightMotor(speed);
}

void fenceFrontMovement(int speed, int delayBack, int delayMovement) {

  backMove(diodeRed, speed, delayBack);
  if (fenceL <= 20 || fenceR <= 20) {
    if (fenceL <= fenceR) {
      rightCircleMove(speed, delayMovement);
    }
    else {
      leftCircleMove(speed, delayMovement);
    }
  }
  else {
    turnRandom(speed, delayMovement);
  }
}

void sensorSideLowBoth(int speed, int delayMovement, int delaySideSensor) {

  backMove(diodeRed, speed, delayMovement);
  if (digitalRead(sensorSideR) == LOW) {
    rightCircleMove(speed, delaySideSensor);
  }
  else if (digitalRead(sensorSideL) == LOW) {
    leftCircleMove(speed, delaySideSensor);
  }
  else {
    turnRandom(speed, delaySideSensor);
  }
}

void turnRandom(int speed, int delay) {
  int los = random(0, 2);
  if (los == 0) {
    rightCircleMove(speed, delay);
  }
  else {
    leftCircleMove(speed, delay);
  }
}

void turnLeft(int speedBack, int speedTurn, int delayBack, int delayTurn) {
  backMove(diodeRed, speedBack, delayBack);
  leftCircleMove(speedTurn, delayTurn);
}

void turnRight(int speedBack, int speedTurn, int delayBack, int delayTurn) {
  backMove(diodeRed, speedBack, delayBack);
  rightCircleMove(speedTurn, delayTurn);
}

void backMove(int diode, int movementSpeed, int delayBack) {
  digitalWrite(diode, HIGH);
  leftMotor(-movementSpeed);
  rightMotor(-movementSpeed);
  delay(delayBack);
  digitalWrite(diode, LOW);
  calculateFence();
}

void leftCircleMove(int movementSpeed, int delayMovement) {
  digitalWrite(buzzer, HIGH);
  leftMotor(movementSpeed);
  rightMotor(-movementSpeed);
  delay(delayMovement);
  digitalWrite(buzzer, LOW);
  calculateFence();
}

void rightCircleMove(int movementSpeed, int delayMovement) {
  digitalWrite(buzzer, HIGH);
  leftMotor(-movementSpeed);
  rightMotor(movementSpeed);
  delay(delayMovement);
  digitalWrite(buzzer, LOW);
  calculateFence();
}

void leftMotor(int V) {
  if (V > 0) {
    digitalWrite(L_DIRECTION, 1);
  }
  else {
    V = abs(V);
    digitalWrite(L_DIRECTION, 0);
  }
  V = map(V, 0, 100, 0, PWM_MAX);
  analogWrite(L_SPEED, V);
}

void rightMotor(int V) {
  if (V > 0) {
    digitalWrite(R_DIRECTION, 0);
  }
  else {
    V = abs(V);
    digitalWrite(R_DIRECTION, 1);
  }
  V = map(V, 0, 100, 0, PWM_MAX);
  analogWrite(R_SPEED, V);
}

void stopMotors() {
  analogWrite(L_SPEED, 0);
  analogWrite(R_SPEED, 0);
}

void stopRobot() {
  stopMotors();
  glimmerDiode(diodeRed);
  glimmerDiode(diodeBlueFront);
  glimmerDiode(diodeBlueL);
  glimmerDiode(diodeBlueR);
}

void glimmerDiode(int diode) {
  for (int i = 0; i < 5; i++) {
    digitalWrite(diode, HIGH);
    delay(100);
    digitalWrite(diode, LOW);
    delay(100);
  }
}
