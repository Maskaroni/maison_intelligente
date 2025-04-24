#include <U8g2lib.h>
#include <LiquidCrystal.h>
#include <LCD_I2C.h>
#include <AccelStepper.h>
#include <HCSR04.h>
#define TRIGGER_PIN 11
#define ECHO_PIN 12
#define MOTOR_INTERFACE_TYPE 4
#define IN_1 2
#define IN_2 3
#define IN_3 4
#define IN_4 5
#define CLK_PIN 22
#define DIN_PIN 26
#define CS_PIN  24
const int RED_PIN = 8;
const int BLUE_PIN = 9;
const int ALARM = 13;

LCD_I2C lcd(0x27, 16, 2);
HCSR04 hc(TRIGGER_PIN, ECHO_PIN);
AccelStepper motor(MOTOR_INTERFACE_TYPE, IN_1, IN_3, IN_2, IN_4);
U8G2_MAX7219_8X8_F_4W_SW_SPI u8g2(U8G2_R0, CLK_PIN, DIN_PIN, CS_PIN, U8X8_PIN_NONE, U8X8_PIN_NONE);

unsigned long currentTime = 0;

int distance;
int degre;
int aimingAt;
const int zero = 0;
int maxNear = 30;
int maxFar = 60;
const int motorLeft = 0;
const int motorRight = 1024;
const int left = 10;
const int right = 170;
int alarmTreshold = 15;
bool changeAlarmState = false;
bool isMatrixOn = false;


//SETUP AND LOOP
void setup() {                                                                                  //setup
  Serial.begin(115200);

  lcd.begin(115200);
  lcd.backlight();

  motor.setMaxSpeed(500);
  motor.setAcceleration(150);
	motor.setSpeed(150);

  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  pinMode(ALARM, OUTPUT);

  pinMode(RED_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);

  u8g2.begin();
  u8g2.setContrast(7);
  u8g2.clearBuffer();
  u8g2.sendBuffer(); 

  Serial.println("Setup completed!");


  lcd.print("2486739");
  lcd.setCursor(0, 2);
  lcd.print("Labo 4B");
  Serial.println("The screen has displayed my student's and lab number already!");
  delay(2000);

  screen();
}

void loop() {                                                                                   //loop
  currentTime = millis();

  getDistance(currentTime);
  refreshScreen(currentTime);
  aimingStates();
  alarmStates(currentTime);
  commands(currentTime);
}

//COMMANDS
void commands(unsigned long currentTime) {
  enum Icon {NO_ICON, CHECK, ERROR, LIMIT_CROSSED};
  static Icon whichIcon = NO_ICON;
  display(currentTime, whichIcon);
  String cmd = "";
  int slicedCmd;

  if (Serial.available() > 0) {
    cmd = Serial.readStringUntil('\n');

    if (cmd == "gDist") {
      Serial.println(distance);
      isMatrixOn = true;
      whichIcon = CHECK;
    }
    else if (cmd.startsWith("cfg;alm;")) {
      cmd = cmd.substring(8);
      alarmTreshold = cmd.toInt();
      isMatrixOn = true;
      whichIcon = CHECK;
    }
    else if (cmd.startsWith("cfg;lim_")) {
      isMatrixOn = true;
      whichIcon = processLimitsMotor(cmd);
    }
    else {
      Serial.println("Arduino a compris: '" + cmd + "'! Ce n'est pas une commande valide, recommencez.");
      isMatrixOn = true;
      whichIcon = ERROR;
    }
  }
}

int processLimitsMotor(String cmd) {
  int pastLimit;
  int icon;

  if (cmd.startsWith("cfg;lim_inf;")) {
    pastLimit = maxNear;

    cmd = cmd.substring(13);
    maxNear = cmd.toInt();

    icon = 1;
  }
  else if (cmd.startsWith("cfg;lim_sup;")) {
    pastLimit = maxFar;

    cmd = cmd.substring(13);
    maxFar = cmd.toInt();

    icon = 1;
  }
  else {
    Serial.println("Arduino a compris: '" + cmd + "'! Ce n'est pas une commande valide, recommencez.");
    icon = 2;
  }

  if (maxNear >= maxFar) {
    Serial.println("  ERREUR! \n Limite inférieure plus grande que limite supérieure.");
    maxNear = pastLimit;
    icon = 3;
  }

  return icon;
}

void display(unsigned long ct, int whichOne) {
  unsigned static long timer;
  static bool isTimerStarted = false;

  if (!isTimerStarted && isMatrixOn) {
    u8g2.clearBuffer();
    switch (whichOne) {
      case 1:
        u8g2.drawLine(6, 7, 1, 2);
        u8g2.drawLine(2, 1, 3, 0);
        break;
      case 2:
        u8g2.drawLine(0, 0, 7, 7);
        u8g2.drawLine(7, 0, 0, 7);
        break;
      case 3:
        u8g2.drawCircle(4, 3, 3);
        u8g2.drawLine(7, 0, 0, 7);
        break;
    }
    u8g2.sendBuffer(); 
    timer = ct + 3000;
    isTimerStarted = true;
  }
  if (ct >= timer) {
    isMatrixOn = false;
    isTimerStarted = false;
    u8g2.clearBuffer();
    u8g2.sendBuffer();
  }
}

//REFRESH AND DISTANCE
void refreshScreen(unsigned long ct) {                                                          //refreshScreen
  unsigned static long timer;
  static bool isTimerStarted = false;

  if (!isTimerStarted) {
    timer = ct + 100;
    isTimerStarted = true;
  }
  if (ct >= timer) {
    screen();
    isTimerStarted = false;
  }
}

void getDistance(unsigned long ct) {                                                            //getDistance
  unsigned static long timer;
  static bool isTimerDone = false;
  static int lastDistance = zero;
  static int tempDistance;

  if (!isTimerDone) {
    timer = ct + 50;
    isTimerDone = true;
  }

  if (ct >= timer) {
    tempDistance = hc.dist();
    if (tempDistance != zero) {
      distance = tempDistance;
      lastDistance = distance;
    }
    else {
      distance = lastDistance;
    }
    isTimerDone = false;
  }
}

//STATES
void aimingStates() {                                                                           //aimingStates
  enum AimingStates {MOTOR_OFF, AIMING};
  static AimingStates currentState;

  currentState = (distance < maxNear || distance > maxFar) ? MOTOR_OFF : AIMING;

  motor.run();
  switch (currentState) {
    case AIMING:
      aiming();
      break;
    case MOTOR_OFF:
      motorOff();
      break;
  }
}

void alarmStates(unsigned long currentTime) {                                                   //alarmStates
  enum AlarmStates {ALARM_OFF, ALARM_ON};
  static AlarmStates currentState;

  int timeBeforeCheckingForAlarm = 50;
  static unsigned long startOfTheTime = currentTime;

  if (currentTime - startOfTheTime >= timeBeforeCheckingForAlarm) {
    if (changeAlarmState == true) {
      if (currentState == ALARM_OFF) {
        currentState = ALARM_ON;
      }
      else {
        currentState = ALARM_OFF;
      }
    }

    changeAlarmState = false;

    switch (currentState) {
      case ALARM_OFF:
        digitalWrite(RED_PIN, LOW);
        digitalWrite(BLUE_PIN, LOW);
        stopTheSound();
        break;
      case ALARM_ON:
        makeSound(currentTime);
        policeLights(currentTime);
        break;
      default:
        currentState = ALARM_OFF;
        break;
    }
  }
}

//THINGS TO DO DEPENDING ON STATE
void aiming() {                                                                                 //aiming
  degre = map(distance, maxNear, maxFar, left, right);
  aimingAt = map(degre, left, right, motorLeft, motorRight);
  motor.moveTo(aimingAt);
}

void motorOff() {                                                                               //motorOff
  if (motor.distanceToGo() == zero) {
    motor.disableOutputs();
  }
}

void stopTheSound() {                                                                           //stopTheSound
  digitalWrite(ALARM, zero);

  if (distance <= alarmTreshold && distance != zero) {
    changeAlarmState = true;
  }
}

void makeSound(unsigned long currentTime) {                                                     //makeSound
  static bool isTimerStarted = false;
  static unsigned long timer;

  if (distance > alarmTreshold) {
    if (!(isTimerStarted)) {
      timer = currentTime + 2500;   //Je sais que c'est moins de 3000 milisecondes mais dû aux autres parti, ça se rapproche plus de 3 secondes
      isTimerStarted = true;
    }
    else if (currentTime >= timer) {
      changeAlarmState = true;
      isTimerStarted = false;
      return;
    }
  }
  else {
    isTimerStarted = false;
  }

  digitalWrite(ALARM, 255);
}

void policeLights(unsigned long currentTime) {                                                  //policeLights
  static int redLedState = LOW;
  bool isChangingColor = false;
  int delay = 50;
  static unsigned long lastTime = currentTime;

  if (currentTime - lastTime >= delay) {
    lastTime = currentTime;
    isChangingColor = true;
  }

  if (isChangingColor) {
    if (redLedState == LOW) {
      redLedState = HIGH;
      digitalWrite(RED_PIN, HIGH);
      digitalWrite(BLUE_PIN, LOW);
    }
    else {
      redLedState = LOW;
      digitalWrite(RED_PIN, LOW);
      digitalWrite(BLUE_PIN, HIGH);
    }
  }
}

//SCREEN DISPLAY CALLED BY REFRESH_SCREEN
void screen() {                                                                                 //Screen
  lcd.clear();

  lcd.print("Dist: ");
  lcd.print(distance);
  lcd.print(" cm");

  lcd.setCursor(0, 2);
  lcd.print("Obj: ");
  if (!(distance < maxNear) && !(distance > maxFar)) {
    lcd.print(degre);
    lcd.print(" deg");
  }
  else if (distance < maxNear) {
    lcd.print("Trop pret");
  }
  else {
    lcd.print("Trop loin");
  }
}