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
const int RED_PIN = 8;
const int BLUE_PIN = 9;
const int ALARM = 13;

LCD_I2C lcd(0x27, 16, 2);
HCSR04 hc(TRIGGER_PIN, ECHO_PIN);
AccelStepper motor(MOTOR_INTERFACE_TYPE, IN_1, IN_3, IN_2, IN_4);

unsigned long currentTime = 0;

int distance;
int degre;
int aimingAt;
const int zero = 0;
const int maxNear = 30;
const int maxFar = 60;
const int motorLeft = 0;
const int motorRight = 1024;
const int left = 10;
const int right = 170;
const int alarmTreshold = 15;
bool changeAlarmState = false;


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
  refreshScreenPlusStats(currentTime);
  aimingStates();
  alarmStates(currentTime);
}


//REFRESH AND DISTANCE
void refreshScreenPlusStats(unsigned long ct) {                                                 //refreshScreenPlusStats
  unsigned static long timer;
  static bool timerIsDone = false;

  if (!timerIsDone) {
    timer = ct + 100;
    timerIsDone = true;
  }

  if (ct >= timer) {
    Serial.print("etd:2486739,dist:");
    Serial.print(distance);
    Serial.print(",deg:");
    Serial.println(degre);

    screen();

    timerIsDone = false;
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
  noTone(ALARM);

  if (distance <= alarmTreshold) {
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

  tone(ALARM, 500);
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