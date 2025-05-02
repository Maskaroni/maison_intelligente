#include "Alarm.h"
#include "ViseurAutomatique.h"

#include <LCD_I2C.h>
#include <HCSR04.h>
#include <U8g2lib.h>

#define TRIGGER_PIN 11
#define ECHO_PIN 12
#define p1 2
#define p2 3
#define p3 4
#define p4 5
#define rPin 8
#define gPin 7
#define bPin 9
#define alarmPin 13

LCD_I2C lcd(0x27, 16, 2);
HCSR04 hc(TRIGGER_PIN, ECHO_PIN);

#define DIN_PIN 26
#define CLK_PIN 22
#define CS_PIN 24
U8G2_MAX7219_8X8_F_4W_SW_SPI u8g2(U8G2_R0, CLK_PIN, DIN_PIN, CS_PIN, U8X8_PIN_NONE, U8X8_PIN_NONE);

unsigned long currentTime = 0;

float distance;
const int zero = 0;
float maxNear = 30;
float maxFar = 60;
const float left = 10.0;
const float right = 170.0;
int alarmTreshold = 15;
bool changeAlarmState = false;
bool isMatrixOn = false;
int whichIcon = 0;
String fullCmd = "";
String cmd = "";
String arg1 = "";
String arg2 = "";

Alarm alarm(rPin, gPin, bPin, alarmPin, &distance);
ViseurAutomatique motor(p1, p2, p3, p4, &distance);


//SETUP AND LOOP
void setup() {                                                                                  //setup
  Serial.begin(115200);

  lcd.begin(115200);
  lcd.backlight();

  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  initiationAlarm();
  initiationMotor();

  u8g2.begin();
  u8g2.setContrast(7);
  u8g2.clearBuffer();
  u8g2.sendBuffer(); 

  Serial.println("Setup completed!");


  lcd.print("2486739");
  lcd.setCursor(0, 2);
  lcd.print("MUAHAHAH");
  Serial.println("The screen has displayed my student's and lab number already!");
  delay(2000);

  screen();
}

void loop() {                                                                                   //loop
  currentTime = millis();
  getDistance(currentTime);

  alarm.update();
  motor.update();
  refreshScreen();
  processCommands(currentTime);
}

//INITIATION//////////////////////////////////////////////////////////////////////////////////////
void initiationAlarm() {                                                                        //initiationAlarm
  alarm.setColourA(255, 0, 0);
  alarm.setColourB(0, 0, 255);
  alarm.setDistance(alarmTreshold);
  alarm.setTimeout(3000);
  alarm.setVariationTiming(70);
}

void initiationMotor() {                                                                        //initiationMotor
  motor.setPasParTour(2048);
  motor.setAngleMin(left);
  motor.setAngleMax(right);
  motor.setDistanceMinSuivi(maxNear);
  motor.setDistanceMaxSuivi(maxFar);
}

//COMMANDS////////////////////////////////////////////////////////////////////////////////////////
void processCommands(unsigned long currentTime) {                                               //processCommands
  enum Icon {NO_ICON, CHECK, ERROR, LIMIT_ROSSED};
  Icon _whichIcon = NO_ICON;
  bool isWrong = false;

  displayIcon(currentTime, whichIcon);

  if (Serial.available() > 0) {
    fullCmd = Serial.readStringUntil('\n');
    Serial.println("Arduino a compris: '" + fullCmd + "'");
    analyseCommand();

    if (cmd == "gDist" || cmd == "g_dist") {
      Serial.println(distance);
      isMatrixOn = true;
      whichIcon = CHECK;
    }
    else if (cmd == "cfg") {
      if (arg1 == "alm") {
        alarmTreshold = arg2.toInt();
        alarm.setDistance(alarmTreshold);
        isMatrixOn = true;
        whichIcon = CHECK;
      }
      else if (arg1 == "lim_inf" || arg1 == "lim_sup") {
        isMatrixOn = true;
        whichIcon = processLimitsMotor();
      }
      else {isWrong = true;}
    }
    else {isWrong = true;}


    if (isWrong) {
      Serial.println("Ce n'est pas une commande valide, recommencez.");
      isMatrixOn = true;
      whichIcon = ERROR;
    }
  }
}

void analyseCommand() {                                                                         //analyseCommand
  cmd = "";
  arg1 = "";
  arg2 = "";

  int firstStep = fullCmd.indexOf(';');
  int secondStep = fullCmd.indexOf(';', firstStep + 1);

  if (firstStep == -1) {
    cmd = fullCmd;
    return;
  }

  cmd = fullCmd.substring(0, firstStep);

  if (secondStep != -1) {
    arg1 = fullCmd.substring(firstStep + 1, secondStep);
    arg2 = fullCmd.substring(secondStep + 1);
  } else {
    arg1 = fullCmd.substring(firstStep + 1);
  }
}

int processLimitsMotor() {                                                                      //processLimitsMotor
  int pastLimit;
  int icon;

  if (arg1 == "lim_inf") {
    pastLimit = maxNear;

    maxNear = arg2.toInt();

    icon = 1;

    if (maxNear >= maxFar) {
    Serial.println("  ERREUR! \n Limite inférieure plus grande que limite supérieure.");
    maxNear = pastLimit;
    icon = 3;
    }
  }
  else if (arg1 == "lim_sup") {
    pastLimit = maxFar;

    maxFar = arg2.toInt();

    icon = 1;

    if (maxNear >= maxFar) {
    Serial.println("  ERREUR! \n Limite inférieure plus grande que limite supérieure.");
    maxFar = pastLimit;
    icon = 3;
    }
  }
  else {
    Serial.println("Ce n'est pas une commande valide, recommencez.");
    icon = 2;
  }

  motor.setDistanceMinSuivi(maxNear);
  motor.setDistanceMaxSuivi(maxFar);
  return icon;
}

void displayIcon(unsigned long ct, int whichOne) {                                              //displayIcon
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

//REFRESH AND DISTANCE////////////////////////////////////////////////////////////////////////////
void refreshScreen() {                                                                          //refreshScreen
  unsigned static long timer;
  static bool isTimerStarted = false;

  if (!isTimerStarted) {
    timer = currentTime;
    timer += 100;
    isTimerStarted = true;
  }
  if (currentTime >= timer) {
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

//SCREEN DISPLAY CALLED BY REFRESH_SCREEN/////////////////////////////////////////////////////////
void screen() {                                                                                 //Screen
  lcd.clear();

  lcd.print("Dist: ");
  lcd.print((int)distance);
  lcd.print(" cm");

  lcd.setCursor(0, 2);
  lcd.print("Obj: ");
  if (!(distance < maxNear) && !(distance > maxFar)) {
    lcd.print((int)motor.getAngle());
    lcd.print(" deg");
  }
  else if (distance < maxNear) {
    lcd.print("Trop pret");
  }
  else {
    lcd.print("Trop loin");
  }
}