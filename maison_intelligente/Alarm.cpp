#include <Arduino.h>
#include "Alarm.h"

Alarm::Alarm(int rPin, int gPin, int bPin, int buzzerPin, float* distancePtr) {
  _rPin = rPin;
  _gPin = gPin;
  _bPin = bPin;
  _buzzerPin = buzzerPin;
  _distance = distancePtr;
}

void Alarm::update() {
  static bool firstTime = true;
  if (firstTime) {
    _state = WATCHING;
    firstTime = false;
  }

  _currentTime = millis();

  int timeBeforeCheckingForAlarm = 50;
  static unsigned long startOfTheTime = _currentTime;

  if (_currentTime - startOfTheTime >= timeBeforeCheckingForAlarm) {
    if (_turnOnFlag == true) {
      _state = WATCHING;
      _turnOnFlag = false;
    }
    if (_turnOffFlag == true) {
      _state = OFF;
      _turnOffFlag = false;
    }

    switch (_state) {
      case OFF:
        _offState();
        break;
      case WATCHING:
        _watchState();
        break;
      case ON:
        _onState();
        break;
      case TESTING:
        _testingState();
        break;
    }
  }
}

void Alarm::setColourA(int r, int g, int b) {
  _colA[0] = r;
  _colA[1] = g;
  _colA[2] = b;
}
void Alarm::setColourB(int r, int g, int b) {
  _colB[0] = r;
  _colB[1] = g;
  _colB[2] = b;
}

void Alarm::setVariationTiming(unsigned long ms) {
  _variationRate = ms;
}

void Alarm::setDistance(float d) {
  _distanceTrigger = d;
}

void Alarm::setTimeout(unsigned long ms) {
  _timeoutDelay = ms;
}

void Alarm::turnOff() {
  _turnOffFlag = true;
}

void Alarm::turnOn() {
  _turnOnFlag = true;
}

void Alarm::test() {
  _state = TESTING;
}

AlarmState Alarm::getState() const {
  return _state;
}

void Alarm::_setRGB(int r, int g, int b) {
  digitalWrite(_rPin, r);
  digitalWrite(_gPin, g);
  digitalWrite(_bPin, b);
}

void Alarm::_turnOff() {
  noTone(_buzzerPin);
  _setRGB(0, 0, 0);
}

void Alarm::_turnOn() {
  tone(_buzzerPin, 255);

  static bool newColor = true;

  if (newColor) {
    _lastUpdate = _currentTime;
    newColor = false;
  }

  if (_currentTime - _lastUpdate >= _variationRate) {
    newColor = true;
    if (!_currentColor) {
      _currentColor = true;
      _setRGB(_colA[0], _colA[1], _colA[2]);
    }
    else {
      _currentColor = false;
      _setRGB(_colB[0], _colB[1], _colB[2]);
    }
  }
}

void Alarm::_offState() {
  _turnOff();
}

void Alarm::_watchState() {
  static bool firstTime = true;
  if (firstTime) {
    _turnOff();
    firstTime = false;
  }

  if (*_distance <= _distanceTrigger && *_distance != 0) {
    firstTime = true;
    _state = ON;
  }
}

void Alarm::_onState() {
  static bool isTimerStarted = false;
  static unsigned long timer;

  if (*_distance > _distanceTrigger) {
    
    if (!(isTimerStarted)) {
      timer = _currentTime + _timeoutDelay;
      isTimerStarted = true;
    }
    else if (_currentTime >= timer) {
      turnOn();
      isTimerStarted = false;
      return;
    }
  }
  else {
    isTimerStarted = false;
  }

  _turnOn();
}

void Alarm::_testingState() {
  static bool firstTime = true;
  if (firstTime) {
    _testStartTime = _currentTime;
    firstTime = false;
  }

  if (_currentTime - _testStartTime >= _timeoutDelay) {
    firstTime = true;
    turnOff();
  }
  else {
    _turnOn();
  }
}