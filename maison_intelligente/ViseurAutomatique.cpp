#include <Arduino.h>
#include <AccelStepper.h>
#include "ViseurAutomatique.h"

#define MOTOR_INTERFACE_TYPE 4
const int zero = 0;
bool isChangingState = false;

ViseurAutomatique::ViseurAutomatique(int p1, int p2, int p3, int p4, float* distanceRef) {
  _stepper = AccelStepper(MOTOR_INTERFACE_TYPE, p1, p3, p2, p4);
  _stepper.setMaxSpeed(500);
  _stepper.setAcceleration(150);
	_stepper.setSpeed(150);
  _distance = distanceRef;
}

void ViseurAutomatique::update() {
  _currentTime = millis();
  getAngle();

  if (!_turnOnFlag && !_turnOffFlag) {
    _etat = (*_distance < _distanceMinSuivi || *_distance > _distanceMaxSuivi) ? REPOS : SUIVI;
  }

  if (_turnOnFlag) {
    _etat = REPOS;
    _turnOnFlag = false;
  }
  if (_turnOffFlag) {
    _etat = INACTIF;
  }

  _stepper.run();
  switch (_etat) {
    case INACTIF:
      _inactifState();
      break;
    case SUIVI:
      _suiviState();
      break;
    case REPOS:
      _reposState();
      break;
  }
}

void ViseurAutomatique::setAngleMin(float angle) {
  _angleMin = angle;
}

void ViseurAutomatique::setAngleMax(float angle) {
  _angleMax = angle;
}

void ViseurAutomatique::setPasParTour(int steps) {
  _stepsPerRev = steps;
}

void ViseurAutomatique::setDistanceMinSuivi(float distanceMin) {
  _distanceMinSuivi = distanceMin;
}

void ViseurAutomatique::setDistanceMaxSuivi(float distanceMax) {
  _distanceMaxSuivi = distanceMax;
}

float ViseurAutomatique::getAngle() const {
  return (map(*_distance, _distanceMinSuivi, _distanceMaxSuivi, _angleMin, _angleMax));
}

void ViseurAutomatique::activer() {
  _turnOnFlag = true;
  _turnOffFlag = false;
}

void ViseurAutomatique::desactiver() {
  _turnOffFlag = true;
}

const char* ViseurAutomatique::getEtatTexte() const {
  switch (_etat) {
    case 0:
      return "INACTIF";
      break;
    case 1:
      return "SUIVI";
      break;
    case 2:
      return "REPOS";
      break;
    default:
      return "Erreur";
      break;
  }
}

void ViseurAutomatique::_inactifState() {
  _stepper.disableOutputs();
}

void ViseurAutomatique::_suiviState() {
  _stepper.moveTo(_angleEnSteps(getAngle()));
}

void ViseurAutomatique::_reposState() {
  if (_stepper.distanceToGo() <= zero) {
    _stepper.disableOutputs();
  }
}


long ViseurAutomatique::_angleEnSteps(float angle) const {
  return (map(angle, _angleMin, _angleMax, (-1*(_stepsPerRev/4)), (_stepsPerRev/4)));
}