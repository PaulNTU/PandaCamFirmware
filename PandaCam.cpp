#include <Arduino.h>
#include "PandaCam.h"

PandaCam::PandaCam()
{
  this->panMotorSpeed = PAN_MOTOR_SPEED;
  this->panMotorInOne = PAN_MOTOR_IN_ONE;
  this->panMotorInTwo = PAN_MOTOR_IN_TWO;

  this->tiltMotorSpeed = TILT_MOTOR_SPEED;
  this->tiltMotorInOne = TILT_MOTOR_IN_ONE;
  this->tiltMotorInTwo = TILT_MOTOR_IN_TWO;

  this->panPosPin = PAN_POSITION_PIN;
  this->tiltPosPin = TILT_POSITION_PIN;
}

void PandaCam::InitCamera()
{
  // Setup the motor pins
  pinMode(this->panMotorInOne, OUTPUT);
  pinMode(this->panMotorInTwo, OUTPUT);
  pinMode(this->tiltMotorInOne, OUTPUT);
  pinMode(this->tiltMotorInTwo, OUTPUT);

  // Speed pins (PWM)
  pinMode(this->panMotorSpeed, OUTPUT);
  pinMode(this->tiltMotorSpeed, OUTPUT);

  // Position pins (Analog in)
  pinMode(this->panPosPin, INPUT);
  pinMode(this->tiltPosPin, INPUT);

  pinMode(MOTOR_STANDBY, OUTPUT);
  digitalWrite(MOTOR_STANDBY, HIGH);

  // Set the home position
  this->GoHome();
}

void PandaCam::UpdateCamera()
{
  this->currentPanPosition = this->GetPositionAngle(this->panPosPin);
  this->currentTiltPosition = this->GetPositionAngle(this->tiltPosPin);

  this->panCameraStatus = this->UpdateAxis(this->currentPanPosition, this->panMotorSpeed, this->panMotorInOne, this->panMotorInTwo, this->destinationPan);
  this->tiltCameraStatus = this->UpdateAxis(this->currentTiltPosition, this->tiltMotorSpeed, this->tiltMotorInOne, this->tiltMotorInTwo, this->destinationTilt);
}

void PandaCam::MoveToPan(int angle)
{
  // Clamp to within an acceptable range
  this->destinationPan = constrain(angle, this->minPan, this->maxPan);
}

void PandaCam::MoveToTilt(int angle)
{
  // Clamp to within an acceptable range
  this->destinationTilt = constrain(angle, this->minTilt, this->maxTilt);
}

void PandaCam::PanLeft()
{
  this->destinationPan = this->minPan;
}

void PandaCam::PanRight()
{
  this->destinationPan = this->maxPan;
  this->panCameraStatus = CAMERA_STATUS_MOVING;
}

void PandaCam::TiltUp()
{
  this->destinationTilt = this->maxTilt;
}

void PandaCam::TiltDown()
{
  this->destinationTilt = this->minTilt;
}

void PandaCam::StopTilt()
{
  // Stop at current position
  this->tiltCameraStatus = CAMERA_STATUS_ABORTED;
  this->destinationTilt = this->currentTiltPosition;
}

void PandaCam::StopPan()
{
  this->panCameraStatus = CAMERA_STATUS_ABORTED;
  this->destinationPan = this->currentPanPosition;
}

void PandaCam::GoHome()
{
  if(!this->IsHome())
  {
    Serial.println("Going Home");
    this->destinationPan = this->panHome;
    this->destinationTilt = this->tiltHome;
    this->panCameraStatus = CAMERA_STATUS_HEADINGHOME;
    this->tiltCameraStatus = CAMERA_STATUS_HEADINGHOME;
  }
}

bool PandaCam::IsHome()
{
  if (this->panCameraStatus == CAMERA_STATUS_HOME)
  {
    if (this->tiltCameraStatus == CAMERA_STATUS_HOME)
    {
      return true;
    }
  }

  return false;
}

void PandaCam::PrintCameraDebug()
{
  // Are we moving?
  if(this->currentPanPosition == this->destinationPan && this->currentTiltPosition == this->destinationTilt)
  {
    // No. Are we home?
    if(this->IsHome())
    {
      Serial.print ("Camera is Home\t");
    }
    else
    {
      Serial.print("In Position\t");
    }

    Serial.print("Pan: ");
    Serial.print(this->currentPanPosition);
    Serial.print("\tTilt: ");
    Serial.print(this->currentTiltPosition);
  }
  else
  {
    Serial.print("Moving To Position\t");
  
    Serial.print ("Pan: ");
    Serial.print(this->currentPanPosition);
    Serial.print(" -- > ");
    Serial.print (this->destinationPan);
    Serial.print("\t");
  
    // Show Tilt Debug
    Serial.print("Tilt: ");
    Serial.print(this->currentTiltPosition);
    Serial.print(" -- > ");
    Serial.print (this->destinationTilt);
  }
  // End line
  Serial.println();
}

int PandaCam::GetCurrentPanAngle()
{
  return this->currentPanPosition;
}

int PandaCam::GetCurrentTiltAngle()
{
  return this->currentTiltPosition;
}

uint8_t PandaCam::GetPanMotorStatus()
{
  return this->panCameraStatus;
}

uint8_t PandaCam::GetTiltMotorStatus()
{
  return this->tiltCameraStatus;
}

int PandaCam::GetPanDestinationAngle()
{
  return this->destinationPan;
}

int PandaCam::GetTiltDestinationAngle()
{
  return this->destinationTilt;
}

int PandaCam::UpdateAxis(int angle, int motSPin, int motPinOne, int motPinTwo, int destination)
{
  int distanceToTravel = angle - destination;
  // Is Pan in position?
  if(abs(distanceToTravel) < 2)
  {
    // Stop motor / Brakes
    analogWrite(motSPin, 0);
    digitalWrite(motPinOne, LOW);
    digitalWrite(motPinTwo, LOW);

    // Return false to indicate we are stopped.
    return CAMERA_STATUS_ARRIVED;
  }
  else
  {
    if (distanceToTravel < 0)
    {
      // Turn
      digitalWrite(motPinOne, HIGH);
      digitalWrite(motPinTwo, LOW);
    }
    else
    {
      // Turn opposite
      digitalWrite(motPinOne, LOW);
      digitalWrite(motPinTwo, HIGH);
    }

    // How fast? Slow down as we approach the destination
    int distance = abs(distanceToTravel);
    if(distance <= 10)
    {
      analogWrite(motSPin, map(distance, 1, 10, 50, 255));
    }
    else
    {
      analogWrite(motSPin, 255);
    }
  }

  // Return true to indicate we are still moving.
  return CAMERA_STATUS_MOVING;
}

int PandaCam::GetPositionAngle(int potPin)
{
  // Get three readings and average to remove noise
  /*
  int vals[50];

  for(unsigned int x = 0; x < 50; x++)
  {
    vals[x] = analogRead(potPin);
    delayMicroseconds(10);
  }
  */
  int filtered = analogRead(potPin); //FilterAnalog(vals, 50);
  
  // Convert analog signal to angle (0 - 1024)
  int val = 0;
  

  if(FlipPotentiometer(potPin))
  {
    val = map(filtered, 0, 1024, 270, 0);
  }
  else
  {
    val = map(filtered, 0, 1024, 0, 270);
  }

  // Clamp it
  if(val < 0) val = 0;
  if(val > 270) val = 270;
  return val;
}

int PandaCam::FilterAnalog(int* readings, unsigned int count) {
  // Sort readings
  for (unsigned int i = 0; i < count - 1; i++) {
    for (unsigned int j = i + 1; j < count; j++) {
      if (readings[j] < readings[i]) {
        int temp = readings[i];
        readings[i] = readings[j];
        readings[j] = temp;
      }
    }
  }

  // Remove lowest and highest
  long sum = 0;
  for (unsigned int i = 1; i < count - 1; i++) {
    sum += readings[i];
  }

  return sum / (count - 2);
}

/* Todo: Maybe add proper config option */
bool PandaCam::FlipPotentiometer(int potPin)
{
  // Flip the Pan potentiometer
  if(potPin == this->tiltPosPin)
  {
    return true;
  }
  return false;
}


/* Allows the user to change the max and min angles for the camera tilt */
bool PandaCam::SetMinMaxTilt(int minAngle, int maxAngle)
{
    // Allow user to set the camera to its default values
  if (minAngle == DEFAULT_ALLOWED) minAngle = MIN_TILT_ALLOWED;
  if (maxAngle == DEFAULT_ALLOWED) maxAngle = MAX_TILT_ALLOWED;

  // Apply limits
  maxAngle = constrain(maxAngle, MIN_TILT_ALLOWED, MAX_TILT_ALLOWED);
  minAngle = constrain(minAngle, MIN_TILT_ALLOWED, MAX_TILT_ALLOWED);

  // Ignore if min is above max or both are identical
  if(minAngle >= maxAngle) return false;

  // Set but constrain
  this->maxTilt = maxAngle;
  this->minTilt = minAngle;

  // Success.
  return true;
}

    /* Allows the user to change the max and min angles for the camera pan */
bool PandaCam::SetMinMaxPan(int minAngle, int maxAngle)
{
  // Allow user to set the camera to its default values
  if (minAngle == DEFAULT_ALLOWED) minAngle = MIN_PAN_ALLOWED;
  if (maxAngle == DEFAULT_ALLOWED) maxAngle = MAX_PAN_ALLOWED;

  // Apply limits
  maxAngle = constrain(maxAngle, MIN_PAN_ALLOWED, MAX_PAN_ALLOWED);
  minAngle = constrain(minAngle, MIN_PAN_ALLOWED, MAX_PAN_ALLOWED);

  // Ignore if min is above max or both are identical
  if(minAngle >= maxAngle) return false;

  // Set but constrain
  this->maxPan = maxAngle;
  this->minPan = minAngle;

  // Success.
  return true;
}