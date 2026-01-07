#ifndef __PANDACAM_H__
#define __PANDACAM_H__

#include "HWPins.h"

#define CAMERA_STATUS_IDLE 0x00
#define CAMERA_STATUS_ARRIVED 0x01
#define CAMERA_STATUS_HOME 0x02
#define CAMERA_STATUS_MOVING 0x03
#define CAMERA_STATUS_HEADINGHOME 0x04
#define CAMERA_STATUS_ABORTED 0x05

#define MAX_TILT_ALLOWED 200
#define MIN_TILT_ALLOWED 5
#define MAX_PAN_ALLOWED 265
#define MIN_PAN_ALLOWED 5

#define DEFAULT_ALLOWED -1

class PandaCam
{
  private:
    int panMotorSpeed;
    int panMotorInOne;
    int panMotorInTwo;

    int tiltMotorSpeed;
    int tiltMotorInOne;
    int tiltMotorInTwo;

    int panPosPin;
    int tiltPosPin;

    int destinationPan = 0;
    int destinationTilt = 0;

    int currentPanPosition = 0;
    int currentTiltPosition = 0;

    int minPan = MIN_PAN_ALLOWED;
    int maxPan = MAX_PAN_ALLOWED;
    int minTilt = MIN_TILT_ALLOWED;
    int maxTilt = MAX_TILT_ALLOWED;

    int panHome = (270 / 2) + minPan;
    int tiltHome = 45;

    /* Keep track of the status of the camera motion */
    int panCameraStatus = CAMERA_STATUS_IDLE;

    /* Keep track of the status of the camera motion */
    int tiltCameraStatus = CAMERA_STATUS_IDLE;
    
    /* Update the Axis and move it if needed. */
    int UpdateAxis(int angle, int motSPin, int motPinOne, int motPinTwo, int destination);

    /* Get the angle from the specified potentiometer pin */
    int GetPositionAngle(int potPin);

    /* Function allows for flipping a potentiometer if needed (It is needed for Pan on my camera)*/
    bool FlipPotentiometer(int potPin);

    int FilterAnalog(int* readings, unsigned int count);

  public:
    /* Initialise the Panda camera with required pins (speed pins need to be PWM pins and position pins need to be analog pins)*/
    PandaCam();

    /* Initialise and start the camera. It will go home. */
    void InitCamera();

    /* Pan the camera to the specified angle */
    void MoveToPan(int angle);

    /* Tilt the camera to the specified angle */
    void MoveToTilt(int angle);

    /* Pan the camera left until it hits its limit */
    void PanLeft();

    /* Pan the camera right until it hits its limit */
    void PanRight();

    /* Tilt the camera up until it hits its limit */
    void TiltUp();

    /* Tilt the camera down until it hits its limit */
    void TiltDown();

    /* Stop the tilt in its current position */
    void StopTilt();

    /* Stop the pan in its current position */
    void StopPan();

    /* Update the camera and check its limits. Returns true if either motor is in motion
    !! Make sure you call this regularly on every loop !! */
    void UpdateCamera();

    /* Send the Panda Camera to its home position */
    void GoHome();

    /* Is the camera in its home position? */
    bool IsHome();

    /* Print to the serial any debug information (Don't forget to have Serial.begin() in your own code to see the output)*/
    void PrintCameraDebug();

    /* Get the current pan angle of the camera */
    int GetCurrentPanAngle();

    /* Get the current tilt angle of the camera */
    int GetCurrentTiltAngle();

    /* Get the current status of the Pan Motor */
    uint8_t GetPanMotorStatus();

    /* Get the current status of the Tilt Motor */
    uint8_t GetTiltMotorStatus();

    /* Get the current destination angle for the PAN axis */    
    int GetPanDestinationAngle();

    /* Get the current destination angle for the TILT axis */    
    int GetTiltDestinationAngle();

    /* Allows the user to change the max and min angles for the camera tilt */
    bool SetMinMaxTilt(int minAngle, int maxAngle);

    /* Allows the user to change the max and min angles for the camera pan */
    bool SetMinMaxPan(int minAngle, int maxAngle);

};
#endif




