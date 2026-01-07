#include <Wire.h>

#define I2C_SLAVE_ADDRESS 0x18

#define STATUS_COMMAND 0x00
#define STOP_COMMAND 0x01
#define PAN_COMMAND 0x02
#define TILT_COMMAND 0x03
#define HOME_COMMAND 0x04
#define SETTINGS_COMMAND 0x05

#define PAN_LEFT 0x00
#define PAN_RIGHT 0x01
#define PAN_TO 0x02
#define PAN_STOP 0x03

#define TILT_UP 0x00
#define TILT_DOWN 0x01
#define TILT_TO 0x02
#define TILT_STOP 0x03

#define STATUS_FIRMWARE 0x00
#define STATUS_POSITION 0x01

#define SETTINGS_LIMITS 0x00

#define COMMAND_MAX_LENGTH 32
uint8_t commandReceived[COMMAND_MAX_LENGTH];
int receivedCommandSize = 0;

void BeginI2c()
{
  // Move 
  Wire.begin(I2C_SLAVE_ADDRESS);
  Wire.onReceive(ReceivedI2CEvent);
  Wire.onRequest(RequestI2CEvent);
}

void RequestI2CEvent(int howMany)
{
  switch(commandReceived[0])
  {
    case STATUS_COMMAND:
      // Return the current status and camera position
      switch(commandReceived[1])
      {
        case STATUS_FIRMWARE:
          Serial.println("Status Firmware Command Received");
          SendFirmwareStatusResponse();
          break;

        case STATUS_POSITION:
          SendPositionStatusResponse();
          break;

      }
      break;
  }
}

/*
Event has been received on the I2C Channel
*/
void ReceivedI2CEvent(int howMany)
{
  // No events
  if(howMany < 1) return;

  // Clear any previous command
  memset(commandReceived, 0, COMMAND_MAX_LENGTH);

  // Should never exceed but clamp just in case
  if (howMany > COMMAND_MAX_LENGTH) howMany = COMMAND_MAX_LENGTH;

  // Read the command data
  for(int x = 0; x < howMany; x++)
  {
    commandReceived[x] = Wire.read();
  }

  // Store in case it is needed for responses
  receivedCommandSize = howMany;

  // Process the incomming command
  switch(commandReceived[0])
  {
    case STOP_COMMAND: // Stop Both Motors
      camera.StopPan();
      camera.StopTilt();
      break;

    case PAN_COMMAND: // Pan the camera
      // What is the sub command
      switch(commandReceived[1])
      {
        case PAN_LEFT:
          {
            camera.PanLeft();
            break;
          }
        case PAN_RIGHT:
          {
            camera.PanRight();
            break;
          }
        case PAN_TO:
          {
            // Combine two bytes into an int
            int angle = (commandReceived[2] << 8) | commandReceived[3];
            camera.MoveToPan(angle);
            break;
          }
        case PAN_STOP:
          {
            // Stop the Pan Axis Movement
            camera.StopPan();
            break;
          }
      }
      break;

    case TILT_COMMAND:
    // What is the sub command
      switch(commandReceived[1])
      {
        case TILT_UP:
          {
            camera.TiltUp();
            break;
          }
        case TILT_DOWN:
          {
            camera.TiltDown();
            break;
          }
        case TILT_TO:
          {
            // Combine two bytes into an int
            int angle = (commandReceived[2] << 8) | commandReceived[3];
            camera.MoveToTilt(angle);
            break;
          }
        case TILT_STOP:
          {
            // Stop tilt in current position
            camera.StopTilt();
            break;
          }
      }
      break;

    case HOME_COMMAND:
      // Return the camera to home
      camera.GoHome();
      break;

    case STATUS_COMMAND:
      // Wait for the request event to respond to this
      break;

    case SETTINGS_COMMAND:
      // Received information for new settings
      switch(commandReceived[1])
      {
        case SETTINGS_LIMITS:
          if(receivedCommandSize == 10)
          {
            // The user has requested we limit the motions of the different axis
            int minTiltAngle = (commandReceived[2] << 8) | commandReceived[3];
            int maxTiltAngle = (commandReceived[4] << 8) | commandReceived[5];
            int minPanAngle = (commandReceived[6] << 8) | commandReceived[7];
            int maxPanAngle = (commandReceived[8] << 8) | commandReceived[9];

            // Set the camera limits as requested (Absolutes will still constrain)
            camera.SetMinMaxTilt(minTiltAngle, maxTiltAngle);
            camera.SetMinMaxPan(minPanAngle, maxPanAngle);
          }
      }
  }
}

void SendFirmwareStatusResponse()
{
  // Version 1.0.0
  uint8_t response[5] { STATUS_COMMAND, STATUS_FIRMWARE, 1, 0, 0};
  Wire.write(response, sizeof(response));
}

void SendPositionStatusResponse()
{
  /* Response Format
  STATUS_COMMAND, STATUS_POSITION, PanMotorStatus, TiltMotorStatus, PanPosHByte, PanPosLByte, TiltPosHByte, TiltPosLByte
  */
  uint8_t response[12] { STATUS_COMMAND, STATUS_POSITION, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

  // Camera Status
  response[2] = camera.GetPanMotorStatus();
  response[3] = camera.GetTiltMotorStatus();

  // Send the current angles of the pan and tilt
  int panPosition = camera.GetCurrentPanAngle();
  response[4] = (panPosition >> 8) & 0xFF; // Extract the high byte
  response[5] = panPosition & 0xFF; 

  int tiltPosition = camera.GetCurrentTiltAngle();
  response[6] = (tiltPosition >> 8) & 0xFF;
  response[7] = tiltPosition & 0xFF;

  // Send Destination Values
  panPosition = camera.GetPanDestinationAngle();
  response[8] = (panPosition >> 8) & 0xFF; // Extract the high byte
  response[9] = panPosition & 0xFF; 

  tiltPosition = camera.GetTiltDestinationAngle();
  response[10] = (tiltPosition >> 8) & 0xFF;
  response[11] = tiltPosition & 0xFF;

  Wire.write(response, sizeof(response));
}