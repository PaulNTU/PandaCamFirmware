#include "HWPins.h"
#include "NTUTimer.h"
#include "PandaCam.h"

PandaCam camera = PandaCam();

NTUTimer timer = NTUTimer(true);

void setup() { 

  // ONLY ENABLE FOR DEBUGGING AS THIS WILL BREAK THE PAN MOTOR CONTROL on the PandaCam PCB!!!!
  //Serial.begin(9600);

  // Initialise the camera
  camera.InitCamera();
  //timer.Start(1000);

  // Start I2C Communications
  BeginI2c();

  // Run Motors
  //camera.PanLeft();
  //camera.TiltDown();
}

void loop() {
  // Update the camera to ensure it stops in the correct position.
  camera.UpdateCamera();

  delay(100);
  /*
  if(timer.Update())
  {  
    camera.PrintCameraDebug();
  }
  */
}
