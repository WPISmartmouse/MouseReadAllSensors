
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Wire.h>
#include <Encoder.h>
#include <RegulatedMotor.h>
#include "KinematicController.h"
#include <VL6180X.h>
#include "Mouse16.h"
#include <math.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define OLED_RESET 5
Adafruit_SSD1306 display(OLED_RESET);

#define BNO055_SAMPLERATE_DELAY_MS (100)  

Adafruit_BNO055 bno = Adafruit_BNO055();
Encoder encL;
Encoder encR;
long encLCount = 0;
long encRCount = 0;

RegulatedMotor motL(&encLCount, MOTOR1B, MOTOR1A);
RegulatedMotor motR(&encRCount, MOTOR2B, MOTOR2A);
KinematicController kc(&motL,&motR,1,-1,78.3f,31.71f,(int)(12*75.81));

Mouse16 dumbmouse = Mouse16();

// Useful code below

// where the sensor is plug into (look at the board), an arbitrary i2c address (do not conflict with other stuff)
VL6180X l30(VL6180EN1,0x41);
VL6180X fl0(VL6180EN2,0x42);
VL6180X r30(VL6180EN3,0x43);

int state = 0;

void setup() {
  analogReadResolution(12);
  dumbmouse.init();
  encL.init(ENCODER1A, ENCODER1B);
  encR.init(ENCODER2A, ENCODER2B);


    
  Serial.begin(115200); //Start Serial at 115200bps
  Wire.begin(); //Start I2C library
  Wire1.begin();
  delay(100); // delay .1s
  display.clearDisplay();  
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3D (for the 128x64)
  display.display();
  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    display.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    display.display();
    while(1);
  }

  delay(200);

  bno.setExtCrystalUse(true);

  motL.setPID(0.09,0.01,0.01,0);
  motR.setPID(0.09,0.01,0.01,0);
  motL.setSampleTime(10000);
  motR.setSampleTime(10000);
  kc.setAcceleration(2000,2000,2000,2000);


  display.clearDisplay();  
  // critical code:
  display.println("VL init: ");

  // ====================
  // initialization involves changing the i2c addr if it is not set
  // The EN pins should be have been written low in dumbmouse.init()
  // should see 000. if the return is not zero, the electrical connection is likely to be broken.
  display.print(l30.initMouse());  
  display.print(fl0.initMouse());
  display.print(r30.initMouse());

  display.display();

  // so you can see the screen
  delay(1000);


  // range interval 50 have been working well. lower value = faster update. too fast will make the sensor stuck.
  int range_interval = 50;

  l30.startRangeContinuous(range_interval);
  fl0.startRangeContinuous(range_interval);
  r30.startRangeContinuous(range_interval);
  // ====================
  
}


void loop() { 

  display.clearDisplay();  
  display.setCursor(0,0);

  float voltage = analogRead(BATTERYSENSE) * (3.3/4095) * (49/10.0) / 3.0;


  display.print("batt ");
  display.println(voltage);



  //======================

  // this is how you do it. return val is in mm. out of range is 255.
  display.print("Dist ");
  display.println(l30.readRangeContinuous());
  display.println(fl0.readRangeContinuous());
  display.println(r30.readRangeContinuous());

  //======================


  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  /* Display the floating point data */
  display.print("X: ");
  display.println(euler.x());
  display.print(" Y: ");
  display.println(euler.y());
  display.print(" Z: ");
  display.println(euler.z());
  
  display.display();
  
};

double constrainAngle(double x){
    x = fmod(x,360);
    if (x < 0)
        x += 360;
    return x;
}

