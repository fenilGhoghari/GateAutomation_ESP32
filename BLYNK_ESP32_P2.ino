
#define BLYNK_TEMPLATE_ID "TMPLZxWFXIao"
#define BLYNK_DEVICE_NAME "PROJECT 1"

#define BLYNK_FIRMWARE_VERSION        "0.1.0"

#define BLYNK_PRINT Serial

// SPI
#include "FS.h" // Your laptop is running very slow = I kmow, something is wrong. this is i7 laptop, may be thjere is some bugs in your laptop need to check
#include "SPIFFS.h"

/* You only need to format SPIFFS the first time you run a
   test or else use the SPIFFS plugin to create a partition
   https://github.com/me-no-dev/arduino-esp32fs-plugin */
#define FORMAT_SPIFFS_IF_FAILED true


#define APP_DEBUG

// Uncomment your board, or configure a custom board in Settings.h for WIFI resetting Wi-Fi configuration.
#define ADDED_PUSH_BUTTON_MANUALLY   // here we have used pin no 16 for push button if you want to change ythen you can change from Setting.h section.
//#define USE_WROVER_BOARD
//#define USE_TTGO_T7
//#define USE_TTGO_T_OI
//#define USE_ESP32C3_DEV_MODULE
//#define USE_ESP32S2_DEV_KIT



#include "BlynkEdgent.h"
// PIR
int ledPinG = 25;                // choose the pin for the LED for Power ON - GREEN
int ledPinR = 27;                // choose the pin for the LED for Power OFF - RED
int ledPinY = 13;                // choose the pin for the LED for dtection of motion by PIR - YELLOW
int ledPinB = 32;                // choose the pin for the LED for configuration - BLUE
int inputPin = 2;                // choose the input pin (for PIR sensor)
int pirState = LOW;              // we start, assuming no motion detected
int val1;                        // variable for reading the pin status


// Flash esp32

// esptool.py --chip esp32 --port /dev/ttyUSB0 erase_flash

// MOTOR
#include <AccelStepper.h>

long receivedSteps = 0; //Number of steps
long receivedSpeed = 0; //Steps / second
long receivedAcceleration = 0; //Steps / second^2
long receivedAcceleration_stop = 0; //Steps / second^2
long receivedSteps_from_configuration;  // step we get after pressing stop button it will be -ve or +ve bcz it's depend on motor operation
long configuration_steps;  // this is final steps and it will be in + ve because here we are getting only positive steps beacuse it is used to convert negative into positive and positive into positive

//***************************************SPEED FOR CONFIGURATION AND NORMAL MODE ************************************

long stepper_speed = 500;  // THE SPEED WILL SAME IN BOTH MODES

// ************************************CONFIGURATION MODE STEPS**********************************
long Configuration_steps = 100000;  // MAX STEPS THE WILL ONLY TRAVEL THIS STEPS IN CONFIGURATION MODE



#define dirPin 2   // You can change the pin no from here
#define stepPin 15
#define motorInterfaceType 1

AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin);
int pinValue0;
int pinValue1;
int pinValue2;
int pinValue3;
int pinValue4;


//-------------------------------------------------------------------------------
int directionMultiplier = 1; // = 1: positive direction, = -1: negative direction
bool newData, runallowed = false; // booleans for new data from serial, and runallowed flag

bool configuration = false; // rotation configuration flag
bool state = true; // rotation configuration flag
bool state1 = true; // rotation configuration flag
// push button
//const int PushButton = 16;

BLYNK_WRITE(V0)
{
  pinValue0 = param.asInt();
  Serial.println(pinValue0);
  Serial.println("v0 ");
  //digitalWrite(15, pinValue);
}
BLYNK_WRITE(V1)
{
  pinValue1 = param.asInt();
  Serial.println(pinValue1);
  Serial.println("v1 ");
  //digitalWrite(15, pinValue);
}
BLYNK_WRITE(V2)
{
  pinValue2 = param.asInt();
  Serial.println(pinValue2);
  Serial.println("v2 ");
  //digitalWrite(15, pinValue);
}
BLYNK_WRITE(V3)
{
  pinValue3 = param.asInt();
  Serial.println(pinValue3);
  Serial.println("v3 ");
}
BLYNK_WRITE(V4)
{
  pinValue4 = param.asInt();
  Serial.println(pinValue4);
  Serial.println("v4 ");
}
void setup()
{
  //enterConfigMode();
  // pinMode(15, OUTPUT);

  pinMode(ledPinG, OUTPUT);      // declare LED as output
  pinMode(ledPinY, OUTPUT);      // declare LED as output
  pinMode(ledPinR, OUTPUT);      // declare LED as output
  pinMode(ledPinB, OUTPUT);      // declare LED as output
  pinMode(inputPin, INPUT);     // declare sensor as input

  // pinMode(PushButton, INPUT_PULLUP);
  stepper.setMaxSpeed(1000);
  stepper.setAcceleration(500);
  Serial.begin(115200);
  delay(100);


  if (!SPIFFS.begin(FORMAT_SPIFFS_IF_FAILED)) {
    Serial.println("SPIFFS Mount Failed");
    return;
  }

  BlynkEdgent.begin();
 //stepper.setEnablePin(23); // Connect pin no23 with ENA-
  stepper.disableOutputs(); //disable outputs, so the motor is not getting warm (no current)

}


// Spiffs READ
String readFile(fs::FS &fs, const char *path)
{
  // Serial.printf("Reading file: %s\r\n", path);

  String ret = "";

  /*if (!fs.exists(path)){
    return ret;
    }*/

  File file = fs.open(path);

  if (!file || file.isDirectory())
  {
   // Serial.println("- failed to open file for reading");
    return ret;
  }

  //Serial.println("- read from file:");
  while (file.available())
  {
    char c = file.read();
    //Serial.write(c);
    ret += c;
  }
  //Serial.println("");
  file.close();
  return ret;
}

// Spiffs WRITE

void writeFile(fs::FS &fs, const char *path, const char *message)
{
  Serial.printf("Writing file: %s\r\n", path);

  File file = fs.open(path, FILE_WRITE);
  if (!file)
  {
    Serial.println("- failed to open file for writing");
    return;
  }
  if (file.print(message))
  {
    //Serial.println("- file written");
  }
  else
  {
    //Serial.println("- write failed");
  }
  file.close();
}
void ROTATE_CONFIGURATION()
{

  if (pinValue4 == 0) // Power OFF
  {
    digitalWrite(ledPinB, LOW);
    state1 = true;
    stepper.stop(); //stop motor
    //digitalWrite(ledPinR, HIGH);
  }

  if (pinValue4 == 1) //Configuration mode ON
  {
    state1 = false;
    // Blynk.virtualWrite(V0, 0);
    // pinValue0 = 0 ;

    digitalWrite(ledPinB, HIGH);
    configuration = true; //this creates a flag


  }

  if (configuration == true) //if we received something (see above)
  {

    //digitalWrite(ledPinR, LOW);
    // Blynk.virtualWrite(V0, 0);
    // pinValue0 = 0 ;
    // delay(100);

    if (pinValue1 == 1) //OPENING
    {
      stepper.setCurrentPosition(0); // reset position
      Serial.println("OPEN_Configuration "); //print the action
      //stepper.setAcceleration(receivedAcceleration);
      receivedSteps = Configuration_steps;
      receivedSpeed = stepper_speed;
      directionMultiplier = 1; //We define the direction
      RotateRelative_configuration();
      delay(200);
      Blynk.virtualWrite(V1, 0);
      pinValue1 = 0 ;
    }

    //START - CLOSE
    if (pinValue2 == 1) //CLOSING - Rotates the motor in the opposite direction as opening
    {
      stepper.setCurrentPosition(0); // reset position
      Serial.println("CLOSE_Configuration "); //print action
      stepper.setAcceleration(receivedAcceleration);
      receivedSteps = Configuration_steps;
      receivedSpeed = stepper_speed;
      directionMultiplier = -1; //We define the direction
      RotateRelative_configuration();
      delay(200);
      Blynk.virtualWrite(V2, 0);
      pinValue2 = 0 ;
    }

    //STOP - STOP
    if (pinValue3 == 1) //immediately stops the motor
    {
      StopTheMotor_Configuration();
    }



  }
  //after we went through the above tasks, newData becomes false again, so we are ready to receive new commands again.
  configuration = false;



}

void RotateRelative_configuration()
{
  //We move X steps from the current position of the stepper motor in a given direction.
  //The direction is determined by the multiplier (+1 or -1)

  runallowed = true; //allow running - this allows entering the RunTheMotor() function.
  stepper.setMaxSpeed(receivedSpeed); //set speed
  stepper.move(directionMultiplier * receivedSteps); //set relative distance and direction

  Serial.println("Target Position"); //print action
  Serial.println(stepper.targetPosition()); //print action
  Serial.println("current position , when stop button is pressed"); //print action
  Serial.println(stepper.currentPosition()); //print action
  Serial.println("Distance to go"); //print action
  Serial.println(stepper.distanceToGo()); //print action
  Serial.println("SPEEED"); //print action
  Serial.println(stepper.speed()); //print action

}



void StopTheMotor_Configuration() // motor configuration mode stop function
{
  if (runallowed == true)
  {
    //receivedAcceleration_stop = 1200;
    //stepper.setAcceleration(receivedAcceleration_stop);
    // stepper.setCurrentPosition(0);
    stepper.stop(); //stop motor
    Serial.println("Target Position"); //print action
    Serial.println(stepper.targetPosition()); //print action
    Serial.println("current position , when stop button is pressed"); //print action
    Serial.println(stepper.currentPosition()); //print action
    Serial.println("Distance to go"); //print action
    Serial.println(stepper.distanceToGo()); //print action
    Serial.println("SPEEED"); //print action
    Serial.println(stepper.speed()); //print action

    //*****************--------------------------******************************

    receivedSteps_from_configuration = stepper.currentPosition();



    Serial.println("Motor steps after configuration mode"); //print action
    Serial.println(receivedSteps_from_configuration); //print action

    //*****************--------------------------******************************
    if (receivedSteps_from_configuration > 1)
    {
      configuration_steps = 1 * receivedSteps_from_configuration;
    }

    //*****************--------------------------******************************
    if (receivedSteps_from_configuration < 1)
    {
      configuration_steps = -1 * receivedSteps_from_configuration;
    }

    Serial.println("Final configuration steps "); //print action
    Serial.println(configuration_steps); //print action
    writeFile(SPIFFS, "/FINAL_STEPS.txt", String(configuration_steps).c_str());

    //********************----------------------*******************************
    stepper.setCurrentPosition(0);

    //stepper.runToPosition();
    runallowed = false; //disable running

    // _currentPos
  }
}


void loop() {
  //  int Push_button_state = digitalRead(PushButton);
  //val1 = !digitalRead(inputPin);  // read input value   // PIR READING
  //Serial.println(val1);

  BlynkEdgent.run();
  if (state == true)
  {
    ROTATE_CONFIGURATION(); // check steps configuration
  }
  if (state1 == true)
  {
    checkInput(); //check serial port for new commands
  }
  continuousRun(); //method to handle the motor


}

void continuousRun() //method for the motor
{
  receivedAcceleration = 500; // This is the starting acceleration

  if (runallowed == true)
  {

    stepper.enableOutputs(); //enable pins
    stepper.run(); //step the motor (this will step the motor by 1 step at each loop)

  }

  else //program enters this part if the runallowed is FALSE, we do not do anything
  {
    //  digitalWrite(ledPinR, HIGH);
    return;

  }
}

void StopTheMotor() //function for the motor
{
  if (runallowed == true)
  {
    receivedAcceleration_stop = 1200;
    stepper.setAcceleration(receivedAcceleration_stop);
    stepper.stop(); //stop motor
    Serial.println("Target Position"); //print action
    Serial.println(stepper.targetPosition()); //print action
    Serial.println("current position , when stop button is pressed"); //print action
    Serial.println(stepper.currentPosition()); //print action
    Serial.println("Distance to go"); //print action
    Serial.println(stepper.distanceToGo()); //print action
    Serial.println("SPEEED"); //print action
    Serial.println(stepper.speed()); //print action
    stepper.runToPosition();
    runallowed = false; //disable running

    // _currentPos
  }
}

void checkInput() //method for receiving the commands
{

   // val1 = !digitalRead(inputPin);  // read input value   // PIR READING
  val1 = 1;
  if (pinValue0 == 0) // Power OFF
  {
    state = true;
    digitalWrite(ledPinG, LOW);
    digitalWrite(ledPinR, HIGH);
    //stepper.disableOutputs(); //disable power
  }
  if (pinValue0 == 1) //Power ON
  {

    state = false;
    // Blynk.virtualWrite(V4, 0);
    // pinValue4 = 0;


    digitalWrite(ledPinG, HIGH);
    digitalWrite(ledPinR, LOW);
    newData = true; //this creates a flag
    // Make configuration mode OFF in power section
    //
    //

  }

  if (newData == true) //if we received something (see above)
  {
    String ritik =   readFile(SPIFFS, "/FINAL_STEPS.txt");

    if (pinValue1 == 1) //OPENING
    {

      Serial.println("OPEN "); //print the action
      // stepper.setAcceleration(receivedAcceleration);
      Serial.println("Final configuration steps"); //print action
      Serial.println(configuration_steps); //print action

      receivedSteps = ritik.toInt();
      receivedSpeed = stepper_speed;
      directionMultiplier = 1; //We define the direction
      RotateRelative();
      delay(200);
      Blynk.virtualWrite(V1, 0);
      pinValue1 = 0 ;
    }

    //START - CLOSE
    if (pinValue2 == 1) //CLOSING - Rotates the motor in the opposite direction as opening
    {
      Serial.println("CLOSE "); //print action
      // stepper.setAcceleration(receivedAcceleration);
      receivedSteps = 0;
      receivedSpeed = stepper_speed;
      directionMultiplier = -1; //We define the direction
      RotateRelative();
      delay(200);
      Blynk.virtualWrite(V2, 0);
      pinValue2 = 0 ;
    }

    //STOP - STOP
    if (pinValue3 == 1) //immediately stops the motor
    {
      StopTheMotor();
    }

    if (val1 == 0) //PIR DETECT
    {
      //  runallowed = true; //disable running
      digitalWrite(ledPinY, HIGH);
      Serial.println("MOTION DETECTETD "); //print action
      Serial.println("Target Position"); //print action
      Serial.println(stepper.targetPosition()); //print action
      Serial.println("Distance to go"); //print action
      Serial.println(stepper.distanceToGo()); //print action
      long remain_distance = stepper.distanceToGo();
      StopTheMotor();
      digitalWrite(ledPinY, HIGH);
      delay(3000);
      digitalWrite(ledPinY, LOW);
      Serial.println("jkchasdjhasdgasdjhgfgv"); //print action

      if (pinValue1 == 1) //OPENING
      {
        directionMultiplier = 1; //We define the direction
        receivedSteps = ritik.toInt(); //value for the steps
        Serial.println("PIR _Forward");
      }
      if (pinValue2 == 1) //OPENING
      {
        directionMultiplier = -1; //We define the direction
        receivedSteps = 4000; //value for the steps
        Serial.println("PIR _Reverse");
      }
      receivedSpeed = stepper_speed; //value for the speed
      RotateRelative();
    }

  }
  //after we went through the above tasks, newData becomes false again, so we are ready to receive new commands again.
  newData = false;


}

void RotateRelative()
{
  //We move X steps from the current position of the stepper motor in a given direction.
  //The direction is determined by the multiplier (+1 or -1)

  runallowed = true; //allow running - this allows entering the RunTheMotor() function.
  stepper.setMaxSpeed(receivedSpeed); //set speed
  stepper.moveTo(directionMultiplier * receivedSteps); //set relative distance and direction


  Serial.println("Target Position"); //print action
  Serial.println(stepper.targetPosition()); //print action
  Serial.println("current position , when stop button is pressed"); //print action
  Serial.println(stepper.currentPosition()); //print action
  Serial.println("Distance to go"); //print action
  Serial.println(stepper.distanceToGo()); //print action
  Serial.println("SPEEED"); //print action
  Serial.println(stepper.speed()); //print action
}
