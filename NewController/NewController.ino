#include <SPI.h>
#define CAN_2515
#include "mcp2515_can.h"
#include <FlexyStepper.h>
#include "Controller_Constants.h"
#include "Sensor_Debounce.h"

//States
#define ZERO 0
#define RUNNING 1
#define SHORT_CAN_WAIT 2
#define EMERGENCY_STOP 3
#define DISABLING 4
#define STARTING 5
#define STOP_SWITCH 6
#define LARGE_CAN_DELAY 7
#define LONG_CAN_WAIT 8

const bool SERIAL_ON = true;
const bool SERIAL_MESSAGES = true;
const bool SERIAL_STATES = true;
//E-STOP function removed, Hardware determines if stepper drivers will drive motors, if ENABLE == LOW then no run

//Define steppers
FlexyStepper translation_stepper;
FlexyStepper rotation_stepper;

//Sensor (Zeroing switches) Debouncing
Sensor_Debounce rotational_zero = Sensor_Debounce(ROTATION_DRIVER_ZERO, SENSOR_DEBOUNCE, INPUT_PULLUP, LOW);
Sensor_Debounce translational_zero = Sensor_Debounce(TRANSLATION_DRIVER_ZERO, SENSOR_DEBOUNCE, INPUT_PULLUP, LOW);


bool zero();
void setControl();
void CANSender();
void CANReceiver();
void evaluateState();

int board_ID=0; //This can be changed between flashing different boards so we do not use two pins

int state = ZERO; //first state
bool emergency_stop = false; 

double translation_measured = 0;
double rotation_measured = 0;
double translation_desired = 0;
double rotation_desired = 0;

unsigned long receive_time = 0;
double send_time = 0;

typedef union {
    float value;
    byte bytes[sizeof(float)];
} FLOAT_BYTE_UNION;

const int SPI_CS_PIN = 9; //setting CS pin for arduino CAN shields
mcp2515_can CAN(SPI_CS_PIN); //setting CS pin Default is 9 

void commStart(){

  SERIAL_PORT_MONITOR.begin(115200);
  while (CAN_OK != CAN.begin(CAN_500KBPS)) {             // init can bus : baudrate = 500k
        SERIAL_PORT_MONITOR.println(F("CAN init fail, retry..."));
        delay(100);
    }
    SERIAL_PORT_MONITOR.println(F("CAN init ok!"));
  while(!init_Filt(0b1<<board_ID,0,0b10100000 + (1 << board_ID) + 0x1fffff00){
//init_Filt(0 or 1 for mask: 0 to 5 for filter, 0 for standard frame: 1 for extended, content of mask or filter)
        if (SERIAL_ON) Serial.println(F("filter failed!"));
        //digitalWrite(ALL_GOOD_LED, LOW);
        delay(1000);
    }
}



void StepMotorSetup(){//Connecting stepper motor pins, setting constants, speed, accel, etc


  translation_stepper.connectToPins(TRANSLATION_DRIVER_PULSE,TRANSLATION_DRIVER_DIR);
  rotation_stepper.connectToPins(ROTATION_DRIVER_PULSE, ROTATION_DRIVER_DIR);

  translation_stepper.setStepsPerMillimeter(STEP_PULSE_TRANSLATION_CONVERSION[board_ID]);
  translation_stepper.setAccelerationInMillimetersPerSecondPerSecond(MAX_ACCELERATION_TRANSLATION);

  rotation_stepper.setStepsPerRevolution(STEP_PULSE_ROTATION_CONVERSION);
  rotation_stepper.setAccelerationInRevolutionsPerSecondPerSecond(MAX_ACCELERATION_ROTATION);

}


void setup() {
  commStart();
  StepMotorSetup();
  // put your setup code here, to run once:
  delay(3000);
  receive_time=millis();
  send_time=millis();


  

}

void loop() {
  // put your main code here, to run repeatedly:

  //evaluateState();
  //CANReceiver();
  //setControl();
  //if(millis()-send_time>COM_DELAY){
  //send_time=millis();
  //CANSender();
  //}









}



bool start_CAN(int BAUD){


  
}




























