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
    }else{
      if(SERIAL_ON) Serial.println(F("Filter started"));
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






bool zero(){
  //digitalWrite(ALL_GOOD_LED, LOW);
 
    if (SERIAL_ON) Serial.print(F("Zeroing Translation"));
    bool success_translation = zeroTranslation();
    if (!success_translation) {
        if (SERIAL_ON && digitalRead(ENABLE) == LOW) Serial.println(F(" Failed due to ENABLE being low | 0"));
        else if (SERIAL_ON) Serial.println(F(" Failed"));
        return success_translation;
    }
    if (SERIAL_ON) {
        Serial.println(F(" Success"));
        Serial.println(F("Zeroing Rotation"));
    }
    bool success_rotation = zeroRotation();
    if (!success_rotation) {
        if (SERIAL_ON && digitalRead(ENABLE) == LOW) Serial.println(F(" Failed due to ENABLE being low | 0"));
        else if (SERIAL_ON) Serial.println(F(" Failed"));
        return success_rotation;
    }
    bool success = success_rotation && success_translation;
    if (SERIAL_ON) Serial.println(F(" Success"));
    digitalWrite(ALL_GOOD_LED, HIGH);
    return success;
}



bool zeroRotation(){
    for (int i = 0; i<SENSOR_DEBOUNCE*2; i++) rotational_zero.sensorMonitor();
    int timer_serial = 0;
    int timer_bounce = 0;
    int count = 0;
    //rotation_stepper.setCurrentPositionAsHomeAndStop();
    rotation_stepper.setSpeedInRevolutionsPerSecond(HOME_SPEED_ROTATION);
    double distance = 3*DIRECTIONS[board_ID][ROTATION];
    Serial.Print(F("Rotation is: ");
    Serial.Print(F(distance));


    rotation_stepper.moveToHomeInRevolutions(-1, 20, 50, ROTATION_SENSOR);//ROTATION ZERO HERE 
    
    
    rotation_stepper.setTargetPositionInRevolutions(distance);
    while(!rotation_stepper.processMovement() && rotational_zero.sensorActive()){ //if sensor is pressed, rotate until un pressed
        if (millis() - timer_bounce > 1) {
            timer_bounce = millis();
            rotational_zero.sensorMonitor();
        }
        if (SERIAL_ON && millis() - timer_serial > 1000) {
            timer_serial = millis();
            Serial.print(".");
        }
        if (digitalRead(ENABLE) == LOW) {
            //rotation_stepper.emergencyStop();
            Serial.print(F(" Failed due to ENABLE being low | 1"));
            return false;
        }
    }
    while(!rotation_stepper.processMovement() && !rotational_zero.sensorActive()){
        if (millis() - timer_bounce > 1) {
            timer_bounce = millis();
            rotational_zero.sensorMonitor();
        }
        if (SERIAL_ON && millis() - timer_serial > 1000) {
            timer_serial = millis();
            Serial.print(".");
        }
        if (digitalRead(ENABLE) == LOW) {
            //rotation_stepper.emergencyStop(false);
            Serial.print(F(" Failed due to ENABLE being low | 2"));
            return false;
        }
    }
    if (rotational_zero.sensorActive()) {
        //rotation_stepper.setCurrentPositionAsHomeAndStop();
         rotation_stepper.moveToHomeInRevolutions(-1, HOME_SPEED_ROTATION, 1, ROTATION_DRIVER_ZERO);
        return true;
    } else {
       // rotation_stepper.emergencyStop();
        Serial.print(F(" Failed due to rotational_zero.sensorActive() being false | 0"));
        return false;
    }
    
}


bool zeroTranslation(){
    for (int i = 0; i<SENSOR_DEBOUNCE*2; i++) translational_zero.sensorMonitor();
    int timer_serial = millis();
    int timer_bounce = millis();
    //translation_stepper.setCurrentPositionAsHomeAndStop();
    
    
    translation_stepper.setSpeedInMillimetersPerSecond(HOME_SPEED_TRANSLATION);
    double distance = DIRECTIONS[board_ID][TRANSLATION]*-1*MAX_TRANSLATIONS[board_ID];
    Serial.print(F("The Distance is: "));
    Serial.println(distance);

    translation_stepper.moveToHomeInMillimeters(-1, HOME_SPEED_TRANSLATION, distance, TRANSLATION_DRIVER_ZERO);

    translation_stepper.setTargetPositionInMillimeters(distance);
    while(!translation_stepper.processMovement() && !translational_zero.sensorActive()){
        if (millis() - timer_bounce > 1) {
            timer_bounce = millis();
            translational_zero.sensorMonitor();
        }

        if (SERIAL_ON && (millis() - timer_serial > 1000)) {
            timer_serial = millis();
            Serial.print(".");
        }

        if (digitalRead(ENABLE) == LOW) {
            //translation_stepper.emergencyStop(false);
            Serial.print(F(" Failed due to ENABLE being low | 3"));
            return false;
        }

    }
    if (translational_zero.sensorActive()) {
       // translation_stepper.setCurrentPositionAsHomeAndStop();
       translation_stepper.moveToHomeInMillimeters(-1, HOME_SPEED_TRANSLATION, 1, TRANSLATION_DRIVER_ZERO);
        translation_stepper.setTargetPositionInMillimeters(DIRECTIONS[board_ID][TRANSLATION]);                  //Single mm change... weird
        while(!translation_stepper.processMovement());
        //translation_stepper.setCurrentPositionAsHomeAndStop();
        translation_stepper.moveToHomeInMillimeters(-1, HOME_SPEED_TRANSLATION, 1, TRANSLATION_DRIVER_ZERO);
        return true;
    } else {
       // translation_stepper.emergencyStop(false);
        Serial.print(F("Failed due to translational_zero.sensorActive() being false | 0"));
        return false;
    }
}

void setControl(){
    if (state == RUNNING) {
        if (translation_desired > MAX_TRANSLATIONS[board_ID]) translation_desired = MAX_TRANSLATIONS[board_ID];
        if (translation_desired < 0) translation_desired = 1;
        if (translation_stepper.getCurrentPositionInMillimeters()*DIRECTIONS[board_ID][TRANSLATION] < -1) {
            if (SERIAL_ON && SERIAL_STATES){
                Serial.print(F("Went Negative: "));
                Serial.println(translation_stepper.getCurrentPositionInMillimeters()*DIRECTIONS[board_ID][TRANSLATION]);
            }
            state = ZERO;
        }
        else {
            translation_stepper.setTargetPositionInMillimeters(DIRECTIONS[board_ID][TRANSLATION]*translation_desired);
            rotation_stepper.setTargetPositionInRevolutions(DIRECTIONS[board_ID][ROTATION]*rotation_desired);
        }
    }
}


















