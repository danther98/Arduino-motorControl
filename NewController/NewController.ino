#include <SPI.h>
#define CAN_2515
#include "mcp2515_can.h"
#include <FlexyStepper.h>
#include "Controller_Constants.h"
#include "Sensor_Debounce.h"
//#include <CAN.h>

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
        SERIAL_PORT_MONITOR.println(("CAN init fail, retry..."));
        delay(100);
    }
    SERIAL_PORT_MONITOR.println(F("CAN init ok!"));
  while(!CAN.init_Filt(0b1<<board_ID,0,0b10100000 + (1 << board_ID) + 0x1fffff00)){
  //init_Filt(0 or 1 for mask: 0 to 5 for filter, 0 for standard frame: 1 for extended, content of mask or filter)
        if (SERIAL_ON) Serial.println(F("filter failed!"));
        //digitalWrite(ALL_GOOD_LED, LOW);
        delay(1000);
    }
      if(SERIAL_ON) Serial.println(F("Filter started"));
    
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

  evaluateState();
  CANReceiver();
  setControl();
  if(millis()-send_time>COM_DELAY){
  send_time=millis();
  CANSender();
  }



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
    Serial.print(F("Rotation is: "));
    Serial.print((distance));


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


void CANSender(){
    FLOAT_BYTE_UNION translation_measured_f;
    FLOAT_BYTE_UNION rotation_measured_f;
    translation_measured_f.value = (float)(DIRECTIONS[board_ID][TRANSLATION]*translation_stepper.getCurrentPositionInMillimeters());
    rotation_measured_f.value = (float)(DIRECTIONS[board_ID][ROTATION]*rotation_stepper.getCurrentPositionInRevolutions()*DEGREES_PER_REVOLUTION);
    if (SERIAL_ON && SERIAL_MESSAGES) Serial.print("Sent: (packet: 0b");
    if (state == RUNNING) {
      unsigned char dat[8]={0,0,0,0,0,0,0,0};
      /* for (int i = sizeof(float)-1; i >= 0; i--){
            CAN.write(translation_measured_f.bytes[i]);
       }
 */
      //replace packet with array? create own packet functions?
       // CAN.beginPacket(0b10010000 + (1 << board_ID));// start sequence of sending packet, need to see if begin packet will work with new lib. 
        for (int i = sizeof(float)-1; i >= 0; i--){
            dat[i]=translation_measured_f.bytes[i]; // these are to write to the packet 
        }
        for (int i = sizeof(float)-1; i >= 0; i--){
            dat[i+4]=rotation_measured_f.bytes[i];// these are to write to the packet 
        }
        CAN.sendMsgBuf(0b10010000 + (1 << board_ID),0,8,dat);

       // CAN.endPacket(); //ends sequence of sending packet, so replace with CAN.sendMsgBuf()?
        if (SERIAL_ON && SERIAL_MESSAGES) Serial.print(0b10010000 + (1 << board_ID), BIN);

        
   
    else if (state == SHORT_CAN_WAIT || state == EMERGENCY_STOP || state == STOP_SWITCH || state == ZERO) {
       // CAN.beginPacket((1 << board_ID) + 0b10000000);
        for (int i = sizeof(float)-1; i >= 0; i--){
            dat[i]=translation_measured_f.bytes[i]; // these are to write to the packet 
        }
        for (int i = sizeof(float)-1; i >= 0; i--){
            dat[i+4]=rotation_measured_f.bytes[i];// these are to write to the packet 
        }
        CAN.sendMsgBuf((1 << board_ID) + 0b10000000,0,8,dat);
      
        if (SERIAL_ON && SERIAL_MESSAGES) Serial.print((1 << board_ID) + 0b10000000, BIN);
    } else {
        if (SERIAL_ON && SERIAL_MESSAGES) Serial.println("NON PRINT STATE");
    }
    if (SERIAL_ON && SERIAL_MESSAGES) {
        Serial.print(" ROTATION: ");
        Serial.print(rotation_measured_f.value);
        Serial.print(" Translation: ");
        Serial.println(translation_measured_f.value);
    }
  }
}
int received_zero = false; //The goal rod will receive phantom zero commands and needs to be stopped

void CANReceiver(){

  if(CAN.checkReceive()){
     receive_time = millis();
        FLOAT_BYTE_UNION translation_desired_f;
        FLOAT_BYTE_UNION rotation_desired_f;
        if (SERIAL_ON && SERIAL_MESSAGES) {
            Serial.print("Recv: (packet: 0b");
            Serial.print(CAN.getCanId(), BIN);
            Serial.print(" ");
        }
  

 if ((CAN.getCanId() & (0x1fffff00 + 0b11111111)) == 0b00001111) {
            if (SERIAL_ON) Serial.println("E STOP message received");
            emergency_stop = true;
            evaluateState();
            return;
        } //else {
           // emergency_stop = false;
        //}


     if ((CAN.getCanId() & (0x1fffff00 + 0b11100000)) == 0b01000000) {
            if (SERIAL_ON) Serial.println("Zero message received");
            state = ZERO;
            if (received_zero) state = LARGE_CAN_DELAY;
            received_zero = true;
            evaluateState();
            return;
        }

 if ((CAN.getCanId() & (0x1fffff00 + 0b11110000)) == 0b10000){
            received_zero = false;
            int rotation_index = sizeof(float)-1;
            int translation_index = sizeof(float)-1;
            bool data_received = false;
            while(CAN.checkReceive()){
                data_received = true;
                unsigned char readDatTrans[4]={0,0,0,0};
                unsigned char readDatRot[4]={0,0,0,0};
                if (translation_index >= 0){
                    translation_desired_f.bytes[translation_index--] = (byte)CAN.readMsgBuf(4,readDatTrans);
                } else if (rotation_index >= 0) {
                    rotation_desired_f.bytes[rotation_index--] = (byte)CAN.readMsgBuf(4,readDatRot);
                } else {
                    if (SERIAL_ON && SERIAL_MESSAGES) Serial.println("DATA DROPPED)");
                    return;
                }
            }
  if (data_received){
                if (!((translation_index < 0) && (rotation_index < 0))) {
                    if (SERIAL_ON && SERIAL_MESSAGES) Serial.println("LESS THAN 8 BYTES RECEIVED)");
                    return;
                }
                rotation_desired = (double)rotation_desired_f.value/DEGREES_PER_REVOLUTION;
                translation_desired = (double)translation_desired_f.value;
                if (SERIAL_ON && SERIAL_MESSAGES){
                    Serial.print(" Rotation: ");
                    Serial.print(rotation_desired*DEGREES_PER_REVOLUTION);
                    Serial.print(" Translation: ");
                    Serial.print(translation_desired);
                }
            }
            if (SERIAL_ON && SERIAL_MESSAGES) Serial.println(")");
        }
    
  }
}

void evaluateState(){
    if (SERIAL_ON && SERIAL_STATES) {
        //last_state = state;
        Serial.print("State: ");
        Serial.println(state);
    }
    if (state == ZERO){
        digitalWrite(ALL_GOOD_LED, LOW);
        //translation_stepper.emergencyStop(false);
        //rotation_stepper.emergencyStop(false);
        //if (translation_stepper.isStartedAsService()) translation_stepper.stopService(); --I'm not sure we need these 
        //if (rotation_stepper.isStartedAsService()) rotation_stepper.stopService();--I'm not sure we need these 
        if (zero()) {
            translation_stepper.setSpeedInMillimetersPerSecond(MAX_SPEED_TRANSLATION);
            rotation_stepper.setSpeedInRevolutionsPerSecond(MAX_SPEED_ROTATION);
           // translation_stepper.startAsService(STEPPER_CORE);--I'm not sure we need these 
           // rotation_stepper.startAsService(STEPPER_CORE);--I'm not sure we need these 
            receive_time = millis();
            state = SHORT_CAN_WAIT;
        }
    } else if (state == RUNNING){
        digitalWrite(ALL_GOOD_LED, HIGH);
        if (emergency_stop || (millis() - receive_time > MAX_COM_DELAY) || (digitalRead(ENABLE) == LOW)) {
            state = DISABLING;
            if (SERIAL_ON && SERIAL_STATES && emergency_stop) Serial.println("Emergency Stop Command Active");
            if (SERIAL_ON && SERIAL_STATES && ((millis() - receive_time) > MAX_COM_DELAY)) Serial.println("COM timeout");
            if (SERIAL_ON && SERIAL_STATES && (digitalRead(ENABLE) == LOW)) Serial.println("Emergency Stop Button Pressed");
        }
    } else if (state == SHORT_CAN_WAIT){
        digitalWrite(ALL_GOOD_LED, LOW);
        if (emergency_stop) state = EMERGENCY_STOP;
        else if (digitalRead(ENABLE) == LOW) state = STOP_SWITCH;
        else if ((millis() - receive_time) < MAX_COM_DELAY) {
            state = STARTING;
            if (SERIAL_ON && SERIAL_STATES) Serial.println("COM timeout not exceeded");
        }
        if ((millis() - receive_time) > MAX_COM_DELAY*10) state = LARGE_CAN_DELAY;
        if (SERIAL_ON && SERIAL_STATES && (digitalRead(ENABLE) == LOW)) Serial.println("Emergency Stop Button Pressed");
        if (SERIAL_ON && SERIAL_STATES && ((millis() - receive_time) > MAX_COM_DELAY)) {
            Serial.print("Timeout of: ");
            Serial.print(millis() - receive_time);
            Serial.println("ms");
        }
    } else if (state == EMERGENCY_STOP){
        digitalWrite(ALL_GOOD_LED, LOW);
        if (!emergency_stop) {
            if (SERIAL_ON && SERIAL_STATES) Serial.println("Emergency Stop Command Inactive");
            state = SHORT_CAN_WAIT;
        }
    } else if (state == STOP_SWITCH) {
        digitalWrite(ALL_GOOD_LED, LOW);
        if (digitalRead(ENABLE) == HIGH){
            if (SERIAL_ON && SERIAL_STATES) Serial.println("Emergency Stop Button Released");
            state = ZERO;
        }
    } else if (state == LARGE_CAN_DELAY){
        if (SERIAL_ON && SERIAL_STATES) Serial.println("LARGE CAN TIMEOUT");
        CAN.begin(CAN_500KBPS); //restarting can
        state = LONG_CAN_WAIT;
    } else if (state == LONG_CAN_WAIT){
        digitalWrite(ALL_GOOD_LED, LOW);
        if (emergency_stop) state = EMERGENCY_STOP;
        else if (digitalRead(ENABLE) == LOW) state = STOP_SWITCH;
        else if ((millis() - receive_time) < MAX_COM_DELAY) {
            if (SERIAL_ON && SERIAL_STATES) Serial.println("COM timeout not exceeded");
            state = STARTING;
        }
        if (SERIAL_ON && SERIAL_STATES && (digitalRead(ENABLE) == LOW)) Serial.println("Emergency Stop Button Pressed");
    }

    if (state == STARTING){
        if (SERIAL_ON) Serial.println("STARTING");
        if (emergency_stop) {
            if (SERIAL_ON && SERIAL_STATES) Serial.println("Emergency Stop Command Active");
            state = DISABLING;
        } else {
           // if (!translation_stepper.isStartedAsService()) translation_stepper.startAsService(STEPPER_CORE);
            //if (!rotation_stepper.isStartedAsService()) rotation_stepper.startAsService(STEPPER_CORE);    
            state = RUNNING;
            if (SERIAL_ON) Serial.println("STARTED");
        }
    }
    if (state == DISABLING){
        if (SERIAL_ON) Serial.println("DISABLING");
        //translation_stepper.emergencyStop(false);
        //rotation_stepper.emergencyStop(false);
        if (emergency_stop) {
            if (SERIAL_ON && SERIAL_STATES) Serial.println("Emergency Stop Command Active");
            state = EMERGENCY_STOP;
        } else if(digitalRead(ENABLE) == LOW) {
            if (SERIAL_ON && SERIAL_STATES) Serial.println("Emergency Stop Button Pressed");
            state = STOP_SWITCH;
        } else state = SHORT_CAN_WAIT;
    }
}














