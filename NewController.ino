#include <CAN.h>
#include <FlexyStepper.h>
#include "Controller_Constants.h"
#include "Sensor_Debounce.h"


#include <SPI.h>
//#include "mcp2515fd_can.h"

// Set SPI CS Pin according to your hardware
// For Wio Terminal w/ MCP2518FD RPi Hat：
// Channel 0 SPI_CS Pin: BCM 8
// Channel 1 SPI_CS Pin: BCM 7
// Interupt Pin: BCM25
// *****************************************
// For Arduino MCP2515 Hat:
// SPI_CS Pin: D9




unsigned char len = 0;
unsigned char buf[8];






//NEED TO REMOVE STRING LITERALS USING F("TEXT HERE")






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

FlexyStepper translation_stepper;
FlexyStepper rotation_stepper;

Sensor_Debounce rotational_zero = Sensor_Debounce(ROTATION_DRIVER_ZERO, SENSOR_DEBOUNCE, INPUT_PULLUP, LOW);
Sensor_Debounce translational_zero = Sensor_Debounce(TRANSLATION_DRIVER_ZERO, SENSOR_DEBOUNCE, INPUT_PULLUP, LOW);

bool zero();
void setControl();
void CANSender();
void CANReceiver();
void evaluateState();

int board_ID = 0;

int state = ZERO;
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

void setup() {
//delay(25000);



  
 // analogWrite(HIGH, 0);
  if (SERIAL_ON) {
    Serial.begin(115200);
  }
 
CAN.setPins(SPI_CS_PIN);

if(!CAN.begin(1000E3)){
  Serial.println("STARTING CAN FAILED");
while(1);  
}
  

  SERIAL_PORT_MONITOR.println(F("CAN init ok!"));
  //Pin Initialization
  pinMode(ID_1, INPUT);
  pinMode(ID_2, INPUT);
  pinMode(ENABLE, INPUT);

  //CAN.setPins(RXR_CAN, TXD_CAN);
  pinMode(ALL_GOOD_LED, OUTPUT);
  translation_stepper.connectToPins(TRANSLATION_DRIVER_PULSE, TRANSLATION_DRIVER_DIR);
  rotation_stepper.connectToPins(ROTATION_DRIVER_PULSE, ROTATION_DRIVER_DIR);

  board_ID = digitalRead(ID_2) * 2 + digitalRead(ID_1);

  if (SERIAL_ON) {
    Serial.print(F("Board ID: "));
    Serial.println(board_ID);
  }

  translation_stepper.setStepsPerMillimeter(STEP_PULSE_TRANSLATION_CONVERSION[board_ID]);
  translation_stepper.setAccelerationInMillimetersPerSecondPerSecond(MAX_ACCELERATION_TRANSLATION);
  //translation_stepper.setDecelerationInMillimetersPerSecondPerSecond(MAX_ACCELERATION_TRANSLATION);

  rotation_stepper.setStepsPerRevolution(STEP_PULSE_ROTATION_CONVERSION);
  rotation_stepper.setAccelerationInRevolutionsPerSecondPerSecond(MAX_ACCELERATION_ROTATION);
  //rotation_stepper.setDecelerationInRevolutionsPerSecondPerSecond(MAX_ACCELERATION_ROTATION);


  delay(3000);
  receive_time = millis();
  send_time = millis();
  evaluateState();
}
unsigned char stmp[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };

void loop() {

  /* //copy
  // send data:  id = 0x00, standrad frame, data len = 8, stmp: data buf
    stmp[7] = stmp[7] + 1;
    if (stmp[7] == 100) {
        stmp[7] = 0;
        stmp[6] = stmp[6] + 1;

        if (stmp[6] == 100) {
            stmp[6] = 0;
            stmp[5] = stmp[5] + 1;
        }
    } */

  //CAN_SEND.sendMsgBuf(0x00, 0, 8, stmp);
  //delay(100);                       // send data per 100ms
  //SERIAL_PORT_MONITOR.println(F("CAN BUS sendMsgBuf ok!"));

  // ---------------------

  //if (CAN_MSGAVAIL == CAN_RECEIVE.checkReceive()) {
  // read data,  len: data length, buf: data buf
  // SERIAL_PORT_MONITOR.println(F("checkReceive"));
  // CAN_RECEIVE.readMsgBuf(&len, buf);
  // print the data
  //for (int i = 0; i < len; i++) {
  //  SERIAL_PORT_MONITOR.print(buf[i]); SERIAL_PORT_MONITOR.print(" ");
  //}
  //SERIAL_PORT_MONITOR.println();

  //SERIAL_PORT_MONITOR.println(F("---LOOP END---"));



  //end copy


  evaluateState();
  CANReceiver();
  setControl();
  if (millis() - send_time > COM_DELAY) {
    send_time = millis();
    CANSender();  //Something inside here is sending rot commands to move while sensor is low
  }
}

bool start_CAN(int BAUD) {
  if (SERIAL_ON) Serial.println(F("Starting CAN"));
  while (!CAN.begin(1000E3)) {
    if (SERIAL_ON) Serial.println(F("failed!"));
    digitalWrite(ALL_GOOD_LED, LOW);
    delay(1000);
  }
  while (!CAN.filter(0b1 << board_ID, 0b10100000 + (1 << board_ID) + 0x1fffff00)) {
    if (SERIAL_ON) Serial.println(F("filter failed!"));
    digitalWrite(ALL_GOOD_LED, LOW);
    delay(1000);
  }
  if (SERIAL_ON) Serial.println(F("Starting CAN success"));
}

bool zero() {
  digitalWrite(ALL_GOOD_LED, LOW);
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
  Serial.println("Zero rot being called");
  delay(1000);
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

bool zeroRotation() {
  for (int i = 0; i < SENSOR_DEBOUNCE * 2; i++) rotational_zero.sensorMonitor();
  int timer_serial = 0;
  int timer_bounce = 0;
  int count = 0;
  //rotation_stepper.setCurrentPositionAsHomeAndStop();
  //rotation_stepper.moveToHomeInRevolutions(-1, 20, 50, ROTATION_SENSOR);//ROTATION ZERO HERE AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA
  rotation_stepper.setSpeedInRevolutionsPerSecond(5);
  rotation_stepper.setSpeedInStepsPerSecond(50);
  double distance = 3 * DIRECTIONS[board_ID][ROTATION];
  rotation_stepper.processMovement();
  //NEW CODE START
  while (digitalRead(ROTATION_DRIVER_ZERO) == HIGH) {
    Serial.println("FIRST LOOP high ROT");     //
    rotation_stepper.moveRelativeInSteps(-5);  //MOVES towards stepper
                                               /* if (millis() - timer_bounce > 1) {
            timer_bounce = millis();
            translational_zero.sensorMonitor();
        } */

    //Serial.println(translation_stepper.getCurrentPositionInMillimeters());
  }
  while (digitalRead(ROTATION_DRIVER_ZERO) == LOW) {
    rotation_stepper.setTargetPositionToStop();
    rotation_stepper.processMovement();
    rotation_stepper.moveRelativeInSteps(5);
    Serial.println("ROT DRIVE zero LOW");
  }
  rotation_stepper.setCurrentPositionInSteps(0.0);
  while (digitalRead(ROTATION_DRIVER_ZERO) == HIGH) {
    rotation_stepper.setTargetPositionToStop();
    rotation_stepper.processMovement();
    rotation_stepper.moveRelativeInSteps(-5);
  }
    rotation_stepper.setCurrentPositionInSteps(0.0);

  
    /* rotation_stepper.setCurrentPositionInMillimeters(0.00);

    while(digitalRead(ROTATION_DRIVER_ZERO)==LOW){//first click of switch, should stop then reverse
      //   Serial.println("LOW");     
       
        rotation_stepper.setTargetPositionToStop();
        rotation_stepper.processMovement();
         delay(1000);
         Serial.println("I am now rotating away from motor");  
         
         

           rotation_stepper.moveRelativeInSteps(500); //moves in 5 mm steps away from motor 
    } */

    // while(digitalRead(TRANSLATION_DRIVER_ZERO)==HIGH){//switch is now active again, should stop and move towards motor
    //  translation_stepper.moveRelativeInMillimeters(-1); //MOVES
    //Serial.println("I am now moving back towards switch to finish zero");
    //}

    //translation_stepper.setCurrentPositionInMillimeters(0.00);

    //delay(1000);
    //Serial.println("MOVE TO HOME FINISHED ");


    delay(500);

 // if (rotational_zero.sensorActive()) {
    return true;
// } else {
   // Serial.println(F("Failed due to translation_zero.sensorActive() being false | 0 "));
   // return false;
  //}
    







  //NEW CODE END








  /*  rotation_stepper.setTargetPositionInRevolutions(distance);
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
    } */
}

bool zeroTranslation() {
  for (int i = 0; i < SENSOR_DEBOUNCE * 2; i++) translational_zero.sensorMonitor();
  int timer_serial = millis();
  int timer_bounce = millis();
  translation_stepper.setSpeedInMillimetersPerSecond(HOME_SPEED_TRANSLATION);
 // double distance = DIRECTIONS[board_ID][TRANSLATION] * -1 * MAX_TRANSLATIONS[board_ID];
 // Serial.print(F(" The Distance is: "));
 // Serial.println(distance);

  while (digitalRead(TRANSLATION_DRIVER_ZERO) == HIGH) {
    //Serial.println("FIRST LOOP high");//

    /* if (millis() - timer_bounce > 1) {
            timer_bounce = millis();
            translational_zero.sensorMonitor();
        } */
    translation_stepper.moveRelativeInMillimeters(-1);  //MOVES towards stepper

    //Serial.println(translation_stepper.getCurrentPositionInMillimeters());
  }
  translation_stepper.setCurrentPositionInMillimeters(0.00);

  while (digitalRead(TRANSLATION_DRIVER_ZERO) == LOW) {  //first click of switch, should stop then reverse
    //Serial.println("LOW");

   // translation_stepper.setTargetPositionToStop();
   // translation_stepper.processMovement();
    delay(1000);
    Serial.println("I am now moving away from motor and switch in 5mm steps");



    translation_stepper.moveRelativeInMillimeters(5);  //moves in 5 mm steps away from motor
  }

  while (digitalRead(TRANSLATION_DRIVER_ZERO) == HIGH) {  //switch is now active again, should stop and move towards motor
    translation_stepper.moveRelativeInMillimeters(-1);    //MOVES
    Serial.println("I am now moving back towards switch to finish zero");
//if(translation_stepper.getCurrentPositionInMillimeters()>=5){
     //   translation_stepper.setTargetPositionInMillimeters(0);
      //  translation_stepper.processMovement();
          //  } 
 }

  //translation_stepper.setCurrentPositionInMillimeters(0.00);

  delay(500);

  //if (translational_zero.sensorActive()) {
    return true;
  //} else {
   // Serial.println(F("Failed due to translation_zero.sensorActive() being false | 0 "));
    //return false;
 // }

}

void setControl() {
  if (state == RUNNING) {
    if (translation_desired > MAX_TRANSLATIONS[board_ID]) translation_desired = MAX_TRANSLATIONS[board_ID];
    if (translation_desired < 0) translation_desired = 1;
    if (translation_stepper.getCurrentPositionInMillimeters() * DIRECTIONS[board_ID][TRANSLATION] < -1) {
      if (SERIAL_ON && SERIAL_STATES) {
        Serial.print(F("Went Negative: "));
        Serial.println(translation_stepper.getCurrentPositionInMillimeters() * DIRECTIONS[board_ID][TRANSLATION]);
      }
      state = ZERO;
    } else {
Serial.println("SET CONTROL LOOOP");      
     translation_stepper.setTargetPositionInMillimeters(DIRECTIONS[board_ID][TRANSLATION] * translation_desired);
     rotation_stepper.setTargetPositionInRevolutions(DIRECTIONS[board_ID][ROTATION] * rotation_desired);
    }
  }
}

void CANSender() {
  FLOAT_BYTE_UNION translation_measured_f;
  FLOAT_BYTE_UNION rotation_measured_f;
  translation_measured_f.value = (float)(DIRECTIONS[board_ID][TRANSLATION] * translation_stepper.getCurrentPositionInMillimeters());
  rotation_measured_f.value = (float)(DIRECTIONS[board_ID][ROTATION]*rotation_stepper.getCurrentPositionInRevolutions()*DEGREES_PER_REVOLUTION);
  if (SERIAL_ON && SERIAL_MESSAGES) Serial.print(F("Sent: (packet: 0b"));
  if (state == RUNNING) {
    CAN.beginPacket(0b10010000 + (1 << board_ID));
    for (int i = sizeof(float) - 1; i >= 0; i--) {
      CAN.write(translation_measured_f.bytes[i]);
    }
    for (int i = sizeof(float) - 1; i >= 0; i--) {
      CAN.write(rotation_measured_f.bytes[i]);
    }
    CAN.endPacket();
    if (SERIAL_ON && SERIAL_MESSAGES) Serial.print(0b10010000 + (1 << board_ID), BIN);
  } else if (state == SHORT_CAN_WAIT || state == EMERGENCY_STOP || state == STOP_SWITCH || state == ZERO) {
    CAN.beginPacket((1 << board_ID) + 0b10000000);
    for (int i = sizeof(float) - 1; i >= 0; i--) {
      CAN.write(translation_measured_f.bytes[i]);
    }
    for (int i = sizeof(float) - 1; i >= 0; i--) {
      CAN.write(rotation_measured_f.bytes[i]);
    }
    CAN.endPacket();
    if (SERIAL_ON && SERIAL_MESSAGES) Serial.print((1 << board_ID) + 0b10000000, BIN);
  } else {
    if (SERIAL_ON && SERIAL_MESSAGES) Serial.println(F("NON PRINT STATE"));
  }
  if (SERIAL_ON && SERIAL_MESSAGES) {
    Serial.print(F(" ROTATION: "));
    Serial.print(rotation_measured_f.value);
    Serial.print(F(" Translation: "));
    Serial.println(translation_measured_f.value);
  }
}

int received_zero = false;  //The goal rod will receive phantom zero commands and needs to be stopped
void CANReceiver() {
  if (CAN.parsePacket()) {
    receive_time = millis();
    FLOAT_BYTE_UNION translation_desired_f;
    FLOAT_BYTE_UNION rotation_desired_f;
    if (SERIAL_ON && SERIAL_MESSAGES) {
      Serial.print(F("Recv: (packet: 0b"));
      Serial.print(CAN.packetId(), BIN);
      Serial.print(" ");
    }

    //EMERGENCY STOP
    if ((CAN.packetId() & (0x1fffff00 + 0b11111111)) == 0b00001111) {
      if (SERIAL_ON) Serial.println(F("E STOP message received"));
      emergency_stop = true;
      evaluateState();
      return;
    } else {
      emergency_stop = false;
    }

    //zero
    if ((CAN.packetId() & (0x1fffff00 + 0b11100000)) == 0b01000000) {
      if (SERIAL_ON) Serial.println(F("Zero message received"));
      state = ZERO;
      if (received_zero) state = LARGE_CAN_DELAY;
      received_zero = true;
      evaluateState();
      return;
    }

    if ((CAN.packetId() & (0x1fffff00 + 0b11110000)) == 0b10000) {
      received_zero = false;
      int rotation_index = sizeof(float) - 1;
      int translation_index = sizeof(float) - 1;
      bool data_received = false;
      while (CAN.available()) {
        data_received = true;
        if (translation_index >= 0) {
          translation_desired_f.bytes[translation_index--] = (byte)CAN.read();
        } else if (rotation_index >= 0) {
          rotation_desired_f.bytes[rotation_index--] = (byte)CAN.read();
        } else {
          if (SERIAL_ON && SERIAL_MESSAGES) Serial.println(F("DATA DROPPED)"));
          return;
        }
      }
      if (data_received) {
        if (!((translation_index < 0) && (rotation_index < 0))) {
          if (SERIAL_ON && SERIAL_MESSAGES) Serial.println(F("LESS THAN 8 BYTES RECEIVED)"));
          return;
        }
        rotation_desired = (double)rotation_desired_f.value / DEGREES_PER_REVOLUTION;
        translation_desired = (double)translation_desired_f.value;
        if (SERIAL_ON && SERIAL_MESSAGES) {
          Serial.print(" Rotation: ");
          Serial.print(rotation_desired * DEGREES_PER_REVOLUTION);
          Serial.print(" Translation: ");
          Serial.print(translation_desired);
        }
      }
      if (SERIAL_ON && SERIAL_MESSAGES) Serial.println(F(")"));
    }
  }
}

//int last_state = 0; //This is only used in the top print statement to limit the state prints to only transitions
void evaluateState() {
  if (SERIAL_ON && SERIAL_STATES) {
    //last_state = state;
delay(500);    
    Serial.print(F("State: "));
    Serial.println(state);
  }
  if (state == ZERO) {
    digitalWrite(ALL_GOOD_LED, LOW);
    Serial.println("CALLED AGAIN ZERO IS");
    // translation_stepper.emergencyStop(false);
    //rotation_stepper.emergencyStop(false);
    if (zero()) {
      translation_stepper.setSpeedInMillimetersPerSecond(MAX_SPEED_TRANSLATION);
      rotation_stepper.setSpeedInRevolutionsPerSecond(MAX_SPEED_ROTATION);
      
      receive_time = millis();
      state = SHORT_CAN_WAIT;
    }
  } else if (state == RUNNING) {
    digitalWrite(ALL_GOOD_LED, HIGH);
    if (emergency_stop || (millis() - receive_time > MAX_COM_DELAY) || (digitalRead(ENABLE) == LOW)) {
      state = DISABLING;
      if (SERIAL_ON && SERIAL_STATES && emergency_stop) Serial.println(F("Emergency Stop Command Active"));
      if (SERIAL_ON && SERIAL_STATES && ((millis() - receive_time) > MAX_COM_DELAY)) Serial.println(F("COM timeout"));
      if (SERIAL_ON && SERIAL_STATES && (digitalRead(ENABLE) == LOW)) Serial.println(F("Emergency Stop Button Pressed"));
    }
  } else if (state == SHORT_CAN_WAIT) {
    digitalWrite(ALL_GOOD_LED, LOW);
    if (emergency_stop) state = EMERGENCY_STOP;
    else if (digitalRead(ENABLE) == LOW) state = STOP_SWITCH;
    else if ((millis() - receive_time) < MAX_COM_DELAY) {
      state = STARTING;
      if (SERIAL_ON && SERIAL_STATES) Serial.println(F("COM timeout not exceeded"));
    }
    if ((millis() - receive_time) > MAX_COM_DELAY * 10) state = LARGE_CAN_DELAY;
    if (SERIAL_ON && SERIAL_STATES && (digitalRead(ENABLE) == LOW)) Serial.println(F("Emergency Stop Button Pressed"));
    if (SERIAL_ON && SERIAL_STATES && ((millis() - receive_time) > MAX_COM_DELAY)) {
      Serial.print(F("Timeout of: "));
      Serial.print(millis() - receive_time);
      Serial.println(F("ms"));
    }
  } else if (state == EMERGENCY_STOP) {
    digitalWrite(ALL_GOOD_LED, LOW);
    if (!emergency_stop) {
      if (SERIAL_ON && SERIAL_STATES) Serial.println(F("Emergency Stop Command Inactive"));
      state = SHORT_CAN_WAIT;
    }
  } else if (state == STOP_SWITCH) {
    digitalWrite(ALL_GOOD_LED, LOW);
    if (digitalRead(ENABLE) == HIGH) {
      if (SERIAL_ON && SERIAL_STATES) Serial.println(F("Emergency Stop Button Released"));
      state = ZERO;
    }
  } else if (state == LARGE_CAN_DELAY) {
    if (SERIAL_ON && SERIAL_STATES) Serial.println(F("LARGE CAN TIMEOUT"));
    start_CAN(BAUD_RATE);
    state = LONG_CAN_WAIT;  //CHANGE INTO STATE 0 OR SOMETHING
  } else if (state == LONG_CAN_WAIT) {
    digitalWrite(ALL_GOOD_LED, LOW);
    if (emergency_stop) state = EMERGENCY_STOP;
    else if (digitalRead(ENABLE) == LOW) state = STOP_SWITCH;
    else if ((millis() - receive_time) < MAX_COM_DELAY) {
      if (SERIAL_ON && SERIAL_STATES) Serial.println(F("COM timeout not exceeded"));
      state = STARTING;
    }
    if (SERIAL_ON && SERIAL_STATES && (digitalRead(ENABLE) == LOW)) Serial.println(F("Emergency Stop Button Pressed"));
  }

  if (state == STARTING) {
    if (SERIAL_ON) Serial.println(F("STARTING"));
    if (emergency_stop) {
      if (SERIAL_ON && SERIAL_STATES) Serial.println(F("Emergency Stop Command Active"));
      state = DISABLING;
    } else {
      //if (!translation_stepper.isStartedAsService()) translation_stepper.startAsService(STEPPER_CORE);
      //if (!rotation_stepper.isStartedAsService()) rotation_stepper.startAsService(STEPPER_CORE);
      state = RUNNING;
      if (SERIAL_ON) Serial.println(F("STARTED"));
    }
  }
  if (state == DISABLING) {
    if (SERIAL_ON) Serial.println(F("DISABLING"));
    //translation_stepper.emergencyStop(false);
    //rotation_stepper.emergencyStop(false);
    if (emergency_stop) {
      if (SERIAL_ON && SERIAL_STATES) Serial.println(F("Emergency Stop Command Active"));
      state = EMERGENCY_STOP;
    } else if (digitalRead(ENABLE) == LOW) {
      if (SERIAL_ON && SERIAL_STATES) Serial.println(F("Emergency Stop Button Pressed"));
      state = STOP_SWITCH;
    } else state = SHORT_CAN_WAIT;
  }
}
