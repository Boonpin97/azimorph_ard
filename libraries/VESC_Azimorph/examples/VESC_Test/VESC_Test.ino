/*
  Name:    getVescValues.ino
  Created: 19-08-2018
  Author:  SolidGeek
  Description:  This example is made using a Arduino Micro (Atmega32u4) that has a HardwareSerial port (Serial1) seperated from the Serial port.
                A Arduino Nano or Uno that only has one Serial port will not be able to display the data returned.
*/

#include <VescUart.h>

/** Initiate VescUart class */
VescUart MotorFR;
VescUart MotorFL;
VescUart MotorBL;
VescUart MotorBR;

int spd = 500;
int turn = 1000;


void setup() {

  /** Setup Serial port to display data */
  Serial.begin(9600);

  /** Setup UART port (Serial1 on Atmega32u4) */
  Serial2.begin(115200);
  Serial3.begin(115200);
  Serial4.begin(115200);
  Serial5.begin(115200);

  while (!Serial2) {
    ;
    Serial2.println("Front Right Serial Fail");
  }
  while (!Serial3) {
    ;
    Serial3.println("Front Left Serial Fail");
  }
  while (!Serial4) {
    ;
    Serial4.println("Back Left Serial Fail");
  }
  while (!Serial5) {
    ;
    Serial5.println("Back Right Serial Fail");
  }

  /** Define which ports to use as UART */
  MotorFR.setSerialPort(&Serial2);
  MotorFL.setSerialPort(&Serial3);
  MotorBL.setSerialPort(&Serial4);
  MotorBR.setSerialPort(&Serial5);
  delay(3000);
  //UART.nunchuck.lowerButton = true;
}

void loop() {

  /** Call the function getVescValues() to acquire data from VESC */
    MotorBR.getVescValues();
    MotorFR.getVescValues();
    MotorFL.getVescValues();
    MotorBL.getVescValues();
    Serial.print("BR:");
    Serial.print(MotorBR.data.rpm);
    Serial.print(" |FR:");
    Serial.print(MotorFR.data.rpm);
    Serial.print(" |FL:");
    Serial.print(MotorFL.data.rpm);
    Serial.print(" |BL:");
    Serial.println(MotorBL.data.rpm);
    //UART.nunchuck.valueY = 130;

  int timer = millis();
  //    while (millis() - timer < 4000) {
  //      MotorBR.setRPM(spd);
  //      MotorFR.setRPM(spd);
  //      MotorFL.setRPM(spd);
  //      MotorBL.setRPM(spd);
  //    }
  //    MotorBR.setRPM(0);
  //    MotorFR.setRPM(0);
  //    MotorFL.setRPM(0);
  //    MotorBL.setRPM(0);
  //    delay(2000);
  //
  //    timer = millis();
  //    while (millis() - timer < 2000) {
  //      MotorBR.setRPM(-spd);
  //      MotorFR.setRPM(-spd);
  //      MotorFL.setRPM(-spd);
  //      MotorBL.setRPM(-spd);
  //    }
  //    MotorBR.setRPM(0);
  //    MotorFR.setRPM(0);
  //    MotorFL.setRPM(0);
  //    MotorBL.setRPM(0);
  //    delay(2000);
  //
  //  timer = millis();
  //  while (millis() - timer < 2000) {
  //    MotorBR.setRPM(turn);
  //    MotorFR.setRPM(turn);
  //    MotorFL.setRPM(-turn);
  //    MotorBL.setRPM(-turn);
  //  }
  //    MotorBR.setRPM(0);
  //    MotorFR.setRPM(0);
  //    MotorFL.setRPM(0);
  //    MotorBL.setRPM(0);
  //    delay(2000);
  //  timer = millis();
  //  while (millis() - timer < 2000) {
      MotorBR.setRPM(100);
      MotorFR.setRPM(100);
      MotorFL.setRPM(spd);
      MotorBL.setRPM(spd);
  //  }

  //    MotorBR.setRPM(0);
  //    MotorFR.setRPM(0);
  //    MotorFL.setRPM(0);
  //    MotorBL.setRPM(0);
  //    delay(99000);
}
