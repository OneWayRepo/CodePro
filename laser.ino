/*!
    \file    laser.ino 
    \brief   modif for test laser

    \version 2024-01-29, V1.0
	\author	 kevin.wang
	\note    none
*/

#include <PacketSerial.h>
#include <FastLED.h>

// Teensy4.1 board v2 def

// add by wangwei 2024.01.29
// illumination
static const int LASER_550nm = 19;
static const int LASER_402nm = 18;
static const int LASER_470nm = 17;
static const int LASER_638nm = 16;
static const int LASER_735nm = 15;

// Laser enable pin
static const int LASER_Enable_pin   = 33;

void setup() {
  // put your setup code here, to run once:

  // add by wangwei 2024.01.29
  // enable pins
  pinMode(LASER_550nm, OUTPUT);
  digitalWrite(LASER_550nm, LOW);

  pinMode(LASER_402nm, OUTPUT);
  digitalWrite(LASER_402nm, LOW);

  pinMode(LASER_470nm, OUTPUT);
  digitalWrite(LASER_470nm, LOW);

  pinMode(LASER_638nm, OUTPUT);
  digitalWrite(LASER_638nm, LOW);

  pinMode(LASER_735nm, OUTPUT);
  digitalWrite(LASER_735nm, LOW);

  // Laser enable pin
  pinMode(LASER_Enable_pin, OUTPUT);
  digitalWrite(LASER_Enable_pin ,HIGH);
}

void loop() {
  // put your main code here, to run repeatedly:

}
