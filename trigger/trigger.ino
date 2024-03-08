/*!
    \file    laser.ino 
    \brief   modif for test laser

    \version 2024-01-29, V1.0
	\author	 kevin.wang
	\note    none
*/
#include <cstdlib>
#include <cstdio>

#include <PacketSerial.h>
#include <FastLED.h>

const char gtitle[] = "trigger";
const char gversion[] = "V1.00";

// Teensy4.1 board v2 def

// add by wangwei 2024.01.29
// illumination
static const int TRIGGER1 = 29;

void setup() {
  // enable pins
  pinMode(TRIGGER1, OUTPUT);
  digitalWrite(TRIGGER1, HIGH);
}

void loop() {
  delayMicroseconds(1000);
  // put your main code here, to run repeatedly:
  digitalWrite(TRIGGER1, LOW);
  delayMicroseconds(1000);
  digitalWrite(TRIGGER1, HIGH);
}
