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

const char gtitle[] = "Lazer_single";
const char gversion[] = "V1.01";
char gtmpbuf[100];

// Teensy4.1 board v2 def

// add by wangwei 2024.01.29
// illumination
static const int LASER_550nm = 19;
static const int LASER_402nm = 18;
static const int LASER_470nm = 17;
static const int LASER_638nm = 16;
static const int LASER_735nm = 15;

// TCM1040, TCM1041 module
PacketSerial TCM_104x_PacketSerial;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(38400);
  sprintf(gtmpbuf, "%s %s", gtitle, gversion);
  Serial.println(gtmpbuf);

  // add by wangwei 2024.01.31
  // TCM104x module
  Serial5.begin(57600);

  // add by wangwei 2024.01.29
  // enable pins
  pinMode(LASER_550nm, OUTPUT);
  digitalWrite(LASER_550nm, HIGH);

  pinMode(LASER_402nm, OUTPUT);
  digitalWrite(LASER_402nm, HIGH);

  pinMode(LASER_470nm, OUTPUT);
  digitalWrite(LASER_470nm, LOW);

  pinMode(LASER_638nm, OUTPUT);
  digitalWrite(LASER_638nm, HIGH);

  pinMode(LASER_735nm, OUTPUT);
  digitalWrite(LASER_735nm, HIGH);
}

void loop() {
  // put your main code here, to run repeatedly:

}
