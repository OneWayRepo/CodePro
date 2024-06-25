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

const char gtitle[] = "Lazer_5channels";
const char gversion[] = "V1.00";
char gtmpbuf[100];

// Teensy4.1 board v2 def

// add by wangwei 2024.01.29
// illumination
static const int LASER_550nm = 19;
static const int LASER_402nm = 18;
static const int LASER_470nm = 17;
static const int LASER_638nm = 16;
static const int LASER_735nm = 15;

static const int OUTPUT1 = 36;
static const int OUTPUT2 = 37;
static const int OUTPUT3 = 38;

// despeckler enable pin
static const int DESPECKLER_Enable_pin   = 33;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(38400);
  sprintf(gtmpbuf, "%s %s", gtitle, gversion);
  Serial.println(gtmpbuf);

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

  // Despeckler enable pin
  pinMode(DESPECKLER_Enable_pin, OUTPUT);
  digitalWrite(DESPECKLER_Enable_pin ,HIGH);

  pinMode(OUTPUT1, OUTPUT);
  digitalWrite(OUTPUT1, LOW);

  pinMode(OUTPUT2, OUTPUT);
  digitalWrite(OUTPUT2, LOW);

  pinMode(OUTPUT3, OUTPUT);
  digitalWrite(OUTPUT3, LOW);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(OUTPUT1,HIGH);
   delayMicroseconds(2000000);
   digitalWrite(OUTPUT1,LOW);
   
  digitalWrite(OUTPUT2,HIGH);
   delayMicroseconds(2000000);
  digitalWrite(OUTPUT2,LOW);

    digitalWrite(OUTPUT3,HIGH);
   delayMicroseconds(2000000);
  digitalWrite(OUTPUT3,LOW);

}
