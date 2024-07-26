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

const char gtitle[] = "final_check";
const char gversion[] = "V1.00";
char gtmpbuf[100];

// power good
const int pin_PG = 0;
// cs pin for motor x y z w
const uint8_t pin_TMC4361_CS[4] = {41,36,35,34};

// Teensy4.1 board v2 def

// illumination
static const int LASER_405nm = 5;   // to rename
static const int LASER_488nm = 4;   // to rename
static const int LASER_561nm = 22;   // to rename
static const int LASER_638nm = 3;  // to rename
static const int LASER_730nm = 23;  // to rename

// DAC
const int DAC8050x_CS_pin = 33;
const uint8_t DAC8050x_DAC_ADDR = 0x08;
const uint8_t DAC8050x_GAIN_ADDR = 0x04;
const uint8_t DAC8050x_CONFIG_ADDR = 0x03;

void set_DAC8050x_config()
{
  uint16_t value = 0;
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE2));
  digitalWrite(DAC8050x_CS_pin, LOW);
  SPI.transfer(DAC8050x_CONFIG_ADDR);
  SPI.transfer16(value);
  digitalWrite(DAC8050x_CS_pin, HIGH);
  SPI.endTransaction();
}

void set_DAC8050x_gain(uint8_t div, uint8_t gains) 
{
  uint16_t value = 0;
  value = (div << 8) + gains; 
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE2));
  digitalWrite(DAC8050x_CS_pin, LOW);
  SPI.transfer(DAC8050x_GAIN_ADDR);
  SPI.transfer16(value);
  digitalWrite(DAC8050x_CS_pin, HIGH);
  SPI.endTransaction();
}

void set_DAC8050x_output(int channel, uint16_t value)
{
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE2));
  digitalWrite(DAC8050x_CS_pin, LOW);
  SPI.transfer(DAC8050x_DAC_ADDR + channel);
  SPI.transfer16(value);
  digitalWrite(DAC8050x_CS_pin, HIGH);
  SPI.endTransaction();
}

void setup() {
	// check power
	pinMode(pin_PG,INPUT_PULLUP);

	// wait for PG to turn high
	delay(100);
	while(!digitalRead(pin_PG)) {
		delay(50);
	}

	// disable steppers pins
	for(int i=0;i<4;i++) {
		pinMode(pin_TMC4361_CS[i],OUTPUT);
		digitalWrite(pin_TMC4361_CS[i],HIGH);
	}

  // put your setup code here, to run once:
  Serial.begin(38400);
  sprintf(gtmpbuf, "%s %s", gtitle, gversion);
  Serial.println(gtmpbuf);

  // SPI
  SPI.begin();
  delayMicroseconds(5000);

  // DAC pins
  pinMode(DAC8050x_CS_pin, OUTPUT);
  digitalWrite(DAC8050x_CS_pin, HIGH);

  set_DAC8050x_config();
  set_DAC8050x_gain(0x00, 0x80);

	uint16_t illumination_intensity = 32767;
	for (int i = 0; i < 5; i++) {
		set_DAC8050x_output(i, illumination_intensity);
	}

  // enable pins
  pinMode(LASER_405nm, OUTPUT);
  digitalWrite(LASER_405nm, LOW);

  pinMode(LASER_488nm, OUTPUT);
  digitalWrite(LASER_488nm, LOW);

  pinMode(LASER_561nm, OUTPUT);
  digitalWrite(LASER_561nm, LOW);

  pinMode(LASER_638nm, OUTPUT);
  digitalWrite(LASER_638nm, LOW);

  pinMode(LASER_730nm, OUTPUT);
  digitalWrite(LASER_730nm, LOW);
}

void enable_pin() {
	digitalWrite(LASER_405nm, HIGH);
	digitalWrite(LASER_488nm, HIGH);
	digitalWrite(LASER_561nm, HIGH);
	digitalWrite(LASER_638nm, HIGH);
	digitalWrite(LASER_730nm, HIGH);
}

void disable_pin() {
	digitalWrite(LASER_405nm, LOW);
	digitalWrite(LASER_488nm, LOW);
	digitalWrite(LASER_561nm, LOW);
	digitalWrite(LASER_638nm, LOW);
	digitalWrite(LASER_730nm, LOW);
}

void loop() {
	enable_pin();
	delayMicroseconds(2000000);
	disable_pin();
	delayMicroseconds(2000000);
}
