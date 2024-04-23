/*!
    \file    serialUSB.ino 
    \brief   modif for test laser

    \version 2024-04-23, V1.0
	\author	 kevin.wang
	\note    none
*/
#include <cstdlib>
#include <cstdio>

#include <PacketSerial.h>
#include <FastLED.h>

const char gtitle[] = "serialUSB";
const char gversion[] = "V1.01";
char gtmpbuf[100];

static const int CMD_LENGTH = 8;
static const int MSG_LENGTH = 8;

byte buffer_rx[512];
byte buffer_tx[MSG_LENGTH];

volatile int buffer_rx_ptr;

// Teensy4.1 board v2 def

void setup() {
	/*
	// initialize serial of PC's communication for starting application
	Serial.begin(38400);
	delay(500);

	char cmd;
	while(1) {
		if (Serial.available()) {
			cmd = Serial.read();
			switch(cmd) {
				case 'R':
					goto Run;
					break;
			}
		}
	}

Run:
	sprintf(gtmpbuf, "%s: %s", gtitle, gversion);
	Serial.println(gtmpbuf);

	Serial.end();
	*/

	// Initialize Native USB port
	SerialUSB.begin(2000000);   
	delay(500);
	SerialUSB.setTimeout(5000);
  	buffer_rx_ptr = 0;

	// clean buffer of serialUSB
	while (SerialUSB.available()) 
		SerialUSB.read();
}

void loop() {
	while (SerialUSB) { 
		if (SerialUSB.available()) {
			buffer_rx[buffer_rx_ptr ++] = SerialUSB.read();
			if (buffer_rx_ptr == CMD_LENGTH) {
				buffer_rx_ptr = 0;
				
				SerialUSB.write(buffer_rx, MSG_LENGTH);
			}
		}
	}
}
