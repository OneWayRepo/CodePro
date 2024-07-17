/*!
  \file    laser.ino 
  \brief   modif for test laser

  \version 2024-07-17, V1.0
	\author	 kevin.wang
	\note    none
*/
#include <cstdlib>
#include <cstdio>

#include <FastLED.h>

#include <elapsedMillis.h>
#include <CRC32.h>

const char gtitle[] = "Lazer_Firmware";
const char gversion[] = "V1.00";
char gtmpbuf[100];

// Teensy4.1 board v2 def

// illumination
static const int LASER_550nm = 19;
static const int LASER_402nm = 18;
static const int LASER_470nm = 17;
static const int LASER_638nm = 16;
static const int LASER_735nm = 15;

const int NUM_LASER_CHANNELS = 5;
const int NUM_TEMP_CHANNELS = 6;
const unsigned long IDLE_TIMEOUT = 3UL * 60UL * 60UL * 1000UL; // 3 hours in milliseconds
const float TEMP_ERROR_THRESHOLD = 5.0; // Celsius
const unsigned long ERROR_DURATION = 5000; // 5 seconds in milliseconds

enum ChannelState {
  IDLE,
  WARM_UP,
  ACTIVE,
  ERROR
};

CRC32 crc;

// Pins for laser channels (adjust as needed)
const int laserPins[NUM_LASER_CHANNELS] = {LASER_402nm, LASER_470nm, LASER_550nm, LASER_638nm, LASER_735nm};

// Pins for temperature sensors, TEC voltage, and TEC current (adjust as needed)
const int tempSensorPins[NUM_TEMP_CHANNELS] = {A0, A1, A2, A3, A4, A5};
const int tecVoltagePins[NUM_TEMP_CHANNELS] = {A6, A7, A8, A9, A10, A11};
const int tecCurrentPins[NUM_TEMP_CHANNELS] = {A12, A13, A14, A15, A16, A17};

// Temperature setpoints and channel states
float tempSetpoints[NUM_TEMP_CHANNELS] = {25.0, 25.0, 25.0, 25.0, 25.0, 25.0};
ChannelState channelStates[NUM_TEMP_CHANNELS] = {IDLE};
elapsedMillis timeSinceLastActive;
elapsedMillis timeInErrorState[NUM_TEMP_CHANNELS] = {0};

void enterIdleState() {
  for (int i = 0; i < NUM_TEMP_CHANNELS; i++) {
    channelStates[i] = IDLE;
    disableLaser(i);
  }
}

void disableLaser(int channel) {
  if (channel < NUM_LASER_CHANNELS) {
    digitalWrite(laserPins[channel], HIGH);
  }
}

float readTemperature(int channel) {
  // Convert ADC reading to temperature (adjust based on your sensor)
  //return (analogRead(tempSensorPins[channel]) * 3.3 / 1024.0 - 0.5) * 100.0;
	return 26.5;
}

float readTECVoltage(int channel) {
  //return analogRead(tecVoltagePins[channel]) >> 2;
	return 8.6;
}

float readTECCurrent(int channel) {
  //return analogRead(tecCurrentPins[channel]) >> 2;
	return 10.6;
}

void send_msg(uint8_t* data, uint8_t length)
{
	for (int i = 0; i < length; i++) {
		Serial.println(data[i]);
	}
}

void write_data(uint8_t* data, uint8_t length)
{
	Serial.write(data, length);
	uint8_t end[2] = {0x0A, 0x0D};
	Serial.write(end, 2);
}

void sendACK() {
  uint8_t ack = 'A';
  uint32_t ackCRC = crc.calculate(&ack, 1);
  uint8_t ackPacket[5];
  ackPacket[0] = ack;
  memcpy(ackPacket + 1, &ackCRC, 4);
	write_data(ackPacket, 5);
}

void sendNAK() {
  uint8_t nak = 'N';
  uint32_t nakCRC = crc.calculate(&nak, 1);
  uint8_t nakPacket[5];
  nakPacket[0] = nak;
  memcpy(nakPacket + 1, &nakCRC, 4);
	write_data(nakPacket, 5);
}

void onPacketReceived(const uint8_t* buffer, size_t size) {
  if (size < 5) return; // Minimum packet size (1 byte command + 4 bytes CRC)

  uint32_t receivedCRC;
  memcpy(&receivedCRC, buffer + size - 4, 4);
  uint32_t calculatedCRC = crc.calculate(buffer, size - 4);

  if (receivedCRC != calculatedCRC) {
    sendNAK();
    return;
  }

  switch (buffer[0]) {
    case 'Q': // Query status
      sendStatus();
      break;
    case 'S': // Set parameters
      setParameters(buffer, size - 4);
      break;
  }
}

void setup() {
  Serial.begin(115200);
	char cmd;
	while(1) {
		if (Serial.available()) {
			cmd = Serial.read();
			cmd = 'R';
			switch(cmd) {
				case 'R':
					goto Run;
					break;
			}
		}
	}

Run:
  sprintf(gtmpbuf, "%s %s", gtitle, gversion);
  Serial.println(gtmpbuf);

  // TCM104x module UART5
  Serial5.begin(57600);

  // enable pins
  for (int i = 0; i < NUM_LASER_CHANNELS; i++) {
    pinMode(laserPins[i], OUTPUT);
    digitalWrite(laserPins[i], LOW);
  }
}

void loop() {
	char cmd;
  bool anyActive = false;
  for (int i = 0; i < NUM_TEMP_CHANNELS; i++) {
    float currentTemp = readTemperature(i);
    float tempDiff = currentTemp - tempSetpoints[i];

    switch (channelStates[i]) {
      case IDLE:
        if (tempDiff < -0.5) {
          channelStates[i] = WARM_UP;
        }
        break;
      case WARM_UP:
        if (abs(tempDiff) < 0.5) {
          channelStates[i] = ACTIVE;
        }
        break;
      case ACTIVE:
        if (tempDiff > TEMP_ERROR_THRESHOLD) {
          channelStates[i] = ERROR;
          timeInErrorState[i] = 0;
        }
        anyActive = true;
        break;
      case ERROR:
        if (timeInErrorState[i] >= ERROR_DURATION) {
          disableLaser(i);
        }
        if (abs(tempDiff) < 0.5) {
          channelStates[i] = ACTIVE;
        }
        break;
    }
  }

  if (anyActive) {
    timeSinceLastActive = 0;
  } else if (timeSinceLastActive >= IDLE_TIMEOUT) {
    enterIdleState();
  }

  // put your main code here, to run repeatedly:
	if (Serial.available()) {
		cmd = Serial.read();

		switch (cmd) {
			case 'N':
				sendNAK();
				break;
			case 'A':
				sendACK();
				break;
			case 'T':
				break;
		}
	}
}
