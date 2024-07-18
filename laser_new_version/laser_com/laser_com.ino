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
const char gversion[] = "V1.10";
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

const unsigned long QUERY_TEMP_TIMEOUT = 500UL; // 500 in milliseconds
const int8_t ERR_OUT_OF_RANGE = 100;

enum ChannelState {
  IDLE,
  WARM_UP,
  ACTIVE,
  ERROR
};

enum CommandType{
  TEMPERATURE,
	CURRENT,
 	VOLTAGE, 
	NONE
};

CRC32 crc;

// Pins for laser channels (adjust as needed)
const int laserPins[NUM_LASER_CHANNELS] = {LASER_402nm, LASER_470nm, LASER_550nm, LASER_638nm, LASER_735nm};

// Temperature setpoints and channel states
float tempSetpoints[NUM_TEMP_CHANNELS] = {25.0, 25.0, 25.0, 25.0, 25.0, 25.0};
float tempCurrentPoints[NUM_TEMP_CHANNELS] = {25.0, 25.0, 25.0, 25.0, 25.0, 25.0};

ChannelState channelStates[NUM_TEMP_CHANNELS] = {IDLE};
elapsedMillis timeSinceLastActive;
elapsedMillis timeInErrorState[NUM_TEMP_CHANNELS] = {0};

elapsedMillis timeSinceLastQueryTemp = 0;

uint8_t gQueryTemperatureChannelIndex = 0;

// TCM modules protocol process variables
bool reply_frame_analyzing_flag = false;

char tcm_command_buf[256];
uint8_t tcm_command_buf_length = 0;

char tcm_reply_buf[256];
uint8_t tcm_reply_buf_length = 0;

// for judge whether the replay value is correct or not
char tcm_reply_title[256];
uint8_t tcm_reply_title_length = 0;
uint8_t tcm_reply_address = 0;
uint8_t tcm_reply_module = 0;
CommandType tcm_reply_command_type = NONE;

void setParameters(const uint8_t* buffer, size_t size) {
  if (size == 1 + NUM_TEMP_CHANNELS * sizeof(float)) {
    for (int i = 0; i < NUM_TEMP_CHANNELS; i++) {
      memcpy(&tempSetpoints[i], buffer + 1 + i * sizeof(float), sizeof(float));
    }
    sendACK();
  } else {
    sendNAK();
  }
}

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

/*
TC1:TCACTUALTEMP?@1		channel:0
TC1:TCACTTEMP?@2			channel:1
TC1:TCACTUALTEMP?@3		channel:2
TC1:TCACTUALTEMP?@4		channel:3
TC1:TCACTUALTEMP?@5		channel:4
TC2:TCACTUALTEMP?@5		channel:5
 */
float readTemperature(int channel) {
	return tempCurrentPoints[channel];
}

/*
	address: 1~5
	module_index: 1~2
	return: 0~5
 */
uint8_t getChannelIndex(uint8_t address, uint8_t module_index) {
	if (address < 5)
		return address - 1;
	else if(address == 5) {
    if (module_index == 1)
      return 4;
    else if (module_index == 2)
      return 5;
	}
  return ERR_OUT_OF_RANGE;
}

/*
	channel: 0~5
*/
void getAddressModuleFromChannel(uint8_t channel, uint8_t* address, uint8_t* module_index) {
	if (channel<4) {
		*address = channel + 1;
		*module_index = 1;
	}
	else {
		if (channel == 4) {
			*address = 5;
			*module_index = 1;
		}
		else if (channel == 5) {
			*address = 5;
			*module_index = 2;
		}
		else {
			*address = ERR_OUT_OF_RANGE; 
			*module_index = ERR_OUT_OF_RANGE;
		}
	}
}

/*
	channel_index: 0~5

	address: 1~5
	module_index: 1~2
 */
void tcmQueryTemperatureCommand(uint8_t channel_index) {
	uint8_t address = 1; uint8_t module_index = 1;
	getAddressModuleFromChannel(channel_index, &address, &module_index);
	if (address == ERR_OUT_OF_RANGE && module_index == ERR_OUT_OF_RANGE)
		return;

	tcm_command_buf_length = 0;

	if (address == 2) {
		sprintf(tcm_command_buf, "TC%d:TCACTTEMP?@%d%c", module_index, address, 0x0D);
	}
	else {
		sprintf(tcm_command_buf, "TC%d:TCACTUALTEMP?@%d%c", module_index, address, 0x0D);
	}
	Serial5.write(tcm_command_buf, strlen(tcm_command_buf));
	
	if (address == 2) {
		sprintf(tcm_reply_title, "TC%d:TCACTTEMP=", module_index);
	}
	else {
		sprintf(tcm_reply_title, "TC%d:TCACTUALTEMP=", module_index);
	}

	tcm_reply_title_length = strlen(tcm_reply_title);
	tcm_reply_address = address;
	tcm_reply_module = module_index;
	tcm_reply_command_type = TEMPERATURE;
		
	// reset tcm ptotocol analyzing buffer
	tcm_reply_buf_length = 0;
	
	// enable analyzing frame
	reply_frame_analyzing_flag = true;
}

float readTECVoltage(int channel) {
	return 8.6;
}

float readTECCurrent(int channel) {
	return 10.6;
}

/*
	upload data to upstream host, for example PC
 */
void uploadData(uint8_t* data, uint8_t length)
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
	uploadData(ackPacket, 5);
}

void sendNAK() {
  uint8_t nak = 'N';
  uint32_t nakCRC = crc.calculate(&nak, 1);
  uint8_t nakPacket[5];
  nakPacket[0] = nak;
  memcpy(nakPacket + 1, &nakCRC, 4);
	uploadData(nakPacket, 5);
}

void sendStatus() {
  uint8_t statusPacket[1 + NUM_LASER_CHANNELS + NUM_TEMP_CHANNELS * 5];
  statusPacket[0] = 'S'; // Status packet identifier

  for (int i = 0; i < NUM_LASER_CHANNELS; i++) {
    statusPacket[i + 1] = digitalRead(laserPins[i]);
  }

  for (int i = 0; i < NUM_TEMP_CHANNELS; i++) {
    int offset = 1 + NUM_LASER_CHANNELS + i * 5;
    statusPacket[offset] = channelStates[i];
    
    // Convert temperature to 2-byte representation
    int16_t temp = (int16_t)(readTemperature(i) * 100); // Temperature in centidegrees
    statusPacket[offset + 1] = temp >> 8;
    statusPacket[offset + 2] = temp & 0xFF;
    
    statusPacket[offset + 3] = readTECVoltage(i);
    statusPacket[offset + 4] = readTECCurrent(i);
  }

  uint32_t packetCRC = crc.calculate(statusPacket, sizeof(statusPacket));
  uint8_t finalPacket[sizeof(statusPacket) + 4];
  memcpy(finalPacket, statusPacket, sizeof(statusPacket));
  memcpy(finalPacket + sizeof(statusPacket), &packetCRC, 4);

	uploadData(finalPacket, sizeof(finalPacket));
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
      //sendStatus();
      break;
    case 'S': // Set parameters
      //setParameters(buffer, size - 4);
      break;
  }
}

/*
	get a char position from a char array 

	return: offset address of the char, if not found return -1 
				
 */
int getCharOffset(char* arrayValue, char searchChar) {
	char *p = strchr(arrayValue, searchChar);
	if (p != NULL) {
		return p - arrayValue;
	}
	else {
		return -1; 
	}
}

/*
	convert char array to float	

 */
float convertToFloat(char * arrayValue, int valueLength) {
	char inArray[valueLength + 1]	= {0};
	for (int i = 0; i < valueLength; i++) {
		inArray[i] = arrayValue[i];
	}

	String value(inArray);
	return value.toFloat();
}

/*
	get information from one frame reply

	return: true, the retvalue is the available value
				false, the retvalue is the inavailable value
 */
bool analyzeValueFromProtocol(float *retvalue) {
	if (strncmp(tcm_reply_title, tcm_reply_buf, tcm_reply_title_length) == 0) {
		int offset = getCharOffset(&tcm_reply_buf[tcm_reply_title_length], '@');
		if (offset == -1)
			return false;

		*retvalue = convertToFloat(&tcm_reply_buf[tcm_reply_title_length], offset);
		return true;
	}
	else
		return false;
}

/*
	query loop, call it in the main loop
*/
void queryTMCTemperatureMainLoop() {
	if (timeSinceLastQueryTemp < QUERY_TEMP_TIMEOUT)
		return;
	timeSinceLastQueryTemp = 0;

	tcmQueryTemperatureCommand(gQueryTemperatureChannelIndex++);
	if (gQueryTemperatureChannelIndex == NUM_TEMP_CHANNELS)
		gQueryTemperatureChannelIndex = 0;
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

	timeSinceLastQueryTemp = 0;
}

void loop() {
	char cmd;
  bool anyActive = false;

	queryTMCTemperatureMainLoop();

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

  // code just for debug
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
				for (int i = 0; i < 6; i++) {
					tempCurrentPoints[i] = 27;
				}
				break;
			case 'R':
				for (int i = 0; i < 6; i++) {
					sprintf(gtmpbuf, "ch%d: %f", i, tempCurrentPoints[i]);
  				Serial.println(gtmpbuf);
				}
				break;
		}
	}
  // code just for debug

	// TCM modules protocol process
	if (reply_frame_analyzing_flag) {
		if (Serial5.available()) {
			tcm_reply_buf[tcm_reply_buf_length++] = Serial5.read();
			// read the end flag of frame
			if (tcm_reply_buf[tcm_reply_buf_length - 1] == 0x0D) {
				if (tcm_reply_command_type == TEMPERATURE) {
					float tvalue = 0;
					if (analyzeValueFromProtocol(&tvalue)) {
						uint8_t tindex = getChannelIndex(tcm_reply_address, tcm_reply_module);
						if (tindex != ERR_OUT_OF_RANGE)
							tempCurrentPoints[tindex] = tvalue;
					}
					// finish frame analyzing, reset flag
					reply_frame_analyzing_flag = false;
				}
			}
		}
	}
}
