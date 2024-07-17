#include <PacketSerial.h>
#include <elapsedMillis.h>
#include <CRC32.h>

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

PacketSerial packetSerial;
CRC32 crc;

// Pins for laser channels (adjust as needed)
const int laserPins[NUM_LASER_CHANNELS] = {2, 3, 4, 5, 6};

// Pins for temperature sensors, TEC voltage, and TEC current (adjust as needed)
const int tempSensorPins[NUM_TEMP_CHANNELS] = {A0, A1, A2, A3, A4, A5};
const int tecVoltagePins[NUM_TEMP_CHANNELS] = {A6, A7, A8, A9, A10, A11};
const int tecCurrentPins[NUM_TEMP_CHANNELS] = {A12, A13, A14, A15, A16, A17};

// Temperature setpoints and channel states
float tempSetpoints[NUM_TEMP_CHANNELS] = {25.0, 25.0, 25.0, 25.0, 25.0, 25.0};
ChannelState channelStates[NUM_TEMP_CHANNELS] = {IDLE};
elapsedMillis timeSinceLastActive;
elapsedMillis timeInErrorState[NUM_TEMP_CHANNELS] = {0};

void setup() {
  packetSerial.begin(115200);
  packetSerial.setPacketHandler(&onPacketReceived);

  for (int i = 0; i < NUM_LASER_CHANNELS; i++) {
    pinMode(laserPins[i], OUTPUT);
    digitalWrite(laserPins[i], LOW);
  }

  // Initialize other pins and components as needed
}

void loop() {
  packetSerial.update();

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

  packetSerial.send(finalPacket, sizeof(finalPacket));
}

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
    digitalWrite(laserPins[channel], LOW);
  }
}

float readTemperature(int channel) {
  // Convert ADC reading to temperature (adjust based on your sensor)
  return (analogRead(tempSensorPins[channel]) * 3.3 / 1024.0 - 0.5) * 100.0;
}

uint8_t readTECVoltage(int channel) {
  return analogRead(tecVoltagePins[channel]) >> 2;
}

uint8_t readTECCurrent(int channel) {
  return analogRead(tecCurrentPins[channel]) >> 2;
}

void sendACK() {
  uint8_t ack = 'A';
  uint32_t ackCRC = crc.calculate(&ack, 1);
  uint8_t ackPacket[5];
  ackPacket[0] = ack;
  memcpy(ackPacket + 1, &ackCRC, 4);
  packetSerial.send(ackPacket, 5);
}

void sendNAK() {
  uint8_t nak = 'N';
  uint32_t nakCRC = crc.calculate(&nak, 1);
  uint8_t nakPacket[5];
  nakPacket[0] = nak;
  memcpy(nakPacket + 1, &nakCRC, 4);
  packetSerial.send(nakPacket, 5);
}
