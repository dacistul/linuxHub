#include <Arduino.h>
#include "HID-Project.h"
#include <USBCore.h>
#include <string.h>

constexpr char VERSION[] = "Fan Controller v0.1";

uint16_t readInterval = 1000;

// first fan
const uint8_t f1Timer = 9; // OC1A
const uint8_t f1Tach = 3; // INT0
volatile uint16_t f1Counter = 0;
uint8_t f1Rpm = 2400;
uint16_t f1Time = 0;

// second fan
const uint8_t f2Timer = 5; // OC3A
const uint8_t f2Tach = 2; // INT1
volatile uint16_t f2Counter = 0;
uint8_t f2Rpm = 2400;
uint16_t f2Time = 0;

const uint8_t fTmr[2] = {
  f1Timer, f2Timer
};

const uint8_t fTime[2] = {
  f1Time, f2Time
};

const uint8_t fRpm[2] = {
  f1Rpm, f2Rpm
};

enum arduinopwm_cmd : u8 {
	ARDUINOPWM_CMD_SETPWM=0x01, /* data: index u8, pwm value u8 */
	ARDUINOPWM_CMD_GETRPM=0x02, /* data: index u8 */
	ARDUINOPWM_CMD_GETVER=0x10, /* no data */
	ARDUINOPWM_CMD_SETREADINTERVAL=0x11, /* data: time in ms u16 */
};
enum arduinopwm_ack : u8 {
	ARDUINOPWM_ACK_SETPWM=0x01, /* data: index u8, pwm value u8 */
	ARDUINOPWM_ACK_GETRPM=0x02, /* data: index u8, rpm value u16 */
	ARDUINOPWM_ACK_GETVER=0x10, /* data: version u32 */
};
union arduinopwm_cmd_ack{
	enum arduinopwm_cmd cmd;
	enum arduinopwm_ack ack;
};

struct arduinopwm_packet {
    u8 startByte;
    union arduinopwm_cmd_ack ca;
    u8 dataLength;
    u8 checksum;
    u8 data[0];
};

void receivePacket();
void handleSetPWM(uint8_t index, uint8_t dutyCycle);
void handleGetRPM(uint8_t index);
void handleGetVersion();
void sendPacket(arduinopwm_ack ack, uint8_t len, uint8_t* data);
void readPWM();
void countFall1();
void countFall2();

uint8_t rawhidData[255];

void setup() {
  // first fan setup
  pinMode(f1Timer, OUTPUT);
  TCCR1A = (1 << COM1A1);
  TCCR1A |= (1 << COM1B1);
  TCCR1A |= (1 << WGM10);
  TCCR1B = (1 << WGM12);
  TCCR1B |= (1 << CS11);
  OCR1A = 0;
  attachInterrupt(digitalPinToInterrupt(f1Tach), countFall1, FALLING);

  // second fan setup
  pinMode(f2Timer, OUTPUT);
  TCCR3A = (1 << COM3A1);
  TCCR3A |= (1 << COM3B1);
  TCCR3A |= (1 << WGM30);
  TCCR3B = (1 << WGM32);
  TCCR3B |= (1 << CS31);
  OCR3A = 0;
  attachInterrupt(digitalPinToInterrupt(f2Tach), countFall2, FALLING);

  Serial.begin(115200);

  RawHID.begin(rawhidData, sizeof(rawhidData));

  delay(1000);
  handleGetVersion();
}

void loop() {
  readPWM();

  receivePacket();
}

void receivePacket()
{
  arduinopwm_cmd cmd;
  uint8_t dataLen;
  uint8_t dataChecksum;
  static uint8_t dataBuffer[64];

  auto bytesAvailable = RawHID.available();
  if(bytesAvailable > 4 && RawHID.read() == 0x00)
  {
    cmd = RawHID.read();
    dataLen = RawHID.read();
    dataChecksum = RawHID.read();
    if(RawHID.readBytes(dataBuffer, dataLen) != dataLen) return;
  }
  else return;
  
  Serial.println("Command: " + String(cmd) + "; Len: " + String(dataLen) +
                  "; Checksum: " + String(dataChecksum) + "; data: " + 
                  String(dataBuffer[0]) + String(dataBuffer[1]));
  
  // TODO checksum here

  switch(cmd)
  {
    case ARDUINOPWM_CMD_SETPWM:
      handleSetPWM(dataBuffer[0], dataBuffer[1]);
      break;
    case ARDUINOPWM_CMD_GETRPM:
      handleGetRPM(dataBuffer[0]);
      break;
    case ARDUINOPWM_CMD_GETVER:
      handleGetVersion();
      break;
    case ARDUINOPWM_CMD_SETREADINTERVAL:
      readInterval = dataBuffer[0];
      break;
    default:
      break;
  }
}

void handleSetPWM(uint8_t index, uint8_t dutyCycle)
{
  if(index < 2) analogWrite(fTmr[index], dutyCycle * 255 / 100);
  uint8_t retData[]={index, dutyCycle};
  sendPacket(ARDUINOPWM_ACK_SETPWM, sizeof(retData), retData);
}

void handleGetRPM(uint8_t index)
{
  uint8_t retData[]={index, fRpm[index]};
  sendPacket(ARDUINOPWM_ACK_GETRPM, sizeof(retData), retData);
}

void handleGetVersion()
{
  sendPacket(ARDUINOPWM_ACK_GETVER, sizeof(VERSION), VERSION);
}

void sendPacket(arduinopwm_ack ack, uint8_t len, uint8_t* data)
{
  static uint8_t buffer[64];
  auto& packet = *(arduinopwm_packet*)&buffer;

  packet.startByte = 0x00;
  packet.ca.ack = ack;
  packet.dataLength = len;
  packet.checksum = 0x42;
  //TODO checksum
  memcpy(packet.data, data, len);

  RawHID.write(buffer, sizeof(arduinopwm_packet) + len);
}

void readPWM()
{
  if(millis() - f1Time > 1000UL) {
    f1Rpm = f1Counter * 60 / 2;
    f1Counter = 0;

    f1Time = millis();
  }

  if(millis() - f2Time > 1000UL) {
    f2Rpm = f2Counter * 60 / 2;
    f2Counter = 0;

    f2Time = millis();
  }
}

void countFall1() {
  f1Counter++;
}

void countFall2() {
  f2Counter++;
}
