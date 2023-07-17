#include <SPI.h>
#include "IBusBM.h"
#include "mcp_can.h"

#include "lookup.c"

#define CAN_500KBPS 16    // CAN baud rate
#define SPI_CS_PIN  17    // OK for Leonardo CAN board
#define LED 23            // LED pin to show program active

// #define SERIAL_DEBUG true   // set this to enable the serial port for debugging, NOTE: the code will not run if serial enabled but not connected
// bool SERIAL_DEBUG = true;
bool SERIAL_DEBUG = false;

MCP_CAN CAN(SPI_CS_PIN);    // Set CS pin
IBusBM IBus;

void setup()
{
  pinMode(LED, OUTPUT);

  if(SERIAL_DEBUG) { Serial.begin(115200); delay(500); }

  while(CAN_OK != CAN.begin(CAN_500KBPS))
  {
    if(SERIAL_DEBUG == true) { Serial.println("CAN BUS FAIL!"); }
    delay(100);
  }

  if(SERIAL_DEBUG == true) { Serial.println("CAN BUS OK!"); }

  IBus.begin(Serial1);  // OK for Leonardo CAN board without specifying pins
  if(SERIAL_DEBUG == true) { Serial.println("Start iBUS monitor"); }
}

const int16_t Steer_X[9] = {1000, 1125, 1250, 1490, 1500, 1510, 1750, 1875, 2000};	// iBUS 1000->2000
const int16_t Steer_Y[9] = {1000, 500,  250,  0,    0,    0,    -250, -500, -1000};
const uint16_t Steer_N = sizeof(Steer_X)/sizeof(Steer_X[0]);

const int16_t Torque_X[7] = {1000, 1125, 1250, 1500, 1750, 1875, 2000};	// iBUS 1000->2000
const int16_t Torque_Y[7] = {-100, -50,  -25,  0,    25,   50,   100};
const uint16_t Torque_N = sizeof(Torque_X)/sizeof(Torque_X[0]);

const unsigned long frameRepetitionTime = 5;  // repetition rate of the frame in ms
unsigned long previousTime;                    // last time for frame rate calcs

bool led = true;

byte CustomerControl1[8];
byte CustomerControl2[8];
byte count = 0;  // rolling counter for alive signal

int RollValue = 1500;
int PitchValue = 1500;
int YawValue = 1500;
int ThrottleValue = 1500;
int VrAValue = 1500;
int VrBValue = 1500;
int SwAValue = 1500;
int SwBValue = 1500;
int SwCValue = 1500;
int SwDValue = 1500;

int ControlMode = 0;

int16_t SteerScaler = 100;
int16_t SteerRate = 100;
int16_t SteerValue = 0;
int16_t TorqueValue = 0;
int16_t SteerValue_prev = 0;
int16_t SteerValueScaled = 0;
int16_t TorqueValueScaled = 0;
int8_t SteerValueScaled_8 = 0;

void loop()
{
  unsigned long currentTime = millis();   // grab the current time

  if(currentTime-previousTime >= frameRepetitionTime) {

    UpdateValues();

    switch (ControlMode) {
      default :
      case 0 : {
        SteerScaler = (VrBValue - 1000) / 10; if(SteerScaler < 10) { SteerScaler = 10; }  if(SteerScaler > 100) { SteerScaler = 100; }
        SteerRate = (VrAValue - 1000) / 10; if(SteerRate < 2) { SteerRate = 2; }  if(SteerRate > 100) { SteerRate = 100; }
        SteerValue = lookup_s16_s16(RollValue, Steer_X, Steer_Y, Steer_N);
        TorqueValue = lookup_s16_s16(ThrottleValue, Torque_X, Torque_Y, Torque_N);
        SteerValueScaled = scaleValue(SteerValue,SteerScaler);
        if(SteerValueScaled > (SteerValue_prev + SteerRate)) { SteerValueScaled = SteerValue_prev + SteerRate; }
        if(SteerValueScaled < (SteerValue_prev - SteerRate)) { SteerValueScaled = SteerValue_prev - SteerRate; }
        SteerValue_prev = SteerValueScaled;
        SteerValueScaled_8 = (byte)(SteerValueScaled / 10);
        CustomerControl1[2]= SteerValueScaled_8;
        CustomerControl1[3]= (byte)TorqueValue;
        break;
      }
      case 1 : {
        break;
      }
      case 2 : {
        break;
      }
      case 3 : {
        break;
      }
      case 4 : {
        break;
      }
      case 5 : {
        break;
      }
    }

    CustomerControl1[1] = count++;
    CustomerControl1[4] = 0;
    CustomerControl1[5] = 0;  
    CustomerControl1[6] = 0;
    CustomerControl1[7] = 0;  

    if(SwDValue > 1500) { CustomerControl1[7] += 0x11; }

    //CustomerControl1[0] = Compute_CRC8(CustomerControl1);

		CustomerControl1[0] = CRC_calc(&CustomerControl1[0]);

    CAN.sendMsgBuf(0x101, 0, 8, CustomerControl1 ); // send the CAN message

    previousTime = currentTime; // set previous time for next timer loop
  }  // if currentTime-previousTime

  if(0 == currentTime % 500) {
    led = !led;
  }

  digitalWrite(LED,led);
}  // loop

void UpdateValues() {
    RollValue = IBus.readChannel(0);
    PitchValue = IBus.readChannel(1);
    ThrottleValue = IBus.readChannel(2);
    YawValue = IBus.readChannel(3);
    VrAValue = IBus.readChannel(4);
    VrBValue = IBus.readChannel(5);
    SwAValue = IBus.readChannel(6);
    SwBValue = IBus.readChannel(7);
    SwCValue = IBus.readChannel(8);
    SwDValue = IBus.readChannel(9);

    if(SwBValue < 1500) {
      if(SwCValue < 1250) {
        ControlMode = 0;
      } else if(SwCValue < 1750) {
        ControlMode = 1;
      } else {
        ControlMode = 2;
      }
    } else {
      if(SwCValue < 1250) {
        ControlMode = 3;
      } else if(SwCValue < 1750) {
        ControlMode = 4;
      } else {
        ControlMode = 5;
      }
    }
    
    if(SERIAL_DEBUG == true) {
      for (int i=0; i<10 ; i++) {
        Serial.print(IBus.readChannel(i)); Serial.print(" ");
      }
      Serial.print(ControlMode); Serial.print(" ");
      Serial.print(SteerValue); Serial.print(" ");
      Serial.print(TorqueValue); Serial.print(" ");
      Serial.print(SteerScaler); Serial.print(" ");
      Serial.print(SteerRate); Serial.print(" ");
      Serial.print(SteerValueScaled); Serial.print(" ");
      // Serial.print(SteerValueScaled_8); Serial.print(" ");
      Serial.print(TorqueValueScaled); Serial.print(" ");
      Serial.println();
    }
}

int16_t scaleValue(int16_t value, int16_t scaler) { // -100 to 100, scaled 10 to 100%
  int16_t value_scaled = ((int32_t)value * scaler) / 100; // -10000 to 10000 max
  return value_scaled;
}

const uint8_t crc8_data[256] =
{
	0x00,0x1d,0x3a,0x27,0x74,0x69,0x4e,0x53,0xe8,0xf5,0xd2,0xcf,0x9c,0x81,0xa6,0xbb,
	0xcd,0xd0,0xf7,0xea,0xb9,0xa4,0x83,0x9e,0x25,0x38,0x1f,0x02,0x51,0x4c,0x6b,0x76,
	0x87,0x9a,0xbd,0xa0,0xf3,0xee,0xc9,0xd4,0x6f,0x72,0x55,0x48,0x1b,0x06,0x21,0x3c,
	0x4a,0x57,0x70,0x6d,0x3e,0x23,0x04,0x19,0xa2,0xbf,0x98,0x85,0xd6,0xcb,0xec,0xf1,
	0x13,0x0e,0x29,0x34,0x67,0x7a,0x5d,0x40,0xfb,0xe6,0xc1,0xdc,0x8f,0x92,0xb5,0xa8,
	0xde,0xc3,0xe4,0xf9,0xaa,0xb7,0x90,0x8d,0x36,0x2b,0x0c,0x11,0x42,0x5f,0x78,0x65,
	0x94,0x89,0xae,0xb3,0xe0,0xfd,0xda,0xc7,0x7c,0x61,0x46,0x5b,0x08,0x15,0x32,0x2f,
	0x59,0x44,0x63,0x7e,0x2d,0x30,0x17,0x0a,0xb1,0xac,0x8b,0x96,0xc5,0xd8,0xff,0xe2,
	0x26,0x3b,0x1c,0x01,0x52,0x4f,0x68,0x75,0xce,0xd3,0xf4,0xe9,0xba,0xa7,0x80,0x9d,
	0xeb,0xf6,0xd1,0xcc,0x9f,0x82,0xa5,0xb8,0x03,0x1e,0x39,0x24,0x77,0x6a,0x4d,0x50,
	0xa1,0xbc,0x9b,0x86,0xd5,0xc8,0xef,0xf2,0x49,0x54,0x73,0x6e,0x3d,0x20,0x07,0x1a,
	0x6c,0x71,0x56,0x4b,0x18,0x05,0x22,0x3f,0x84,0x99,0xbe,0xa3,0xf0,0xed,0xca,0xd7,
	0x35,0x28,0x0f,0x12,0x41,0x5c,0x7b,0x66,0xdd,0xc0,0xe7,0xfa,0xa9,0xb4,0x93,0x8e,
	0xf8,0xe5,0xc2,0xdf,0x8c,0x91,0xb6,0xab,0x10,0x0d,0x2a,0x37,0x64,0x79,0x5e,0x43,
	0xb2,0xaf,0x88,0x95,0xc6,0xdb,0xfc,0xe1,0x5a,0x47,0x60,0x7d,0x2e,0x33,0x14,0x09,
	0x7f,0x62,0x45,0x58,0x0b,0x16,0x31,0x2c,0x97,0x8a,0xad,0xb0,0xe3,0xfe,0xd9,0xc4
};

static uint8_t CRC_calc(byte* bytes) {
	uint8_t data = 0xFF;

	data = crc8_data[data ^ bytes[1]];
	data = crc8_data[data ^ bytes[2]];
	data = crc8_data[data ^ bytes[3]];
	data = crc8_data[data ^ bytes[4]];
	data = crc8_data[data ^ bytes[5]];
	data = crc8_data[data ^ bytes[6]];
	data = crc8_data[data ^ bytes[7]];
	
	return(data);
}
