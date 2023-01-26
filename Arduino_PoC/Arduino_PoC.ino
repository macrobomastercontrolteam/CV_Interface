#define MSG_SPECIAL_CHAR_STX 0x02
#define MSG_SPECIAL_CHAR_ETX 0x03
#define MSG_ACK_BIT 0x40
#define MSG_TURN_ON_AUTO_AIM_MODE 0x01
#define MSG_TURN_OFF 0x00

#include <SoftwareSerial.h>
const byte virtualRxPin = 6;
const byte virtualTxPin = 7;
SoftwareSerial mySerial (virtualRxPin, virtualTxPin);

typedef enum {
  ERROR_MSG = 0x0,
  // Control hosted msgs
  MODE_CONTROL = 0x10,
  // CV hosted msgs
  ENEMY_COORDINATE = 0x20,
} eCv_MsgTypes;

typedef struct
{
  uint16_t targetXCoordinate;
  uint16_t targetYCoordinate;
} tCv_TargetCoordinateMsg;

uint8_t rxBuffer[20];  //Initialized variable to store receive
uint8_t txBuffer[10];  //Initialized variable to store receive
uint8_t rxBufferWrittenSize = 0;

eCv_MsgTypes cvMsgHandler(uint8_t* rxBuffer);

void setup() {
  Serial.begin(9600);
}

void loop() {
  // read from STX to ETX
  Serial.print("RECEIVER: ");
  int bytesToRead = Serial.available();
  int bytesToBuffer = min(bytesToRead, sizeof(rxBuffer) - rxBufferWrittenSize);
  Serial.readBytes((char*)rxBuffer[rxBufferWrittenSize], bytesToBuffer);
  rxBufferWrittenSize += bytesToBuffer;
  if (bytesToRead != 0) {
    Serial.println((char*)rxBuffer);
  } else {
    Serial.println();
  }

  // parse and handle
  eCv_MsgTypes receivedMsgType = cvMsgHandler(rxBuffer);

  delay(1000);
}

eCv_MsgTypes cvMsgHandler(uint8_t* rxBuffer) {
  uint8_t index;
  for (index = 0; index < 10;) //placeholder 
  {
    
  }
}
