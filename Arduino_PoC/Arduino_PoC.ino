#define MSG_SPECIAL_CHAR_STX 0x02
#define MSG_SPECIAL_CHAR_ETX 0x03
#define MSG_ACK_BIT 0x40
#define MSG_TURN_ON 0x01
#define MSG_TURN_OFF 0x00

typedef enum {
  ERROR_MSG = 0x00,
  // Control hosted msgs
  MODE_CONTROL = 0x10,
  // CV hosted msgs
  ENEMY_COORDINATE = 0x20,
} eCvInterfaceMsgTypes;

typedef struct
{
  uint8_t STX;
  uint8_t cvInterfaceMsgType;
  uint16_t targetXCoordinate;
  uint16_t targetYCoordinate;
  uint8_t ETX;
} tCvInterfaceTargetCoordinateMsg;

uint8_t RxBuffer[20];  //Initialized variable to store receive
uint8_t TxBuffer[10];  //Initialized variable to store receive

eCvInterfaceMsgTypes cvMsgParser(uint8_t* RxBuffer);
void cvMsgResponder(eCvInterfaceMsgTypes parsedMsgType);

void setup() {
  Serial.begin(9600);
}

void loop() {
  // read
  Serial.print("RECEIVER: ");
  int bytesToRead = Serial.available();
  Serial.readBytes((char*)RxBuffer, min(bytesToRead, sizeof(RxBuffer)));
  if (bytesToRead != 0) {
    Serial.println((char*)RxBuffer);
  } else {
    Serial.println();
  }

  // parse
  eCvInterfaceMsgTypes parsedMsgType = cvMsgParser(RxBuffer);

  // responder
  cvMsgResponder(parsedMsgType);

  delay(1000);
}

eCvInterfaceMsgTypes cvMsgParser(uint8_t* RxBuffer) {
  for
}

void cvMsgResponder(eCvInterfaceMsgTypes parsedMsgType) {
}
