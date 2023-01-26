#include <SoftwareSerial.h>  // SoftwareSerial

/********* macro start *********/
#define SESSION_TIMEOUT 5000
#define DATA_PACKAGE_SIZE 7
/********* macro end *********/

/********* function declaration start *********/
bool MsgReader_Heartbeat(bool *pfRxMsgComplete);
void MsgHandler_Heartbeat(bool *pfMsgReceived, bool fStartCvSignal);
/********* function declaration end *********/

/********* typedefs start *********/
typedef enum {
  ERROR_MSG = 0x00,
  MODE_CONTROL = 0x10,
  ENEMY_COORDINATE = 0x20,
  ACK_MSG = 0x40,
} eMsgTypes;

typedef enum {
  CHAR_STX = 0x02,
  CHAR_ETX = 0x03,
} eCharTypes;

typedef enum {
  TURN_OFF_MODE = 0x00,
  TURN_ON_AUTO_AIM_MODE = 0x01,
} eModeControl;

typedef struct
{
  uint16_t targetXCoordinate;
  uint16_t targetYCoordinate;
} tTargetCoordinateMsg;

typedef struct {
  union {
    struct {
      const uint8_t bStx;  // always CHAR_STX
      uint8_t bMsgType;
      uint8_t abPayload[DATA_PACKAGE_SIZE - sizeof(uint8_t) * 3];
      const uint8_t bEtx;  // always CHAR_ETX
    };
    char bData[DATA_PACKAGE_SIZE];
  } uint8_t bUsedSize;
} tMsgBuffer;
/********* typedefs end *********/

/********* module variable definitions start *********/
// Serial is USB interface with RPi; used to flash Arduino and view real-time output
// mySerial is UART interface with RPi; used to emulate CV communication
const byte virtualRxPin = 6;
const byte virtualTxPin = 7;
SoftwareSerial mySerial(virtualRxPin, virtualTxPin);

tMsgBuffer rxBuffer;
tMsgBuffer txBuffer;
/********* module variable definitions end *********/

void setup() {
  Serial.begin(9600);
  mySerial.begin(9600);
}

void loop() {
  static bool fRxMsgComplete = false;
  MsgReader_Heartbeat(&fRxMsgComplete);
  bool fStartCvSignal = true;  // mock user signal that enables auto-aim mode
  MsgHandler_Heartbeat(&fRxMsgComplete, fStartCvSignal);
  delay(1000);
}

bool MsgReader_Heartbeat(bool *pfRxMsgComplete) {
  if (*pfRxMsgComplete == false) {
    // restart reading only after Rx Msg is processed, indicated by (*pfRxMsgComplete == false)
    // read from STX to ETX
    if ((mySerial.readBytes(rxBuffer, 1) == 1) && (rxBuffer[0] == CHAR_STX)) {
      rxBufferSize = 1;
      while ((rxBufferSize < sizeof(rxBuffer)) && mySerial.readBytes(&rxBuffer[rxBufferSize], 1) == 1) {
        if (rxBuffer[rxBufferSize] == CHAR_ETX) {
          *pfRxMsgComplete = true;
          // echo to scanner
          mySerial.write(rxBuffer, rxBufferSize);
          rxBufferSize = 0;
          break;
        } else if (rxBuffer[rxBufferSize] == CHAR_STX) {
          // restart reading from STX
          rxBufferSize = 1;
        } else {
          rxBufferSize++;
        }
      }
    }
  }
  return *pfRxMsgComplete;
}

void MsgHandler_Heartbeat(bool *pfRxMsgComplete, bool fStartCvSignal) {
  enum {
    HANDLER_IDLE,
    HANDLER_ENABLE_CV,
    HANDLER_WAIT_FOR_COORDINATE,
    HANDLER_DISABLE_CV,
    HANDLER_WAIT_FOR_ACK,
  } tHandlerState;
  static tHandlerState HandlerState = HANDLER_IDLE;
  static tHandlerState NextHandlerState = HANDLER_IDLE;
  static uint32_t ulOldRxTimestamp = 0;
  uint32_t ulNewRxTimestamp = millis();
  bool fMsgProcessed = false;

  // global user interrupt
  if (fStartCvSignal == false) {
    HandlerState = HANDLER_DISABLE_CV;
  }

  txBufferSize = 0;

  switch (HandlerState) {
    case HANDLER_IDLE:
      {
        if (fStartCvSignal) {
          HandlerState = HANDLER_ENABLE_CV;
        }
        break;
      }
    case HANDLER_ENABLE_CV:
      {
        if ((unsigned long)(ulNewRxTimestamp - ulOldRxTimestamp) > SESSION_TIMEOUT) {  // works even with overflow
          HandlerState = HANDLER_DISABLE_CV;
        } else {
          txBuffer[txBufferSize++] = CHAR_STX;
          txBuffer[txBufferSize++] = MODE_CONTROL;
          txBuffer[txBufferSize++] = TURN_ON_AUTO_AIM_MODE;
          txBuffer[txBufferSize++] = CHAR_ETX;
          HandlerState = HANDLER_WAIT_FOR_ACK;
          NextHandlerState = HANDLER_WAIT_FOR_COORDINATE;
        }
        break;
      }
    case HANDLER_WAIT_FOR_COORDINATE:
      {
        if ((unsigned long)(ulNewRxTimestamp - ulOldRxTimestamp) > SESSION_TIMEOUT) {  // works even with overflow
          HandlerState = HANDLER_DISABLE_CV;
        } else {
        }
        break;
      }
    case HANDLER_DISABLE_CV:
      {
        HandlerState = HANDLER_ENABLE_CV;
        break;
      }
    case HANDLER_WAIT_FOR_ACK:
      {
        if ((unsigned long)(ulNewRxTimestamp - ulOldRxTimestamp) > SESSION_TIMEOUT) {  // works even with overflow
          HandlerState = HANDLER_DISABLE_CV;
        } else if (*pfRxMsgComplete) {
          if (rxBuffer[1] == ACK_MSG) {
            HandlerState = NextHandlerState;
          }
          fMsgProcessed = true;
        }
        break;
      }
  }

  if (*pfRxMsgComplete) {
    ulOldRxTimestamp = ulNewRxTimestamp;
  }

  // must check after timestamp is refreshed
  if (fMsgProcessed) {
    *pfRxMsgComplete = false;
  }

  if (txBufferSize > 0) {
    mySerial.write(txBuffer, txBufferSize);
  }
}
