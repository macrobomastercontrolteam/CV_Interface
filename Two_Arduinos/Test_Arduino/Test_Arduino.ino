#include <SoftwareSerial.h>  // SoftwareSerial

/********* macro start *********/
#define TRANSMISSION_TIMEOUT 5000UL
#define DATA_PACKAGE_SIZE 7
#define DATA_PACKAGE_PAYLOAD_SIZE (DATA_PACKAGE_SIZE - sizeof(uint8_t) * 3)
/********* macro end *********/

/********* typedefs start *********/
typedef enum {
  MSG_MODE_CONTROL = 0x10,
  MSG_COORDINATE = 0x20,
  MSG_ACK = 0x40,
} eMsgTypes;

typedef enum {
  CHAR_STX = 0x02,
  CHAR_ETX = 0x03,
  CHAR_UNUSED = 0xFF,
} eCharTypes;

typedef enum {
  MODE_TURN_OFF = 0x00,
  MODE_TURN_ON_AUTO_AIM = 0x01,
} eModeControlTypes;

typedef struct
{
  uint16_t uiXCoordinate;
  uint16_t uiYCoordinate;
  bool fCoordinateValid;
} tCoordinateHandler;

typedef union {
  struct {
    uint8_t bStx;  // supposed be CHAR_STX
    uint8_t bMsgType;
    uint8_t abPayload[DATA_PACKAGE_PAYLOAD_SIZE];
    uint8_t bEtx;  // supposed to be CHAR_ETX
  };
  uint8_t abData[DATA_PACKAGE_SIZE];
} tMsgBuffer;
/********* typedefs end *********/

/********* function declaration start *********/
bool MsgReader_Heartbeat(bool *pfRxMsgComplete);
bool MsgHandler_Heartbeat(bool *pfRxMsgComplete, bool fStartCvSignal, tCoordinateHandler *pCoordinateHandler);
bool IsTimeout(uint32_t ulNewTime, uint32_t ulOldTime, uint32_t ulTimeout);
/********* function declaration end *********/

/********* module variable definitions start *********/
// Serial is USB interface with PC; used to flash Arduino and emulate communication between CV (mocked by PC) and control (mocked by arduino)
// scannerSerial is UART interface with another arduino acting as scanner (check "Second Arduino.ino"); used to echo Rx data to scanner
const byte virtualRxPin = 6;
const byte virtualTxPin = 7;
SoftwareSerial scannerSerial(virtualRxPin, virtualTxPin);

tMsgBuffer rxBuffer;
tMsgBuffer txBuffer;
/********* module variable definitions end *********/

void setup() {
  Serial.begin(9600);
  scannerSerial.begin(9600);
  txBuffer.bStx = CHAR_STX;
  txBuffer.bEtx = CHAR_ETX;
}

void loop() {
  static bool fRxMsgComplete = false;
  MsgReader_Heartbeat(&fRxMsgComplete);

  bool fStartCvSignal = true;                  // mock user signal that enables auto-aim mode
  tCoordinateHandler CoordinateHandler;        // will be used by gimbal
  CoordinateHandler.fCoordinateValid = false;  // pretend coordinate is used up by gimbal elsewhere before each loop
  MsgHandler_Heartbeat(&fRxMsgComplete, fStartCvSignal, &CoordinateHandler);

  delay(1000);
}

bool MsgReader_Heartbeat(bool *pfRxMsgComplete) {
  // restart reading only after Rx Msg is processed, indicated by (*pfRxMsgComplete == false)
  if (*pfRxMsgComplete == false) {
    // read from STX to ETX
    static bool fStxReceived = false;
    if (fStxReceived || ((Serial.readBytes(rxBuffer.abData, 1) == 1) && (rxBuffer.bStx == CHAR_STX))) {
      fStxReceived = false;
      if (Serial.readBytes(&rxBuffer.abData[1], DATA_PACKAGE_SIZE - 1) == DATA_PACKAGE_SIZE - 1) {
        if (rxBuffer.bEtx == CHAR_ETX) {
          *pfRxMsgComplete = true;
          // echo to scanner
          scannerSerial.write(rxBuffer.abData, DATA_PACKAGE_SIZE);
        } else {
          // unsynched
          for (uint8_t bDataIndex = 1; bDataIndex < DATA_PACKAGE_SIZE; bDataIndex++) {
            if (rxBuffer.abData[bDataIndex] == CHAR_STX) {
              // skip checking STX on next loop
              fStxReceived = true;
              break;
            }
          }
        }
      }
    }
  }
  return *pfRxMsgComplete;
}

bool MsgHandler_Heartbeat(bool *pfRxMsgComplete, bool fStartCvSignal, tCoordinateHandler *pCoordinateHandler) {
  typedef enum {
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
  bool fCoordiateReady = false;

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
        if ((fStartCvSignal == false)  // global user interrupt
            || IsTimeout(ulNewRxTimestamp, ulOldRxTimestamp, TRANSMISSION_TIMEOUT)) {
          HandlerState = HANDLER_DISABLE_CV;
        } else {
          txBuffer.bMsgType = MSG_MODE_CONTROL;
          memset(txBuffer.abPayload, CHAR_UNUSED, DATA_PACKAGE_PAYLOAD_SIZE);
          txBuffer.abPayload[0] = MODE_TURN_ON_AUTO_AIM;
          Serial.write(txBuffer.abData, DATA_PACKAGE_SIZE);

          HandlerState = HANDLER_WAIT_FOR_ACK;
          NextHandlerState = HANDLER_WAIT_FOR_COORDINATE;
        }
        break;
      }
    case HANDLER_WAIT_FOR_COORDINATE:
      {
        if ((fStartCvSignal == false)  // global user interrupt
            || IsTimeout(ulNewRxTimestamp, ulOldRxTimestamp, TRANSMISSION_TIMEOUT)) {
          HandlerState = HANDLER_DISABLE_CV;
        } else if (*pfRxMsgComplete) {
          *pfRxMsgComplete = false;
          ulOldRxTimestamp = ulNewRxTimestamp;
          if (rxBuffer.bMsgType == MSG_COORDINATE) {
            memcpy(pCoordinateHandler, rxBuffer.abPayload, DATA_PACKAGE_PAYLOAD_SIZE);
            pCoordinateHandler->fCoordinateValid = true;
            // HandlerState = ; // stay in this state until user interrupt or transmission timeout
          }
        }
        break;
      }
    case HANDLER_DISABLE_CV:
      {
        txBuffer.bMsgType = MSG_MODE_CONTROL;
        memset(txBuffer.abPayload, CHAR_UNUSED, DATA_PACKAGE_PAYLOAD_SIZE);
        txBuffer.abPayload[0] = MODE_TURN_OFF;
        Serial.write(txBuffer.abData, DATA_PACKAGE_SIZE);

        HandlerState = HANDLER_IDLE;
        break;
      }
    case HANDLER_WAIT_FOR_ACK:
      {
        if ((fStartCvSignal == false)  // global user interrupt
            || IsTimeout(ulNewRxTimestamp, ulOldRxTimestamp, TRANSMISSION_TIMEOUT)) {
          HandlerState = HANDLER_DISABLE_CV;
        } else if (*pfRxMsgComplete) {
          *pfRxMsgComplete = false;
          ulOldRxTimestamp = ulNewRxTimestamp;
          if ((rxBuffer.bMsgType == MSG_ACK)
              && (memcmp(rxBuffer.abPayload, "ACK", sizeof("ACK")) == 0)
              && (rxBuffer.abPayload[DATA_PACKAGE_PAYLOAD_SIZE - 1] == CHAR_UNUSED)) {
            HandlerState = NextHandlerState;
          }
        }
        break;
      }
  }
  return fCoordiateReady;
}

bool IsTimeout(uint32_t ulNewTime, uint32_t ulOldTime, uint32_t ulTimeout) {
  // works even with overflow
  return ((uint32_t)(ulNewTime - ulOldTime) > ulTimeout);
}