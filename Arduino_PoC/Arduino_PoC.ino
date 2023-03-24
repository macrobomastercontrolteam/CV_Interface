#define TWO_ARDUINO_SETUP
// #define ARDUINO_RPI_SETUP

#include <SoftwareSerial.h>  // SoftwareSerial
// this is control, jupiter is cv
/********* macro start *********/
#define DATA_PACKAGE_SIZE 15
#define DATA_PACKAGE_PAYLOAD_SIZE (DATA_PACKAGE_SIZE - sizeof(uint8_t) * 3)
/********* macro end *********/

/********* typedefs start *********/
// for compability of control code
typedef float fp32;

typedef enum {
  MSG_MODE_CONTROL = 0x10,
  MSG_CV_CMD = 0x20,
  MSG_ACK = 0x40,
} eMsgTypes;

typedef enum {
  CHAR_STX = 0x02,
  CHAR_ETX = 0x03,
  CHAR_UNUSED = 0xFF,
} eCharTypes;

typedef enum {
  MODE_AUTO_AIM_BIT = 0b00000001,
  MODE_AUTO_MOVE_BIT = 0b00000010,
  MODE_ENEMY_DETECTED_BIT = 0b00000100,
} eModeControlBits;

typedef struct
{
  uint16_t uiXCoordinate;
  uint16_t uiYCoordinate;
  fp32 xSpeed;
  fp32 ySpeed;
} tCvCmdMsg;

/**
 * @brief package header of X,Y coordinates and speeds from CV to control
 */
typedef struct
{
  tCvCmdMsg CvCmdMsg;
  bool fCvCmdValid;  ///< whether CvCmdMsg is valid as received from CV; to be used by gimbal_task
  bool fRxMsgComplete;
  bool fIsWaitingForAck;
  bool fIsStxReceived;
  uint8_t fCvMode;  ///< contains individual CV control flag bits defined by eModeControlBits
} tCvCmdHandler;

/**
 * @brief buffer in which you piece together
 */
typedef union {
  struct
  {
    uint8_t bStx;  ///< supposed to be CHAR_STX in actual msg
    uint8_t bMsgType;
    uint8_t abPayload[DATA_PACKAGE_PAYLOAD_SIZE];
    uint8_t bEtx;  ///< supposed to be CHAR_ETX in actual msg
  };
  uint8_t abData[DATA_PACKAGE_SIZE];
} tMsgBuffer;
/********* typedefs end *********/

/********* function declaration start *********/
void MsgRxHandler_ReaderHeartbeat(tCvCmdHandler *pCvCmdHandler);
void MsgRxHandler_Parser(tCvCmdHandler *pCvCmdHandler);
void MsgTxHandler_SendSetModeRequest(tCvCmdHandler *pCvCmdHandler);
bool IsCvModeOn(uint8_t bCvModeBit);
/********* function declaration end *********/

/********* module variable definitions start *********/
const byte virtualRxPin = 6;
const byte virtualTxPin = 7;  // defines which pins on the board you are sending these messages through
SoftwareSerial virtualSerial(virtualRxPin, virtualTxPin);

#if defined(ARDUINO_RPI_SETUP)
#define cvSerial virtualSerial
#define scannerSerial Serial
#elif defined(TWO_ARDUINO_SETUP)
#define cvSerial Serial
#define scannerSerial virtualSerial
#endif

const int buttonPin = 2;

tMsgBuffer rxBuffer;
tMsgBuffer txBuffer;
tCvCmdHandler CvCmdHandler;
// don't compare with "ACK", since it contains extra NULL char at the end
const uint8_t abExpectedAckPayload[3] = { 'A', 'C', 'K' };
uint8_t abExpectedUnusedPayload[DATA_PACKAGE_PAYLOAD_SIZE];
/********* module variable definitions end *********/

void setup() {
  scannerSerial.begin(9600);
  cvSerial.begin(9600);
  txBuffer.bStx = CHAR_STX;
  txBuffer.bEtx = CHAR_ETX;
  pinMode(buttonPin, INPUT);

  memset(&CvCmdHandler, 0, sizeof(CvCmdHandler));  // clear status
  memset(abExpectedUnusedPayload, CHAR_UNUSED, sizeof(abExpectedUnusedPayload));
}

void loop() {
  // mock event of user enabling/disabling CV control mode by pressing button
  if (digitalRead(buttonPin) == HIGH) {
    static uint8_t bMockCounter = 0;
    switch (bMockCounter) {
      case 0:
        {
          CvCmdHandler.fCvMode = 0;
          // scannerSerial.println("Sent: enemy");
          break;
        }
      case 1:
        {
          CvCmdHandler.fCvMode = MODE_ENEMY_DETECTED_BIT;
          // scannerSerial.println("Sent: enemy");
          break;
        }
      case 2:
        {
          CvCmdHandler.fCvMode = MODE_AUTO_MOVE_BIT | MODE_ENEMY_DETECTED_BIT;
          // scannerSerial.println("Sent: move | enemy");
          break;
        }
      case 3:
        {
          CvCmdHandler.fCvMode = MODE_AUTO_AIM_BIT | MODE_AUTO_MOVE_BIT | MODE_ENEMY_DETECTED_BIT;
          // scannerSerial.println("Sent: all");
          break;
        }
    }
    bMockCounter = (bMockCounter + 1) % 4;
    MsgTxHandler_SendSetModeRequest(&CvCmdHandler);
  }

  MsgRxHandler_ReaderHeartbeat(&CvCmdHandler);
  MsgRxHandler_Parser(&CvCmdHandler);
  delay(100);
}

void MsgRxHandler_ReaderHeartbeat(tCvCmdHandler *pCvCmdHandler) {
  // restart reading only after Rx Msg is processed, indicated by (*pfRxMsgComplete == false)
  if (pCvCmdHandler->fRxMsgComplete == false) {
    // read from STX to ETX
    // if you recieved the stx OR ( (there's at least one byte in the rx buffer) AND (the stx value in the rx buffer is what you expect) )
    if (pCvCmdHandler->fIsStxReceived || ((cvSerial.readBytes(rxBuffer.abData, 1) == 1) && (rxBuffer.bStx == CHAR_STX))) {
      pCvCmdHandler->fIsStxReceived = false;
      if (cvSerial.readBytes(&rxBuffer.abData[1], DATA_PACKAGE_SIZE - 1) == DATA_PACKAGE_SIZE - 1) {  // if you successfully read the next few bytes
        if (rxBuffer.bEtx == CHAR_ETX) {                                                              // and the ETX is correct
          pCvCmdHandler->fRxMsgComplete = true;
        } else {
          // unsynched
          for (uint8_t bDataIndex = 1; bDataIndex < DATA_PACKAGE_SIZE; bDataIndex++) {  // keep iterating until you resynch
            if (rxBuffer.abData[bDataIndex] == CHAR_STX) {
              // skip checking STX on next loop
              pCvCmdHandler->fIsStxReceived = true;
              break;
            }
          }
        }
      }
    }
  }
}

void MsgRxHandler_Parser(tCvCmdHandler *pCvCmdHandler) {
  if (pCvCmdHandler->fRxMsgComplete) {
    pCvCmdHandler->fRxMsgComplete = false;
    // scannerSerial.println("Received msg: " + rxBuffer.bMsgType);

    bool fInvalid = false;
    uint8_t bDataCursor;
    switch (rxBuffer.bMsgType) {
      case MSG_CV_CMD:
        {
          fInvalid |= ((IsCvModeOn(MODE_AUTO_MOVE_BIT) == false) && (IsCvModeOn(MODE_AUTO_AIM_BIT) == false));
          fInvalid |= pCvCmdHandler->fIsWaitingForAck;
          fInvalid |= (memcmp(&rxBuffer.abPayload[sizeof(tCvCmdMsg)], abExpectedUnusedPayload, DATA_PACKAGE_PAYLOAD_SIZE - sizeof(tCvCmdMsg)) != 0);

          if (fInvalid == false) {
            memcpy(&(pCvCmdHandler->CvCmdMsg), rxBuffer.abPayload, sizeof(pCvCmdHandler->CvCmdMsg));
            pCvCmdHandler->fCvCmdValid = true;
          } else {
            pCvCmdHandler->fCvCmdValid = false;
          }
          break;
        }
      case MSG_ACK:
        {
          fInvalid |= (memcmp(&rxBuffer.abPayload[sizeof(abExpectedAckPayload)], abExpectedUnusedPayload, DATA_PACKAGE_PAYLOAD_SIZE - sizeof(abExpectedAckPayload)) != 0);
          fInvalid |= (memcmp(rxBuffer.abPayload, abExpectedAckPayload, sizeof(abExpectedAckPayload)) != 0);
          if (fInvalid == false) {
            pCvCmdHandler->fIsWaitingForAck = false;
          }
          break;
        }
      default:
        {
          fInvalid = true;
          break;
        }
    }

    // ignore invalid msg
    if (fInvalid) {
      // scannerSerial.println("Ignored msg: " + rxBuffer.bMsgType);
    } else {
      // echo to scanner
      scannerSerial.write(rxBuffer.abData, DATA_PACKAGE_SIZE);
    }
  }
}

void MsgTxHandler_SendSetModeRequest(tCvCmdHandler *pCvCmdHandler) {
  txBuffer.bMsgType = MSG_MODE_CONTROL;                                // add msg to the tx bugger - 0x10
  memset(txBuffer.abPayload, CHAR_UNUSED, DATA_PACKAGE_PAYLOAD_SIZE);  // set the next few spaces to the empty 0xFF
  txBuffer.abPayload[0] = CvCmdHandler.fCvMode;                        // turn on auto aim mode 0x01
  cvSerial.write(txBuffer.abData, DATA_PACKAGE_SIZE);                  // write it to the scannerSerial - the ETX and STX have been set up in setup so dont panic
  scannerSerial.write(txBuffer.abData, DATA_PACKAGE_SIZE);             // echo to scanner

  pCvCmdHandler->fCvCmdValid = pCvCmdHandler->fCvCmdValid && (IsCvModeOn(MODE_AUTO_MOVE_BIT) || IsCvModeOn(MODE_AUTO_AIM_BIT));
  pCvCmdHandler->fIsWaitingForAck = true;
}

bool IsCvModeOn(uint8_t bCvModeBit) {
  return (CvCmdHandler.fCvMode & bCvModeBit);
}