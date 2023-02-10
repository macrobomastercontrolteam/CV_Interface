#include <SoftwareSerial.h>  // SoftwareSerial
//this is control, jupiter is cv
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
  uint16_t uiYCoordinate; //to store and send the XY coordinates to the board
  bool fCoordinateValid; 
} tCoordinateHandler;

typedef union {
  struct {
    uint8_t bStx;  // supposed be CHAR_STX
    uint8_t bMsgType;
    uint8_t abPayload[DATA_PACKAGE_PAYLOAD_SIZE];
    uint8_t bEtx;  // supposed to be CHAR_ETX
  }; //ab as in array of bytes 
  uint8_t abData[DATA_PACKAGE_SIZE]; //defining the buffer in which you piece together
  //the parts of the message
} tMsgBuffer;
/********* typedefs end *********/

/********* function declaration start *********/
bool MsgReader_Heartbeat(bool *pfRxMsgComplete);
bool MsgHandler_Heartbeat(bool *pfRxMsgComplete, bool *pfStartCvSignal, tCoordinateHandler *pCoordinateHandler);
bool IsTimeout(uint32_t ulNewTime, uint32_t ulOldTime, uint32_t ulTimeout);
/********* function declaration end *********/

/********* module variable definitions start *********/
// Serial is USB interface with PC; used to flash Arduino and emulate communication between CV (mocked by PC) and control (mocked by arduino)
// cvSerial is UART interface with another arduino acting as scanner (check "Second Arduino.ino"); used to echo Rx data to scanner
const byte virtualRxPin = 6;
const byte virtualTxPin = 7; //defines which pins on the board you are sending these messages through
SoftwareSerial cvSerial(virtualRxPin, virtualTxPin);

tMsgBuffer rxBuffer;
tMsgBuffer txBuffer;
/********* module variable definitions end *********/

void setup() {
  Serial.begin(9600);
  cvSerial.begin(9600);
  txBuffer.bStx = CHAR_STX;
  txBuffer.bEtx = CHAR_ETX;
  Serial.print("Setup Complete.");
}

void loop() {
  static bool fRxMsgComplete = false;
  MsgReader_Heartbeat(&fRxMsgComplete); //call the message reader

  static bool fStartCvSignal = true;           // mock user signal that enables auto-aim mode
  tCoordinateHandler CoordinateHandler;        // will be used by gimbal
  CoordinateHandler.fCoordinateValid = false;  // pretend coordinate is used up by gimbal elsewhere before each loop
  MsgReader_Heartbeat(&fRxMsgComplete); 
  Serial.print("Heartbeat: \n\t");
  delay(1000); //1 second heartbeat delay
}

bool MsgReader_Heartbeat(bool *pfRxMsgComplete) {
  // restart reading only after Rx Msg is processed, indicated by (*pfRxMsgComplete == false)
  if (*pfRxMsgComplete == false) {
    // read from STX to ETX
    static bool fStxReceived = false; //defined as static so this still exists even when out of scope
    //if you recieved the stx OR ( (there's at least one byte in the rx buffer) AND (the stx value in the rx buffer is what you expect) )
    if (fStxReceived || ((cvSerial.readBytes(rxBuffer.abData, 1) == 1) && (rxBuffer.bStx == CHAR_STX))) {
      fStxReceived = false; 
      if (cvSerial.readBytes(&rxBuffer.abData[1], DATA_PACKAGE_SIZE - 1) == DATA_PACKAGE_SIZE - 1) { //if you successfully read the next few bytes
        if (rxBuffer.bEtx == CHAR_ETX) { //and the ETX is correct 
          *pfRxMsgComplete = true; //rx message is complete
          // echo to scanner
          Serial.println("sending rx data to board");
          Serial.write(rxBuffer.abData, DATA_PACKAGE_SIZE); //write the data package to the serial port - send it to the CV 
        } else {
          // unsynched
          for (uint8_t bDataIndex = 1; bDataIndex < DATA_PACKAGE_SIZE; bDataIndex++) { //keep iterating until you resynch
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

bool MsgHandler_Heartbeat(bool *pfRxMsgComplete, bool *pfStartCvSignal, tCoordinateHandler *pCoordinateHandler) {
  typedef enum {
    HANDLER_IDLE,
    HANDLER_ENABLE_CV,
    HANDLER_WAIT_FOR_COORDINATE,
    HANDLER_DISABLE_CV,
    HANDLER_WAIT_FOR_ACK,
  } tHandlerState; //defining handler states
  static tHandlerState HandlerState = HANDLER_IDLE; //static variables to represent what state the handler is in
  static tHandlerState NextHandlerState = HANDLER_IDLE;
  static uint32_t ulOldRxTimestamp = 0; 
  uint32_t ulNewRxTimestamp = millis(); //current time since starting the program
  bool fCoordiateReady = false;

  Serial.println("Handler State: " + HandlerState);
  switch (HandlerState) {
    case HANDLER_IDLE:
      {
        if (*pfStartCvSignal) { //if you got the go signal 
          HandlerState = HANDLER_ENABLE_CV; //go to the state that enables the computer vision
        }
        break;
      }
    case HANDLER_ENABLE_CV:
      {
        if ((*pfStartCvSignal == false)  // global user interrupt OR if its timed out 
            || IsTimeout(ulNewRxTimestamp, ulOldRxTimestamp, TRANSMISSION_TIMEOUT)) {
          HandlerState = HANDLER_DISABLE_CV; //disable
        } else { //if valid 
          txBuffer.bMsgType = MSG_MODE_CONTROL; //add msg to the tx bugger - 0x10
          memset(txBuffer.abPayload, CHAR_UNUSED, DATA_PACKAGE_PAYLOAD_SIZE); //set the next few spaces to the empty 0xFF
          txBuffer.abPayload[0] = MODE_TURN_ON_AUTO_AIM;//turn on auto aim mode 0x01
          cvSerial.write(txBuffer.abData, DATA_PACKAGE_SIZE);//write it to the serial - the ETX and STX have been set up in setup so dont panic 
          HandlerState = HANDLER_WAIT_FOR_ACK; //waiting for acknowledge 
          NextHandlerState = HANDLER_WAIT_FOR_COORDINATE;
        }
        break;
      }
    case HANDLER_WAIT_FOR_COORDINATE:
      {
        if ((*pfStartCvSignal == false)  // global user interrupt
            || IsTimeout(ulNewRxTimestamp, ulOldRxTimestamp, TRANSMISSION_TIMEOUT)) {
          HandlerState = HANDLER_DISABLE_CV;
        } else if (*pfRxMsgComplete) { //if valid and the RX message was sent
          *pfRxMsgComplete = false;
          ulOldRxTimestamp = ulNewRxTimestamp;
          if (rxBuffer.bMsgType == MSG_COORDINATE) { //if the type is that which represents an enemy corrdinate 
            memcpy(pCoordinateHandler, rxBuffer.abPayload, DATA_PACKAGE_PAYLOAD_SIZE); //copy the data from the rx buffer to the coordinate handler which you now created
            pCoordinateHandler->fCoordinateValid = true; //set the coordinate valid boolean to true 
            Serial.println("Coordinate recieved.");
            // HandlerState = ; // stay in this state until user interrupt or transmission timeout
          }
        }
        break;
      }
    case HANDLER_DISABLE_CV:
      {
        txBuffer.bMsgType = MSG_MODE_CONTROL;//add msg to the tx bugger - 0x10
        memset(txBuffer.abPayload, CHAR_UNUSED, DATA_PACKAGE_PAYLOAD_SIZE); //set the next few spaces to the empty 0xFF
        txBuffer.abPayload[0] = MODE_TURN_OFF; //mode is now set to turn off 0x00
        cvSerial.write(txBuffer.abData, DATA_PACKAGE_SIZE); //send the message 
        Serial.println("Disable CV command sent.");
        // turn off auto-aim mode; wait for user input to restart again
        *pfStartCvSignal = false;
        HandlerState = HANDLER_IDLE; 
        break;
      }
    case HANDLER_WAIT_FOR_ACK:
      {
        if ((*pfStartCvSignal == false)  // global user interrupt
            || IsTimeout(ulNewRxTimestamp, ulOldRxTimestamp, TRANSMISSION_TIMEOUT)) {
          HandlerState = HANDLER_DISABLE_CV;
        } else if (*pfRxMsgComplete) { //if valid 
          *pfRxMsgComplete = false;
          ulOldRxTimestamp = ulNewRxTimestamp; //update timestamp 
          if ((rxBuffer.bMsgType == MSG_ACK) //if this is an acknowledgement message
              && (memcmp(rxBuffer.abPayload, "ACK", sizeof("ACK")) == 0) //and writing ack to the payload was successfull
              && (rxBuffer.abPayload[DATA_PACKAGE_PAYLOAD_SIZE - 1] == CHAR_UNUSED)){ // and the final bit is unused 
            Serial.println("Acknowledgement Recieved."); 
            HandlerState = NextHandlerState; //go to next state, usually either idle or wait for coordinate  
          }
        }
        break;
      }
  }
  return fCoordiateReady; 
}

bool IsTimeout(uint32_t ulNewTime, uint32_t ulOldTime, uint32_t ulTimeout) {
  // works even with overflow //if the time between the new (current) time and the old time is above the time out threshold 
  return ((uint32_t)(ulNewTime - ulOldTime) > ulTimeout); 
}
