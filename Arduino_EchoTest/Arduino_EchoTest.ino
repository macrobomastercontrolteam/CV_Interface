// Serial is USB interface with RPi; used to flash Arduino and view real-time output
// mySerial is UART interface with RPi; used to emulate CV communication

#include <SoftwareSerial.h>
const byte virtualRxPin = 6;
const byte virtualTxPin = 7;
SoftwareSerial mySerial (virtualRxPin, virtualTxPin);

char RxString[100];  // Rx Buffer

void setup() {
  Serial.begin(9600);
  mySerial.begin(9600);
  while (!Serial) {
    ;  // wait for serial port to connect. Needed for Native USB only
  }
}
void loop() {
  Serial.print("RECEIVER: ");
  int bytestoread = mySerial.available();
  if (bytestoread != 0) 
  {
    // echo on both interface
    mySerial.readBytes(RxString, bytestoread);
    mySerial.print(RxString);    
    Serial.println(RxString);
  } 
  else {
    Serial.println();
  }
  delay(1000);
}


