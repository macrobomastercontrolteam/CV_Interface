// Serial is USB interface with RPi; used to flash Arduino and view real-time output
// cvSerial is UART interface with RPi; used to emulate CV communication

#include <SoftwareSerial.h>
const byte virtualRxPin = 6;
const byte virtualTxPin = 7;
SoftwareSerial cvSerial(virtualRxPin, virtualTxPin);

char RxString[100];  // Rx Buffer

void setup() {
  Serial.begin(9600);
  cvSerial.begin(9600);
  while (!Serial) {
    ;  // wait for serial port to connect. Needed for Native USB only
  }
}
void loop() {
  int bytestoread = cvSerial.available();
  if (bytestoread != 0) {
    // echo on both interface
    cvSerial.readBytes(RxString, bytestoread);
    cvSerial.print(RxString);
    Serial.print("RECEIVER: \n\t");
    Serial.println(RxString);
  }
  delay(1000);
}
