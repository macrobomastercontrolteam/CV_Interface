//Receiver Arduino Board Code
char RxString[100];  //Initialized variable to store receive
uint8_t RxChar;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;  // wait for serial port to connect. Needed for Native USB only
  }
}
void loop() {
  int bytestoread = Serial.available();
  Serial.readBytes(RxString, bytestoread);
  if (bytestoread != 0)  //Read the serial data
  {
    Serial.println("RECEIVER: ");
    for (int i = 0; i < bytestoread; i++) {
      // // Print by ASCII
      // Serial.print(RxString[i]);

      RxChar = RxString[i];
      Serial.print("\t0x");
      Serial.print(RxChar, HEX);
      Serial.print('\n');
    }
  }
  delay(1000);
}
