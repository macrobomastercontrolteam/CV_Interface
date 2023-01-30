//Receiver Arduino Board Code
char RxString[100];  //Initialized variable to store receive

void setup() {
  Serial.begin(9600);
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
    //    Serial.println(RxString);  //Print data on Serial Monitor
    for (int i = 0; i < bytestoread; i++) {
      Serial.print("\t0x");
      Serial.print(RxString[i], HEX);
      Serial.print('\n');
    }
  }
  delay(1000);
}
