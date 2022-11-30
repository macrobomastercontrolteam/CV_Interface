uint8_t TxString[15] = "LinuxHint.com\n";  //String data which is to be sent
char RxString[20];  //Initialized variable to store receive

void setup() {
  Serial.begin(9600);
}

void loop() {
  Serial.print("RECEIVER: ");
  int bytestoread = Serial.available();
  Serial.readBytes(RxString, bytestoread);
  if (bytestoread != 0)  //Read the serial data
  {
    Serial.println(RxString);  //Print data on Serial Monitor
  }
  else
  {
    Serial.println();
  }
  Serial.write(TxString, 15);  //Write the serial data
  delay(1000);
}
