// esp32_hardware_serial_test.ino

byte incoming_byte;
long counter = 0;

void setup() {
  Serial.begin(115200);                     // initialize serial monitor
  Serial2.begin(115200,SERIAL_8N1, 16, 17); //init hardware serial
  delay(500);
  Serial.println("Setup done!");
}

void loop() {
  Serial.println(String("Hello! ") + counter); // msg to Raspi
  Serial2.println(String("Hello! ") + counter); // msg to Raspi
  while (Serial2.available() > 0) {  // if any serial available, read it
    incoming_byte = Serial2.read();  // and write it to the terminal
    Serial.write(incoming_byte);     // alt: Serial.write(Serial2.read());
  }
  Serial.print(".");
  delay(2000);
  counter++;
}
