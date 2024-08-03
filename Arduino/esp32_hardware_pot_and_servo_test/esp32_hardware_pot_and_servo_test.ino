// esp32_hardware_pot_and_servo_test.ino

#include <ESP32Servo.h>

const byte PIN_POT = 35;
const byte PIN_SERVO = 21;
unsigned short pot_value = 0;
int servo_pos = 0;    // 0-180° variable to store the servo position

Servo My_Servo;

void setup() {
  Serial.begin(115200);              // initialize serial for terminal  
  My_Servo.setPeriodHertz(50);            // see lib example
  My_Servo.attach(PIN_SERVO, 1000, 2000); // see lib example
  delay(500);
  Serial.println("Setup done!");
}
 
void loop() {
  pot_value = analogRead(PIN_POT);    
  Serial.println(String("Pot value: ") + pot_value);
  servo_pos = map(pot_value,0,4095,0,180);
  Serial.println(String("Servo position: ") + servo_pos);
  Serial.println(servo_pos);  
  My_Servo.write(servo_pos);  
  delay(500);
}