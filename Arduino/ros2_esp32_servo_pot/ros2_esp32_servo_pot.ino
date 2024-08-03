// ros2_esp32_servo_pot.ino  weigu.lu
// Using ESP32Servo (install with lib manager) 
// https://github.com/jkb-git/ESP32Servo

#include <ESP32Servo.h>

const long SER_SPEED = 1000000;
const byte PIN_POT = 26;
const byte PIN_SERVO = 18;
const unsigned short PREAMBLE = 0xABCD;
const unsigned long DELAY_MS = 500; // time in ms between sends
unsigned short pot_value = 0;       // 0-4095 variable to store pot position
int servo_angle = 0;                // 0-180° variable to store the servo position
byte checksum = 0;
byte data_out[5] = {0};  // consists for the moment only of pot_value 
byte data_out_length = sizeof(data_out) / sizeof(byte);
byte data_in[5] = {0};  // consists for the moment only of servo_angle 
byte data_in_length = sizeof(data_in) / sizeof(byte);

Servo My_Servo;

void setup() {
  Serial.begin(115200);                        // initialize serial monitor
  Serial2.begin(SER_SPEED,SERIAL_8N1, 16, 17); //init hardware serial  
  My_Servo.setPeriodHertz(50);                 // see lib example
  My_Servo.attach(PIN_SERVO, 1000, 2000);      // see lib example
  delay(500);
  Serial.println("Setup done!");
}

void loop() {
  if (non_blocking_delay(DELAY_MS)) {
    send_pot_value();    
  }
  if (read_telegram() == data_in_length) {
    set_servo();
    
  }
  delay(1); // for the dog
}

unsigned short get_pot_value() {  
  const byte ADC_SAMPLES = 5;
  unsigned long pot_adc_sum = 0;    
  for (byte i = 0; i<ADC_SAMPLES; i++) {
    pot_adc_sum += analogRead(PIN_POT);
  }
  return pot_adc_sum/ADC_SAMPLES;
}

void send_pot_value() {
  data_out[0] = highByte(PREAMBLE);  
  data_out[1] = lowByte(PREAMBLE);    
  pot_value = get_pot_value();
  Serial.println(pot_value);
  data_out[2] = highByte(pot_value);
  data_out[3] = lowByte(pot_value);
  data_out[4] = xor_checksum(data_out,2,3);
  for (byte i = 0; i < data_out_length; i++) {
    Serial2.write(data_out[i]);    
  }  
  for (byte i = 0; i < data_out_length; i++) {
    Serial.print(data_out[i],HEX);
  }  
  Serial.println();
}

byte read_telegram() {
  unsigned short serial_cnt = 0;  
  if (Serial2.available()) {   // wait for the whole stream
    delay(1); // 1ms = 100 byte with 1000000bps
    }  
  while (Serial2.available()) {
    data_in[serial_cnt] = Serial2.read();
    serial_cnt++;
  }
  if ((data_in[0] != highByte(PREAMBLE)) || (data_in[1] != lowByte(PREAMBLE))) {    
    while (Serial2.available() > 0) {Serial2.read();} //clear the buffer!    
    return -1;
  }
  return serial_cnt;
}

void set_servo() {
byte checksum = xor_checksum(data_in,2,3);
  if (data_in[4] == checksum) {
    servo_angle = data_in[2]*256+data_in[3];
    Serial.println(servo_angle);           
    My_Servo.write(servo_angle);        
  }
}

byte xor_checksum(byte *data, byte index1, byte index2) {
  checksum = 0;
  for (byte i = index1; i <= index2; i++) {
    checksum ^= data[i]; // XOR the current byte with the checksum
  }
  return checksum; 
}

bool non_blocking_delay(unsigned long milliseconds) {
  static unsigned long nb_delay_prev_time = 0;
  if(millis() >= nb_delay_prev_time + milliseconds) {
    nb_delay_prev_time += milliseconds;
    return true;
  }
  return false;
} 
