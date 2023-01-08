#include <Wire.h>
#include "maths.hpp"

#define SLAVE_ADDRES  40
#define EN    11

OutLine out_line;

// i2c
void request_event();

void setup() {
  Wire.begin(SLAVE_ADDRES);
  Wire.onRequest(request_event);

  // Enable lighting
  pinMode(EN, OUTPUT);
  digitalWrite(EN, HIGH);

  Serial.begin(115200);

  while(!Serial);
  out_line.begin();
}


void loop() {
  delay(10);
}

void request_event() {
  float direction_line, chord;
  bool st = out_line.process_line(direction_line, chord);

  byte write_data[3] = { 'N', 'N', 'N' };
  
  if (!st) {
    Wire.write(write_data, 3);
    return;
  }

  direction_line = angles::normalize_angle(direction_line + M_PI);

  int16_t a = int(direction_line * 1000);
  byte byte_low = a & 0xFF,
      byte_high = (a >> 8) & 0xFF;

  write_data[0] = 'A';
  write_data[1] = byte_low;
  write_data[2] = byte_high;
  Wire.write(write_data, 3);
}
