#pragma once
#include <EEPROM.h>

namespace angles {
  static inline float mod(float x, float y) {
    x = fmod(x, y);
    return (x < 0 ? x + y : x);
  }
  
  static inline float normalize_angle(float angle) {
    const float result = fmod(angle + M_PI, 2.0 * M_PI);
    if(result <= 0.0) return result + M_PI;
    return result - M_PI;
  }

  static inline float normalize_angle_positive(float angle) {
    return mod(angle, 2.0 * M_PI);
  }

  static inline float nonreflex_angle(float x, float y) {
    float a = normalize_angle_positive(y - x);
    return fmin(a, M_PI * 2 - a);
  }

  static inline float bisector_angle(float x, float y) {
    return normalize_angle_positive(x + normalize_angle_positive(y - x) / 2.0);
  }
}

class OutLine {
public:
  void begin() {
    for(byte i = 0; i < 4; ++i) {
      pinMode(mux_pins[i], OUTPUT);
      digitalWrite(mux_pins[i], LOW);
    }

    Serial.println("Current calibration values:");
    // Read from EEPROM cal data
    bool cal_exist = false;
    EEPROM.get(0, cal_exist);
    for(byte i = 0; i < c_sensors_cnt; ++i) {
      //if (cal_exist) EEPROM.get((1 + i * 2), line_thr[i]);
      //else line_thr[i] = 300;
      line_thr[i] = 430;
      Serial.print(line_thr[i]);
      Serial.print(" ");
    }
    Serial.println("");
    
    step_sensors = (2.0f * M_PI) / float(c_sensors_cnt);
    
    for(byte i = 0; i < c_sensors_cnt; ++i) {
      float alpha = step_sensors * (float(i) - 1.5);
      sensor_points[i][0] = cos(alpha);
      sensor_points[i][1] = sin(alpha);
    }
  }

  float light_alpha(const byte& idx) {
    float alpha = step_sensors * (float(idx) - 1.5);
    return angles::normalize_angle_positive(alpha);
  }

  bool find_bisector(float& alpha, float& chord) {
    static const byte mux_codes[c_sensors_cnt][4] = {
      {0, 1, 1, 0}, // 12
      {1, 1, 1, 0}, // 11
      {0, 0, 0, 1}, // 10
      {1, 0, 0, 1}, // 9
      {0, 1, 0, 1}, // 8
      {1, 1, 0, 1}, // 7
      {0, 0, 1, 1}, // 6
      {1, 0, 1, 1}, // 5
      {0, 1, 1, 1}, // 4
      {1, 1, 1, 1}, // 3
      {0, 0, 0, 0}, // 2
      {1, 0, 0, 0}, // 1
      {0, 1, 0, 0}, // 16
      {1, 1, 0, 0}, // 15
      {0, 0, 1, 0}, // 14
      {1, 0, 1, 0}, // 13
    };

    byte ligh_detect[c_sensors_cnt] = {0};
    byte sz = 0;

    for(byte i = 0; i < c_sensors_cnt; ++i) {

      for(byte j = 0; j < 4; ++j) 
        digitalWrite(mux_pins[j], mux_codes[i][j]);
      
      bool active = (analogRead(mux_singnal) < line_thr[i] ? true : false);

      if(active)
        ligh_detect[sz++] = i;
    }

    alpha = 0;
    chord = 0;

    prev_online = online;

    if(sz == 0) {
      online = false;
      return false;
    }else if(sz == 1) {
      alpha = light_alpha(ligh_detect[0]);
      chord = 0;
    }else {
      float sector_start, sector_end;
      
      for(byte i = 0; i < sz; ++i)
        for(byte j = 0; j < sz; ++j) {
          if(i == j) continue;
          
          float current_alpha = angles::nonreflex_angle(light_alpha(ligh_detect[i]), 
              light_alpha(ligh_detect[j]));
          if(current_alpha > alpha) {
            alpha = current_alpha;
            sector_start = light_alpha(ligh_detect[i]);
            sector_end = light_alpha(ligh_detect[j]);
          }
        }
      alpha = (angles::normalize_angle_positive(sector_end - sector_start) <= M_PI ? 
        angles::bisector_angle(sector_start, sector_end) : angles::bisector_angle(sector_end, sector_start));
      chord = angles::nonreflex_angle(sector_start, sector_end) / M_PI;
    }

    online = true;
    return true;
  }

  bool process_line(float& alpha, float& chord) {
    bool st = find_bisector(alpha, chord);

    if(prev_online && online) {
      if(angles::nonreflex_angle(prev_alpha, alpha) >= (M_PI / 2)) {
        alpha = angles::normalize_angle(alpha + M_PI);
        chord = 2 - chord;
      }
    }else if(!prev_online && online) {
      prev_alpha = alpha;
    }

    return st;
  }

private:
  // MULTIPLEXER PINS
  const byte mux_pins[4] = { 10, 9, 4, 3 };
  const byte mux_singnal = A1;
  // MULTIPLEXER PINS

  static const byte c_sensors_cnt = 16;
  float sensor_points[c_sensors_cnt][2];
  float step_sensors;
  bool prev_online = false,
    online = false;
  float prev_alpha;

  int16_t line_thr[c_sensors_cnt];

  // LINE DIRECTION
  float direction_line = 0;
  // LINE DIRECTION
};
