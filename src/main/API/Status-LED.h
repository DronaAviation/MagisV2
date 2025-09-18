#ifndef STATUS_LED_H
#define STATUS_LED_H

typedef enum {
  GREEN,
  BLUE,
  RED,
  STATUS
} led_status_e;

typedef enum {
  OFF = 0,
  ON,
  TOGGLE
} led_state_e;

extern bool LedStatusState;

void Set_LED ( led_status_e _led_status, led_state_e _state );

#endif