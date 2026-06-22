// Do not remove the include below
#include "PlutoPilot.h"

/**
 * Configures Pluto's receiver mode.
 * AUX channel configurations for ELRS:
 * ARM mode      : Rx_AUX1, range 1300 to 2100 (2-pos switch)
 * ANGLE mode    : Rx_AUX2, range 1300 to 2100 (3-pos switch: mid+high = ANGLE, low = ACRO)
 * MAG mode      : Rx_AUX3, range 1500 to 2100
 * DEV mode      : Rx_AUX4, range 1500 to 2100
 * ALT HOLD / THROTTLE mode : Rx_AUX5, range 1500 to 2100
 *                            (2-pos switch: low = THROTTLE mode, high = ALT HOLD mode)
 */
void plutoRxConfig ( void ) {
  // Receiver mode: Uncomment one line matching your setup.
  Receiver_Mode ( Rx_ESP );    // Onboard ESP
  // Receiver_Mode ( Rx_CAM );    // WiFi CAMERA
  // Receiver_Mode ( Rx_PPM );    // PPM based
  // Receiver_Mode ( Rx_ELRS );      // ExpressLRS (CRSF) on USART1
}

// The setup function is called once at Pluto's hardware startup
void plutoInit ( void ) {
  // Add your hardware initialization code here
  Oled_Init ( );
}

// The function is called once before plutoLoop when you activate Developer Mode
void onLoopStart ( void ) {
  // RGB_Init ( 8 );                  // take control of the 8-LED strip
  // RGB_SetBrightness ( 80 );        // 80% brightness
  // RGB_SetColorAll ( 0, 180, 255 ); // steady glow color (R,G,B) — soft cyan
  // RGB_Show ( );                    // push to the strip
}

// The loop function is called in an endless loop
void plutoLoop ( void ) {
  Oled_Print ( 1, 1, "HELLO" );
}

// The function is called once after plutoLoop when you deactivate Developer Mode
void onLoopFinish ( void ) {
  // RGB_Release ( );                 // hand the strip back to the system
}
