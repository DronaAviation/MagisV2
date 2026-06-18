// Do not remove the include below
#include "PlutoPilot.h"

/* ─────────────────────────────────────────────────────────────────────────
 *  Simple WS2812B glow — the whole 8-LED strip lights up in one steady
 *  (constant) color. Set once; nothing to update each loop.
 *
 *  Change the color by editing the (r,g,b) in RGB_SetColorAll below.
 *  Change overall intensity with RGB_SetBrightness.
 * ───────────────────────────────────────────────────────────────────────*/



/**
 * Configures Pluto's receiver to use PPM or default ESP mode; activate the line matching your setup.
 * AUX channel configurations is only for PPM recievers if no custom configureMode function is called this are the default setup
 * ARM mode : Rx_AUX2, range 1300 to 2100
 * ANGLE mode : Rx_AUX2, range 900 to 2100
 * BARO mode : Rx_AUX3, range 1300 to 2100
 * MAG mode : Rx_AUX1, range 900 to 1300
 * HEADFREE mode : Rx_AUX1, range 1300 to 1700
 * DEV mode : Rx_AUX4, range 1500 to 2100
 */
void plutoRxConfig ( void ) {
  // Receiver mode: Uncomment one line for ESP or CAM or PPM setup.
  Receiver_Mode ( Rx_ESP );    // Onboard ESP
  // Receiver_Mode ( Rx_CAM );    // WiFi CAMERA
  // Receiver_Mode ( Rx_PPM );    // PPM based
}

// The setup function is called once at Pluto's hardware startup
void plutoInit ( void ) {
  // Add your hardware initialization code here
}

// The function is called once before plutoLoop when you activate Developer Mode
void onLoopStart ( void ) {
  RGB_Init ( 8 );                  // take control of the 8-LED strip
  RGB_SetBrightness ( 80 );        // 80% brightness
  RGB_SetColorAll ( 0, 180, 255 ); // steady glow color (R,G,B) — soft cyan
  RGB_Show ( );                    // push to the strip
}

// The loop function is called in an endless loop
void plutoLoop ( void ) {
  // Steady glow — color was set once in onLoopStart, nothing to update here.
}

// The function is called once after plutoLoop when you deactivate Developer Mode
void onLoopFinish ( void ) {
  RGB_Release ( );                 // hand the strip back to the system
}
