// Host-simulation stub ( tools/sim ): shadows the firmware header of the same path.
#pragma once

// The real rc_alias_e head ( io/rc_controls.h ): get / set_PIDProfile index with ROLL, PITCH, YAW.
typedef enum rc_alias { ROLL = 0, PITCH, YAW, THROTTLE } rc_alias_e;
