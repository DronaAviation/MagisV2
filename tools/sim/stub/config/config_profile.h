// Host-simulation stub ( tools/sim ): shadows the firmware header of the same path.
#pragma once

#include "flight/pid.h"    // the real pidProfile_t

// Only the member the PID API touches; the real profile_t has more.
typedef struct profile_s {
  pidProfile_t pidProfile;
} profile_t;
