/*******************************************************************************
 #  SPDX-License-Identifier: GPL-3.0-or-later                                  #
 #  SPDX-FileCopyrightText: 2026 Drona Aviation                                #
 #  -------------------------------------------------------------------------  #
 #  Copyright (c) 2026 Drona Aviation                                          #
 #  All rights reserved.                                                       #
 #  -------------------------------------------------------------------------  #
 #  Author: Ashish Jaiswal (MechAsh) <AJ>                                      #
 #  Project: MagisV2                                                           #
 #  File: \tools\sim\test_set_default_pid.cpp                                  #
 #  Created Date: Sat, 14th Nov 2026                                           #
 #  Brief: Host simulation of setDefault_PIDProfile ( ). Builds the real       #
 #         API-Src/FC-Config-PID.cpp and flight/pid_defaults.cpp against       #
 #         tools/sim/stub; run with tools/sim/run.sh. Not firmware code.       #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  Last Modified: Sat, 14th Nov 2026                                          #
 #  Modified By: AJ                                                            #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  HISTORY:                                                                   #
 #  Date      	By	Comments                                                   #
 #  ----------	---	---------------------------------------------------------  #
 #  14-11-2026	AJ	Created for API test T06 ( B1 ), set-default-pid-profile.  #
*******************************************************************************/

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "common/axis.h"
#include "flight/pid.h"
#include "config/config_master.h"
#include "API/FC-Config.h"

// The globals FC-Config-PID.cpp links against.
static profile_t simProfile;
profile_t *currentProfile = &simProfile;
uint32_t userLoopFrequency;

static int checks   = 0;
static int failures = 0;

static void expectEq ( const char *what, unsigned got, unsigned want ) {
  checks++;
  if ( got != want ) {
    failures++;
    printf ( "  FAIL %-34s got %3u, want %3u\n", what, got, want );
  }
}

static void expectPid ( const char *what, unsigned gp, unsigned gi, unsigned gd, unsigned wp, unsigned wi, unsigned wd ) {
  checks++;
  if ( gp != wp || gi != wi || gd != wd ) {
    failures++;
    printf ( "  FAIL %-34s got %u / %u / %u, want %u / %u / %u\n", what, gp, gi, gd, wp, wi, wd );
  }
}

// The five profiles the API exposes and the firmware defaults T06 expects ( P / I / D ).
typedef struct {
  pid_profile_e api;
  pidIndex_e slot;
  const char *name;
  uint8_t p, i, d;
} apiSlot_t;

static const apiSlot_t apiSlots [] = {
  { PID_ROLL,  PIDROLL,  "PID_ROLL",  40,  10, 30 },
  { PID_PITCH, PIDPITCH, "PID_PITCH", 40,  10, 30 },
  { PID_YAW,   PIDYAW,   "PID_YAW",   150, 70, 50 },
  { PID_ALT,   PIDALT,   "PID_ALT",   128, 0,  30 },
  { PID_USER,  PIDUSER,  "PID_USER",  60,  30, 5 },
};

// The internal slots setDefault_PIDProfile ( ) must never write.
static const struct {
  pidIndex_e slot;
  const char *name;
} hiddenSlots [] = {
  { PIDPOS, "PIDPOS" }, { PIDPOSR, "PIDPOSR" }, { PIDNAVR, "PIDNAVR" },
  { PIDLEVEL, "PIDLEVEL" }, { PIDMAG, "PIDMAG" }, { PIDVEL, "PIDVEL" },
};

#define COUNT( a ) ( sizeof ( a ) / sizeof ( ( a ) [ 0 ] ) )

static void checkApiSlots ( const char *tag ) {
  char what [ 64 ];
  for ( size_t n = 0; n < COUNT ( apiSlots ); n++ ) {
    const apiSlot_t *e     = &apiSlots [ n ];
    const pidProfile_t *pp = &currentProfile->pidProfile;
    PID pid;

    snprintf ( what, sizeof ( what ), "%s %s P8", tag, e->name );
    expectEq ( what, pp->P8 [ e->slot ], e->p );
    snprintf ( what, sizeof ( what ), "%s %s I8", tag, e->name );
    expectEq ( what, pp->I8 [ e->slot ], e->i );
    snprintf ( what, sizeof ( what ), "%s %s D8", tag, e->name );
    expectEq ( what, pp->D8 [ e->slot ], e->d );

    // The read-back T06 does: through the public getter.
    get_PIDProfile ( e->api, &pid );
    snprintf ( what, sizeof ( what ), "%s get_PIDProfile ( %s )", tag, e->name );
    expectPid ( what, pid.p, pid.i, pid.d, e->p, e->i, e->d );
  }
}

// Marks the whole profile, restores the defaults, and checks that only the five
// API slots changed: every other byte of the profile must still hold the marker.
static void testRestoresOnlyApiSlots ( uint8_t marker ) {
  char tag [ 16 ];
  snprintf ( tag, sizeof ( tag ), "[0x%02X]", marker );
  printf ( "test: restores the five API slots only, profile marked %s\n", tag );

  memset ( &simProfile, marker, sizeof ( simProfile ) );
  profile_t before;
  memcpy ( &before, &simProfile, sizeof ( before ) );    // memcpy, not a struct copy: padding bytes must carry over too

  setDefault_PIDProfile ( );

  checkApiSlots ( tag );

  char what [ 64 ];
  for ( size_t n = 0; n < COUNT ( hiddenSlots ); n++ ) {
    const pidIndex_e s = hiddenSlots [ n ].slot;
    snprintf ( what, sizeof ( what ), "%s %s P8 untouched", tag, hiddenSlots [ n ].name );
    expectEq ( what, currentProfile->pidProfile.P8 [ s ], marker );
    snprintf ( what, sizeof ( what ), "%s %s I8 untouched", tag, hiddenSlots [ n ].name );
    expectEq ( what, currentProfile->pidProfile.I8 [ s ], marker );
    snprintf ( what, sizeof ( what ), "%s %s D8 untouched", tag, hiddenSlots [ n ].name );
    expectEq ( what, currentProfile->pidProfile.D8 [ s ], marker );
  }

  // Put the five API slots back to the marker: the rest of the profile
  // ( controller, float gains, cut frequencies, ... ) must be byte-identical.
  profile_t after;
  memcpy ( &after, &simProfile, sizeof ( after ) );
  for ( size_t n = 0; n < COUNT ( apiSlots ); n++ ) {
    const pidIndex_e s = apiSlots [ n ].slot;
    after.pidProfile.P8 [ s ] = before.pidProfile.P8 [ s ];
    after.pidProfile.I8 [ s ] = before.pidProfile.I8 [ s ];
    after.pidProfile.D8 [ s ] = before.pidProfile.D8 [ s ];
  }
  snprintf ( what, sizeof ( what ), "%s rest of profile unchanged", tag );
  expectEq ( what, memcmp ( &after, &before, sizeof ( before ) ) == 0, 1 );
}

// The restored values are the firmware defaults, not a second copy of them.
static void testMatchesFirmwareDefaults ( void ) {
  printf ( "test: API slots equal pidProfileDefaultGains ( )\n" );
  pidProfile_t ref;
  memset ( &ref, 0, sizeof ( ref ) );
  pidProfileDefaultGains ( &ref );

  memset ( &simProfile, 0xAA, sizeof ( simProfile ) );
  setDefault_PIDProfile ( );

  char what [ 64 ];
  for ( size_t n = 0; n < COUNT ( apiSlots ); n++ ) {
    const pidIndex_e s = apiSlots [ n ].slot;
    snprintf ( what, sizeof ( what ), "%s == firmware default", apiSlots [ n ].name );
    expectPid ( what, currentProfile->pidProfile.P8 [ s ], currentProfile->pidProfile.I8 [ s ], currentProfile->pidProfile.D8 [ s ],
                ref.P8 [ s ], ref.I8 [ s ], ref.D8 [ s ] );
  }
}

// T06 as a user would run it: tune through the API, then restore.
static void testAfterUserTuning ( void ) {
  printf ( "test: set_PIDProfile ( ) then setDefault_PIDProfile ( )\n" );
  memset ( &simProfile, 0, sizeof ( simProfile ) );
  for ( size_t n = 0; n < COUNT ( apiSlots ); n++ ) {
    PID tuned = { 1, 2, 3 };
    set_PIDProfile ( apiSlots [ n ].api, &tuned );
  }
  setDefault_PIDProfile ( );
  setDefault_PIDProfile ( );    // twice: idempotent
  checkApiSlots ( "[tuned]" );
}

int main ( void ) {
  testRestoresOnlyApiSlots ( 0xAA );
  testRestoresOnlyApiSlots ( 0x55 );
  testMatchesFirmwareDefaults ( );
  testAfterUserTuning ( );

  printf ( "%s: %d checks, %d failed\n", failures ? "FAIL" : "PASS", checks, failures );
  return failures ? 1 : 0;
}
