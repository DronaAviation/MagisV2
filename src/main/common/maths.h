/*******************************************************************************
 #  SPDX-License-Identifier: GPL-3.0-or-later                                  #
 #  SPDX-FileCopyrightText: 2025 Cleanflight & Drona Aviation                  #
 #  -------------------------------------------------------------------------  #
 #  Copyright (c) 2025 Drona Aviation                                          #
 #  All rights reserved.                                                       #
 #  -------------------------------------------------------------------------  #
 #  Author: Ashish Jaiswal (MechAsh) <AJ>                                      #
 #  Project: MagisV2                                                           #
 #  File: \src\main\common\maths.h                                             #
 #  Created Date: Fri, 7th Nov 2025                                            #
 #  Brief:                                                                     #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  Last Modified: Wed, 31st Dec 2025                                          #
 #  Modified By: AJ                                                            #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  HISTORY:                                                                   #
 #  Date      	By	Comments                                                   #
 #  ----------	---	---------------------------------------------------------  #
*******************************************************************************/

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#ifndef sq
  #define sq( x ) ( ( x ) * ( x ) )
#endif

// Undefine this for use libc sinf/cosf. Keep this defined to use fast sin/cos approximations
#define FAST_TRIGONOMETRY    // order 9 approximation
// #define EVEN_FASTER_TRIGONOMETRY      // order 7 approximation

// Use floating point M_PI instead explicitly.
#define M_PIf       3.14159265358979323846f

#define RAD         ( M_PIf / 180.0f )

#define MIN( a, b ) ( ( a ) < ( b ) ? ( a ) : ( b ) )
#define MAX( a, b ) ( ( a ) > ( b ) ? ( a ) : ( b ) )
#define ABS( x )    ( ( x ) > 0 ? ( x ) : -( x ) )

// acceleration due to gravity in m/s/s
#define GRAVITY_CSS 4096.0f
#define GRAVITY_MSS 9.80665f

// Single precision conversions
#define DEG_TO_RAD                       0.017453292519943295769236907684886f
#define RAD_TO_DEG                       57.295779513082320876798154814105f

#define FLT_EPSILON                      1.19209290E-07F    // decimal constant

#define bitRead( value, bit )            ( ( ( value ) >> ( bit ) ) & 0x01 )
#define bitSet( value, bit )             ( ( value ) |= ( 1UL << ( bit ) ) )
#define bitClear( value, bit )           ( ( value ) &= ~( 1UL << ( bit ) ) )
#define bitWrite( value, bit, bitvalue ) ( bitvalue ? bitSet ( value, bit ) : bitClear ( value, bit ) )

typedef struct stdev_s {
  float m_oldM, m_newM, m_oldS, m_newS;
  int m_n;
} stdev_t;

// Floating point 3 vector.
typedef struct fp_vector {
  float X;
  float Y;
  float Z;
} t_fp_vector_def;

typedef union {
  float A [ 3 ];
  t_fp_vector_def V;
} t_fp_vector;

// Floating point Euler angles.
// Be carefull, could be either of degrees or radians.
typedef struct fp_angles {
  float roll;
  float pitch;
  float yaw;
} fp_angles_def;

typedef union {
  float raw [ 3 ];
  fp_angles_def angles;
} fp_angles_t;

int32_t applyDeadband ( int32_t value, int32_t deadband );

int constrain ( int amt, int low, int high );
float constrainf ( float amt, float low, float high );

void devClear ( stdev_t *dev );
void devPush ( stdev_t *dev, float x );
float devVariance ( stdev_t *dev );
float devStandardDeviation ( stdev_t *dev );
float degreesToRadians ( int16_t degrees );

int scaleRange ( int x, int srcMin, int srcMax, int destMin, int destMax );

void normalizeV ( struct fp_vector *src, struct fp_vector *dest );

void rotateV ( struct fp_vector *v, fp_angles_t *delta );
void buildRotationMatrix ( fp_angles_t *delta, float matrix [ 3 ][ 3 ] );

int32_t quickMedianFilter3 ( int32_t *v );
int32_t quickMedianFilter5 ( int32_t *v );
int32_t quickMedianFilter7 ( int32_t *v );
int32_t quickMedianFilter9 ( int32_t *v );

#if defined( FAST_TRIGONOMETRY ) || defined( EVEN_FASTER_TRIGONOMETRY )
float sin_approx ( float x );
float cos_approx ( float x );
#else
  #define sin_approx( x ) sinf ( x )
  #define cos_approx( x ) cosf ( x )
#endif

void arraySubInt32 ( int32_t *dest, int32_t *array1, int32_t *array2, int count );

uint16_t leastSignificantBit ( uint16_t myInt );

/* used mostly for dcm  */
float safe_asin ( float v );

// degrees -> radians
float radians ( float deg );

// radians -> degrees
float degrees ( float rad );

bool is_positive ( float val );

int32_t map_i32 ( int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max );

typedef struct {
  uint16_t *buf;     // storage provided by caller
  uint32_t sum;      // use 32-bit to avoid overflow
  uint16_t size;     // buffer length
  uint16_t head;     // next write index
  uint16_t count;    // valid samples (<= size)
} ring_avg_u16_t;

void ring_avg_u16_init ( ring_avg_u16_t *r, uint16_t *storage, uint16_t size );
uint16_t ring_avg_u16_get ( ring_avg_u16_t *r, uint16_t sample );
void ring_avg_u16_reset ( ring_avg_u16_t *r );

#ifdef __cplusplus
}
#endif
