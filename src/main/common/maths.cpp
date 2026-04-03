/*******************************************************************************
 #  SPDX-License-Identifier: GPL-3.0-or-later                                  #
 #  SPDX-FileCopyrightText: 2025 Cleanflight & Drona Aviation                  #
 #  -------------------------------------------------------------------------  #
 #  Copyright (c) 2025 Drona Aviation                                          #
 #  All rights reserved.                                                       #
 #  -------------------------------------------------------------------------  #
 #  Author: Ashish Jaiswal (MechAsh) <AJ>                                      #
 #  Project: MagisV2                                                           #
 #  File: \src\main\common\maths.cpp                                           #
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

#include <stdint.h>
#include <math.h>

#include "axis.h"
#include "maths.h"

// http://lolengine.net/blog/2011/12/21/better-function-approximations
// Chebyshev http://stackoverflow.com/questions/345085/how-do-trigonometric-functions-work/345117#345117
// Thanks for ledvinap for making such accuracy possible! See: https://github.com/cleanflight/cleanflight/issues/940#issuecomment-110323384
// https://github.com/Crashpilot1000/HarakiriWebstore1/blob/master/src/mw.c#L1235
#if defined( FAST_TRIGONOMETRY ) || defined( EVEN_FASTER_TRIGONOMETRY )
  #if defined( EVEN_FASTER_TRIGONOMETRY )
    #define sinPolyCoef3 -1.666568107e-1f
    #define sinPolyCoef5 8.312366210e-3f
    #define sinPolyCoef7 -1.849218155e-4f
    #define sinPolyCoef9 0
  #else
    #define sinPolyCoef3 -1.666665710e-1f    // Double: -1.666665709650470145824129400050267289858e-1
    #define sinPolyCoef5 8.333017292e-3f     // Double:  8.333017291562218127986291618761571373087e-3
    #define sinPolyCoef7 -1.980661520e-4f    // Double: -1.980661520135080504411629636078917643846e-4
    #define sinPolyCoef9 2.600054768e-6f     // Double:  2.600054767890361277123254766503271638682e-6
  #endif

float sin_approx ( float x ) {
  int32_t xint = x;
  if ( xint < -32 || xint > 32 )
    return 0.0f;    // Stop here on error input (5 * 360 Deg)
  while ( x > M_PIf )
    x -= ( 2.0f * M_PIf );    // always wrap input angle to -PI..PI
  while ( x < -M_PIf )
    x += ( 2.0f * M_PIf );
  if ( x > ( 0.5f * M_PIf ) )
    x = ( 0.5f * M_PIf ) - ( x - ( 0.5f * M_PIf ) );    // We just pick -90..+90 Degree
  else if ( x < -( 0.5f * M_PIf ) )
    x = -( 0.5f * M_PIf ) - ( ( 0.5f * M_PIf ) + x );
  float x2 = x * x;
  return x + x * x2 * ( sinPolyCoef3 + x2 * ( sinPolyCoef5 + x2 * ( sinPolyCoef7 + x2 * sinPolyCoef9 ) ) );
}

float cos_approx ( float x ) {
  return sin_approx ( x + ( 0.5f * M_PIf ) );
}
#endif

int32_t applyDeadband ( int32_t value, int32_t deadband ) {
  if ( ABS ( value ) < deadband ) {
    value = 0;
  } else if ( value > 0 ) {
    value -= deadband;
  } else if ( value < 0 ) {
    value += deadband;
  }
  return value;
}

int constrain ( int amt, int low, int high ) {
  if ( amt < low )
    return low;
  else if ( amt > high )
    return high;
  else
    return amt;
}

float constrainf ( float amt, float low, float high ) {
  if ( amt < low )
    return low;
  else if ( amt > high )
    return high;
  else
    return amt;
}

void devClear ( stdev_t *dev ) {
  dev->m_n = 0;
}

void devPush ( stdev_t *dev, float x ) {
  dev->m_n++;
  if ( dev->m_n == 1 ) {
    dev->m_oldM = dev->m_newM = x;
    dev->m_oldS               = 0.0f;
  } else {
    dev->m_newM = dev->m_oldM + ( x - dev->m_oldM ) / dev->m_n;
    dev->m_newS = dev->m_oldS + ( x - dev->m_oldM ) * ( x - dev->m_newM );
    dev->m_oldM = dev->m_newM;
    dev->m_oldS = dev->m_newS;
  }
}

float devVariance ( stdev_t *dev ) {
  return ( ( dev->m_n > 1 ) ? dev->m_newS / ( dev->m_n - 1 ) : 0.0f );
}

float devStandardDeviation ( stdev_t *dev ) {
  return sqrtf ( devVariance ( dev ) );
}

float degreesToRadians ( int16_t degrees ) {
  return degrees * RAD;
}

int scaleRange ( int x, int srcMin, int srcMax, int destMin, int destMax ) {
  long int a = ( ( long int ) destMax - ( long int ) destMin ) * ( ( long int ) x - ( long int ) srcMin );
  long int b = ( long int ) srcMax - ( long int ) srcMin;
  return ( ( a / b ) - ( destMax - destMin ) ) + destMax;
}

// Normalize a vector
void normalizeV ( struct fp_vector *src, struct fp_vector *dest ) {
  float length;

  length = sqrtf ( src->X * src->X + src->Y * src->Y + src->Z * src->Z );
  if ( length != 0 ) {
    dest->X = src->X / length;
    dest->Y = src->Y / length;
    dest->Z = src->Z / length;
  }
}

void buildRotationMatrix ( fp_angles_t *delta, float matrix [ 3 ][ 3 ] ) {
  float cosx, sinx, cosy, siny, cosz, sinz;
  float coszcosx, sinzcosx, coszsinx, sinzsinx;

  cosx = cos_approx ( delta->angles.roll );
  sinx = sin_approx ( delta->angles.roll );
  cosy = cos_approx ( delta->angles.pitch );
  siny = sin_approx ( delta->angles.pitch );
  cosz = cos_approx ( delta->angles.yaw );
  sinz = sin_approx ( delta->angles.yaw );

  coszcosx = cosz * cosx;
  sinzcosx = sinz * cosx;
  coszsinx = sinx * cosz;
  sinzsinx = sinx * sinz;

  matrix [ 0 ][ X ] = cosz * cosy;
  matrix [ 0 ][ Y ] = -cosy * sinz;
  matrix [ 0 ][ Z ] = siny;
  matrix [ 1 ][ X ] = sinzcosx + ( coszsinx * siny );
  matrix [ 1 ][ Y ] = coszcosx - ( sinzsinx * siny );
  matrix [ 1 ][ Z ] = -sinx * cosy;
  matrix [ 2 ][ X ] = ( sinzsinx ) - ( coszcosx * siny );
  matrix [ 2 ][ Y ] = ( coszsinx ) + ( sinzcosx * siny );
  matrix [ 2 ][ Z ] = cosy * cosx;
}

// Rotate a vector *v by the euler angles defined by the 3-vector *delta.
void rotateV ( struct fp_vector *v, fp_angles_t *delta ) {
  struct fp_vector v_tmp = *v;

  float matrix [ 3 ][ 3 ];

  buildRotationMatrix ( delta, matrix );

  v->X = v_tmp.X * matrix [ 0 ][ X ] + v_tmp.Y * matrix [ 1 ][ X ] + v_tmp.Z * matrix [ 2 ][ X ];
  v->Y = v_tmp.X * matrix [ 0 ][ Y ] + v_tmp.Y * matrix [ 1 ][ Y ] + v_tmp.Z * matrix [ 2 ][ Y ];
  v->Z = v_tmp.X * matrix [ 0 ][ Z ] + v_tmp.Y * matrix [ 1 ][ Z ] + v_tmp.Z * matrix [ 2 ][ Z ];
}

// Quick median filter implementation
// (c) N. Devillard - 1998
// http://ndevilla.free.fr/median/median.pdf
#define QMF_SORT( a, b )                            \
  {                                                 \
    if ( ( a ) > ( b ) ) QMF_SWAP ( ( a ), ( b ) ); \
  }
#define QMF_SWAP( a, b )  \
  {                       \
    int32_t temp = ( a ); \
    ( a )        = ( b ); \
    ( b )        = temp;  \
  }
#define QMF_COPY( p, v, n )                      \
  {                                              \
    int32_t i;                                   \
    for ( i = 0; i < n; i++ ) p [ i ] = v [ i ]; \
  }

int32_t quickMedianFilter3 ( int32_t *v ) {
  int32_t p [ 3 ];
  QMF_COPY ( p, v, 3 );

  QMF_SORT ( p [ 0 ], p [ 1 ] );
  QMF_SORT ( p [ 1 ], p [ 2 ] );
  QMF_SORT ( p [ 0 ], p [ 1 ] );
  return p [ 1 ];
}

int32_t quickMedianFilter5 ( int32_t *v ) {
  int32_t p [ 5 ];
  QMF_COPY ( p, v, 5 );

  QMF_SORT ( p [ 0 ], p [ 1 ] );
  QMF_SORT ( p [ 3 ], p [ 4 ] );
  QMF_SORT ( p [ 0 ], p [ 3 ] );
  QMF_SORT ( p [ 1 ], p [ 4 ] );
  QMF_SORT ( p [ 1 ], p [ 2 ] );
  QMF_SORT ( p [ 2 ], p [ 3 ] );
  QMF_SORT ( p [ 1 ], p [ 2 ] );
  return p [ 2 ];
}

int32_t quickMedianFilter7 ( int32_t *v ) {
  int32_t p [ 7 ];
  QMF_COPY ( p, v, 7 );

  QMF_SORT ( p [ 0 ], p [ 5 ] );
  QMF_SORT ( p [ 0 ], p [ 3 ] );
  QMF_SORT ( p [ 1 ], p [ 6 ] );
  QMF_SORT ( p [ 2 ], p [ 4 ] );
  QMF_SORT ( p [ 0 ], p [ 1 ] );
  QMF_SORT ( p [ 3 ], p [ 5 ] );
  QMF_SORT ( p [ 2 ], p [ 6 ] );
  QMF_SORT ( p [ 2 ], p [ 3 ] );
  QMF_SORT ( p [ 3 ], p [ 6 ] );
  QMF_SORT ( p [ 4 ], p [ 5 ] );
  QMF_SORT ( p [ 1 ], p [ 4 ] );
  QMF_SORT ( p [ 1 ], p [ 3 ] );
  QMF_SORT ( p [ 3 ], p [ 4 ] );
  return p [ 3 ];
}

int32_t quickMedianFilter9 ( int32_t *v ) {
  int32_t p [ 9 ];
  QMF_COPY ( p, v, 9 );

  QMF_SORT ( p [ 1 ], p [ 2 ] );
  QMF_SORT ( p [ 4 ], p [ 5 ] );
  QMF_SORT ( p [ 7 ], p [ 8 ] );
  QMF_SORT ( p [ 0 ], p [ 1 ] );
  QMF_SORT ( p [ 3 ], p [ 4 ] );
  QMF_SORT ( p [ 6 ], p [ 7 ] );
  QMF_SORT ( p [ 1 ], p [ 2 ] );
  QMF_SORT ( p [ 4 ], p [ 5 ] );
  QMF_SORT ( p [ 7 ], p [ 8 ] );
  QMF_SORT ( p [ 0 ], p [ 3 ] );
  QMF_SORT ( p [ 5 ], p [ 8 ] );
  QMF_SORT ( p [ 4 ], p [ 7 ] );
  QMF_SORT ( p [ 3 ], p [ 6 ] );
  QMF_SORT ( p [ 1 ], p [ 4 ] );
  QMF_SORT ( p [ 2 ], p [ 5 ] );
  QMF_SORT ( p [ 4 ], p [ 7 ] );
  QMF_SORT ( p [ 4 ], p [ 2 ] );
  QMF_SORT ( p [ 6 ], p [ 4 ] );
  QMF_SORT ( p [ 4 ], p [ 2 ] );
  return p [ 4 ];
}

void arraySubInt32 ( int32_t *dest, int32_t *array1, int32_t *array2, int count ) {
  for ( int i = 0; i < count; i++ ) {
    dest [ i ] = array1 [ i ] - array2 [ i ];
  }
}
uint16_t leastSignificantBit ( uint16_t myInt ) {
  uint16_t mask = 1 << 0;
  for ( uint16_t bitIndex = 0; bitIndex <= 8; bitIndex++ ) {
    if ( ( myInt & mask ) != 0 ) {
      return bitIndex;
    }
    mask <<= 1;
  }
  return -1;
}

/* used for mostly dcm */
// a varient of asin() that checks the input ranges and ensures a
// valid angle as output. If nan is given as input then zero is
// returned.
float safe_asin ( float v ) {
  if ( isnan ( v ) ) {
    return 0.0;
  }
  if ( v >= 1.0f ) {
    return M_PIf / 2;
  }
  if ( v <= -1.0f ) {
    return -M_PIf / 2;
  }
  return asinf ( v );
}

// degrees -> radians
float radians ( float deg ) {
  return deg * DEG_TO_RAD;
}

// radians -> degrees
float degrees ( float rad ) {
  return rad * RAD_TO_DEG;
}

bool is_positive ( float val ) {
  return val >= FLT_EPSILON;
}

/**
 * @brief Maps a 32-bit integer from one range to another, clamping when necessary.
 *
 * This function takes an integer `x` and maps it from the input range
 * `[in_min, in_max]` to the output range `[out_min, out_max]`. It
 * ensures the output is clamped within the specified input range and
 * handles potential divide-by-zero errors when the input range is zero.
 *
 * @param x The value to be mapped.
 * @param in_min The lower bound of the input range.
 * @param in_max The upper bound of the input range.
 * @param out_min The lower bound of the output range.
 * @param out_max The upper bound of the output range.
 * @return The mapped integer within the output range.
 */
int32_t map_i32 ( int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max ) {
  // Check if input range is zero to avoid divide-by-zero error
  if ( in_max == in_min ) return out_min;

  // Clamp the input value x to be within the input range [in_min, in_max]
  if ( x < in_min ) x = in_min;
  if ( x > in_max ) x = in_max;

  // Calculate the numerator for mapping by scaling the difference of x and in_min
  // with the size of the output range (out_max - out_min)
  int64_t num = ( int64_t ) ( x - in_min ) * ( out_max - out_min );

  // Calculate the denominator as the size of the input range
  int64_t den = ( in_max - in_min );

  // Return the mapped value in the output range by adding the scaled value to out_min
  return ( int32_t ) ( out_min + ( num / den ) );
}

void ring_avg_u16_init ( ring_avg_u16_t *r, uint16_t *storage, uint16_t size ) {
  r->buf   = storage;
  r->sum   = 0;
  r->size  = size;
  r->head  = 0;
  r->count = 0;

  // optional: clear storage
  for ( uint16_t i = 0; i < size; i++ ) storage [ i ] = 0;
}

static inline void ring_avg_u16_push ( ring_avg_u16_t *r, uint16_t sample ) {
  if ( r->count == r->size ) {
    // remove the sample being overwritten
    r->sum -= r->buf [ r->head ];
  } else {
    r->count++;
  }

  r->buf [ r->head ] = sample;
  r->sum += sample;

  r->head++;
  if ( r->head >= r->size ) r->head = 0;
}

uint16_t ring_avg_u16_get ( ring_avg_u16_t *r, uint16_t sample ) {
  ring_avg_u16_push ( r, sample );
  if ( r->count == 0 ) return 0;
  return ( uint16_t ) ( r->sum / r->count );
}
void ring_avg_u16_reset ( ring_avg_u16_t *r ) {
  r->sum   = 0;
  r->head  = 0;
  r->count = 0;
  for ( uint16_t i = 0; i < r->size; i++ ) r->buf [ i ] = 0;
}