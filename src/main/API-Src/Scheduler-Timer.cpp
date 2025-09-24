/*******************************************************************************
 #  SPDX-License-Identifier: GPL-3.0-or-later                                  #
 #  SPDX-FileCopyrightText: 2025 Drona Aviation                                #
 #  -------------------------------------------------------------------------  #
 #  Copyright (c) 2025 Drona Aviation                                          #
 #  All rights reserved.                                                       #
 #  -------------------------------------------------------------------------  #
 #  Author: Ashish Jaiswal (MechAsh) <AJ>                                      #
 #  Project: MagisV2                                                           #
 #  File: \src\main\API-Src\Scheduler-Timer.cpp                                #
 #  Created Date: Sun, 24th Aug 2025                                           #
 #  Brief:                                                                     #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  Last Modified: Thu, 18th Sep 2025                                          #
 #  Modified By: AJ                                                            #
 #  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  #
 #  HISTORY:                                                                   #
 #  Date      	By	Comments                                                   #
 #  ----------	---	---------------------------------------------------------  #
*******************************************************************************/
#include "API/Scheduler-Timer.h"

#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "platform.h"

#include "common/maths.h"
#include "common/maths.h"

#include "drivers/system.h"

#define TASK_COOKIE 0x5441534Bu /* 'TASK' */

// Ensure the task is initialized properly
static inline void ensure_task_init ( Scheduler_Task *t ) {
  if ( ! t ) return;                    // Return if task pointer is null
  if ( t->_cookie != TASK_COOKIE ) {    // Check if task is not initialized
    memset ( t, 0, sizeof ( *t ) );     // Zero out the memory for task
    t->_cookie = TASK_COOKIE;           // Set the cookie to indicate initialization
  }
}

// Add a new task to the scheduler
bool Scheduler_Add ( Scheduler_Task *t, task_fn_t _task_function, uint32_t interval_ms, bool repeat ) {
  if ( ! t || ! _task_function ) return false;    // Return false if task or function is null
  ensure_task_init ( t );                         // Ensure task is initialized

  ScheduledFn *node = ( ScheduledFn * ) malloc ( sizeof ( ScheduledFn ) );    // Allocate memory for a new scheduled function node
  if ( ! node ) return false;                                                 // Return false if memory allocation fails

  node->task_function = _task_function;    // Assign the task function
  node->interval_ms   = interval_ms;       // Set the interval in milliseconds
  node->last_run_ms   = millis ( );        // Record the current time as last run time
  node->repeat        = repeat;            // Set the repeat flag
  node->next          = t->head;           // Insert node at the beginning of the list
  t->head             = node;              // Update head to point to the new node
  return true;                             // Successfully added the task
}

// Cancel a scheduled task
bool Scheduler_Cancel ( Scheduler_Task *t, task_fn_t _task_function ) {
  if ( ! t || ! _task_function ) return false;    // Return false if task or function is null
  ensure_task_init ( t );                         // Ensure task is initialized

  ScheduledFn *prev = NULL, *cur = t->head;          // Initialize pointers for traversal
  while ( cur ) {                                    // Traverse the list
    if ( cur->task_function == _task_function ) {    // If the function matches
      if ( prev )
        prev->next = cur->next;    // Bypass the current node if it's not the first node
      else
        t->head = cur->next;    // Update head if it's the first node
      free ( cur );             // Free the memory for the canceled task
      return true;              // Task successfully canceled
    }
    prev = cur;
    cur  = cur->next;
  }
  return false;    // Function not found in the list
}

// Clear all scheduled tasks
void Scheduler_Clear ( Scheduler_Task *t ) {
  if ( ! t ) return;               // Return if task pointer is null
  ensure_task_init ( t );          // Ensure task is initialized
  ScheduledFn *cur = t->head;      // Start with the head of the list
  while ( cur ) {                  // Traverse the list
    ScheduledFn *n = cur->next;    // Store next node
    free ( cur );                  // Free current node
    cur = n;                       // Move to next node
  }
  t->head = NULL;    // Set head to null indicating list is empty
}

// Execute scheduled tasks based on their intervals
void Execute_Scheduled ( Scheduler_Task *t ) {
  if ( ! t ) return;         // Return if task pointer is null
  ensure_task_init ( t );    // Ensure task is initialized

  uint32_t now      = millis ( );                                           // Get the current time
  ScheduledFn *prev = NULL, *cur = t->head;                                 // Initialize pointers for traversal
  while ( cur ) {                                                           // Traverse the list
    if ( ( uint32_t ) ( now - cur->last_run_ms ) >= cur->interval_ms ) {    // Check if the interval has passed
      do {
        cur->last_run_ms += cur->interval_ms;    // Update last run time
      } while ( ( uint32_t ) ( now - cur->last_run_ms ) >= cur->interval_ms );

      cur->task_function ( );    // Execute the task function

      if ( ! cur->repeat ) {                // If the task should not repeat
        ScheduledFn *toFree = cur;          // Prepare to free the current node
        cur                 = cur->next;    // Move to the next node
        if ( prev )
          prev->next = cur;    // Bypass the current node if it's not the first node
        else
          t->head = cur;    // Update head if it's the first node
        free ( toFree );    // Free the current node
        continue;           // Continue to the next iteration
      }
    }
    prev = cur;
    cur  = cur->next;
  }
}

// Interval class method to set interval properties and check timing
bool Interval::set ( uint32_t time, bool repeat ) {
  this->repeat = repeat;    // Set the repeat flag

  if ( this->time == 0 ) {                           // If the time is not already set
    this->time     = constrain ( time, 1, 5000 );    // Constrain time within limits and assign
    this->loopTime = millis ( ) + this->time;        // Calculate the loop time
  }

  if ( ( int32_t ) ( millis ( ) - this->loopTime ) >= 0 ) {    // Check if the loop time has been reached
    if ( this->repeat )
      loopTime = millis ( ) + this->time;    // Update loop time for repeating tasks

    return true;    // Interval condition met
  }

  return false;    // Interval condition not met
}

// Reset the interval timing
void Interval::reset ( void ) {
  this->time     = 0;    // Reset time
  this->loopTime = 0;    // Reset loop time
}

// Check if the interval condition is met
bool Interval::check ( ) {
  if ( ( int32_t ) ( millis ( ) - this->loopTime ) >= 0 ) {    // Check if the loop time has been reached
    if ( this->repeat )
      loopTime = millis ( ) + this->time;    // Update loop time for repeating tasks

    return true;    // Interval condition met
  }

  return false;    // Interval condition not met
}
