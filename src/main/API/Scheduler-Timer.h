/*******************************************************************************
 #  SPDX-License-Identifier: GPL-3.0-or-later                                  #
 #  SPDX-FileCopyrightText: 2025 Drona Aviation                                #
 #  -------------------------------------------------------------------------  #
 #  Copyright (c) 2025 Drona Aviation                                          #
 #  All rights reserved.                                                       #
 #  -------------------------------------------------------------------------  #
 #  Author: Ashish Jaiswal (MechAsh) <AJ>                                      #
 #  Project: MagisV2                                                           #
 #  File: \src\main\API\Scheduler-Timer.h                                      #
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

#ifndef FC_CONFIG_SCHEDULING_H
#define FC_CONFIG_SCHEDULING_H

#include <stdint.h>
#include <stdbool.h>

// Define a function pointer type `task_fn_t` for tasks that take no arguments and return void.
typedef void ( *task_fn_t ) ( void );

// Define a structure `ScheduledFn` to represent a scheduled task.
typedef struct ScheduledFn {
  task_fn_t task_function;     // Pointer to the function representing the task.
  uint32_t interval_ms;        // Interval in milliseconds for the task to run.
  uint32_t last_run_ms;        // Last run time of the task in milliseconds.
  bool repeat;                 // Flag indicating if the task should repeat.
  struct ScheduledFn *next;    // Pointer to the next scheduled task in the list.
} ScheduledFn;

// Define a structure `Scheduler_Task` to hold the head of the linked list of scheduled tasks.
typedef struct {
  uint32_t _cookie;     // Lazy-init guard used for initialization verification.
  ScheduledFn *head;    // Pointer to the head of the list of scheduled tasks.
} Scheduler_Task;

/**
 * @brief Add a new task to the scheduler.
 *
 * This function adds a new task to the list of scheduled tasks with a specified interval and repeat option.
 * It returns true if the task is successfully added, otherwise false if the task or function is null or memory allocation fails.
 *
 * @param t Pointer to the Scheduler_Task structure that will hold the new scheduled function.
 * @param _task_function The function pointer representing the task to be scheduled.
 * @param interval_ms The time interval in milliseconds after which the task should run.
 * @param repeat A boolean flag indicating whether the task should repeat.
 * @return true if the task is successfully added, false otherwise.
 */
bool Scheduler_Add ( Scheduler_Task *t, task_fn_t _task_function, uint32_t interval_ms, bool repeat );

/**
 * @brief Cancel a specific scheduled task.
 *
 * This function searches for and removes a task from the list of scheduled tasks based on the provided task function
 * pointer. It returns true if the task is successfully canceled, otherwise false if the task or function is null or not found.
 *
 * @param t Pointer to the Scheduler_Task structure containing the list of scheduled functions.
 * @param _task_function The task function to cancel.
 * @return true if the task is successfully canceled, false otherwise.
 */
bool Scheduler_Cancel ( Scheduler_Task *t, task_fn_t _task_function );

/**
 * @brief Clear all scheduled tasks.
 *
 * This function frees all nodes in the list of scheduled tasks and sets the head of the list to null, indicating that
 * no tasks remain scheduled.
 *
 * @param t Pointer to the Scheduler_Task structure containing the list of scheduled functions.
 */
void Scheduler_Clear ( Scheduler_Task *t );

/**
 * @brief Execute scheduled tasks based on their intervals.
 *
 * This function iterates through a list of scheduled tasks and executes them if their interval has elapsed.
 * Tasks are removed from the list if they are not set to repeat.
 *
 * @param t Pointer to the Scheduler_Task structure containing the list of scheduled functions.
 */
void Execute_Scheduled ( Scheduler_Task *t );

#ifdef __cplusplus
extern "C" {
#endif

uint32_t micros ( void );
uint32_t millis ( void );
class Interval {

 private:
  uint32_t time;
  uint32_t loopTime;
  bool repeat;

 public:
  /**
   * @brief Configure interval timing and repeat behavior.
   *
   * Sets the interval time and repeat flag, ensuring the time is within 1 to 5000 ms.
   * Checks if the interval has elapsed, updating loopTime for repeats as needed.
   *
   * @param time   Interval time in milliseconds.
   * @param repeat Repeat flag for the interval.
   * @return True if the interval has elapsed, otherwise false.
   */
  bool set ( uint32_t time, bool repeat );    // time is in milliseconds
  /**
   * @brief Reset the interval timing.
   *
   * Sets both time and loopTime to zero, effectively resetting the interval's timing state.
   */

  void reset ( void );

  /**
   * @brief Checks if the interval condition is met.
   *
   * Compares current time with loop time to determine if the interval has elapsed.
   * Updates loop time if repeating.
   *
   * @return true if the condition is met, false otherwise.
   */
  bool check ( );
};

#ifdef __cplusplus
}
#endif

#endif
