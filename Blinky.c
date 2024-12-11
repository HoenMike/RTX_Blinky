/*----------------------------------------------------------------------------
 *      RL-ARM - RTX
 *----------------------------------------------------------------------------
 *      Name:    BLINKY.C
 *      Purpose: RTX example program
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2011 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#include <RTL.h>
#include "LPC17xx.H"                    /* LPC17xx definitions               */
#include "GLCD.h"
#include "LED.h"

#define __FI        1                   /* Font index 16x24                  */

OS_TID t_phaseA;                        /* assigned task id of task: phase_a */
OS_TID t_phaseB;                        /* assigned task id of task: phase_b */
OS_TID t_phaseC;                        /* assigned task id of task: phase_c */
OS_TID t_phaseD;                        /* assigned task id of task: phase_d */
OS_TID t_clock;                         /* assigned task id of task: clock   */
OS_TID t_lcd;                           /* assigned task id of task: lcd     */

OS_MUT mut_GLCD;                        /* Mutex to controll GLCD access     */

#define LED_A      0
#define LED_B      1
#define LED_C      2
#define LED_D      3
#define LED_CLK    7


/*----------------------------------------------------------------------------
  switch LED on
 *---------------------------------------------------------------------------*/
void LED_on  (unsigned char led) {
  LED_On (led);
  os_mut_wait(mut_GLCD, 0xffff);
  GLCD_SetBackColor(White);
  GLCD_SetTextColor(Green);
  GLCD_DisplayChar(4, 5+led, __FI, 0x80+1); /* Circle Full                   */
  os_mut_release(mut_GLCD);
}

/*----------------------------------------------------------------------------
  switch LED off
 *---------------------------------------------------------------------------*/
void LED_off (unsigned char led) {
  LED_Off(led);
  os_mut_wait(mut_GLCD, 0xffff);
  GLCD_SetBackColor(White);
  GLCD_SetTextColor(Green);
  GLCD_DisplayChar(4, 5+led, __FI, 0x80+0);  /* Circle Empty                 */
  os_mut_release(mut_GLCD);
}


/*----------------------------------------------------------------------------
  Function 'signal_func' called from multiple tasks
 *---------------------------------------------------------------------------*/
void signal_func (OS_TID task)  {
  os_evt_set (0x0100, t_clock);          /* send event signal to clock task  */
  os_dly_wait (50);                      /* delay 50 clock ticks             */
  os_evt_set (0x0100, t_clock);          /* send event signal to clock task  */
  os_dly_wait (50);                      /* delay 50 clock ticks             */
  os_evt_set (0x0001, task);             /* send event to task 'task'        */
  os_dly_wait (50);                      /* delay 50 clock ticks             */
}

/*----------------------------------------------------------------------------
  Task 1 'phaseA': Phase A output
 *---------------------------------------------------------------------------*/
__task void phaseA (void) {
  for (;;) {
    os_evt_wait_and (0x0001, 0xffff);    /* wait for an event flag 0x0001    */
    LED_on (LED_A);
    signal_func (t_phaseB);              /* call common signal function      */
    LED_off(LED_A);
  }
}

/*----------------------------------------------------------------------------
  Task 2 'phaseB': Phase B output
 *---------------------------------------------------------------------------*/
__task void phaseB (void) {
  for (;;) {
    os_evt_wait_and (0x0001, 0xffff);    /* wait for an event flag 0x0001    */
    LED_on (LED_B);
    signal_func (t_phaseC);              /* call common signal function      */
    LED_off(LED_B);
  }
}

/*----------------------------------------------------------------------------
  Task 3 'phaseC': Phase C output
 *---------------------------------------------------------------------------*/
__task void phaseC (void) {
  for (;;) {
    os_evt_wait_and (0x0001, 0xffff);    /* wait for an event flag 0x0001    */
    LED_on (LED_C);
    signal_func (t_phaseD);              /* call common signal function      */
    LED_off(LED_C);
  }
}

/*----------------------------------------------------------------------------
  Task 4 'phaseD': Phase D output
 *---------------------------------------------------------------------------*/
__task void phaseD (void) {
  for (;;) {
    os_evt_wait_and (0x0001, 0xffff);    /* wait for an event flag 0x0001    */
    LED_on (LED_D);
    signal_func (t_phaseA);              /* call common signal function      */
    LED_off(LED_D);
  }
}

/*----------------------------------------------------------------------------
  Task 5 'clock': Signal Clock
 *---------------------------------------------------------------------------*/
__task void clock (void) {
  for (;;) {
    os_evt_wait_and (0x0100, 0xffff);    /* wait for an event flag 0x0100    */
    LED_on (LED_CLK);
    os_dly_wait (8);                     /* delay 8 clock ticks              */
    LED_off(LED_CLK);
  }
}

/*----------------------------------------------------------------------------
  Task 6 'lcd': LCD Control task
 *---------------------------------------------------------------------------*/
__task void lcd (void) {

  for (;;) {
    os_mut_wait(mut_GLCD, 0xffff);
    GLCD_SetBackColor(Blue);
    GLCD_SetTextColor(White);
    GLCD_DisplayString(0, 0, __FI, "    MCB1700 Demo    ");
    GLCD_DisplayString(1, 0, __FI, "    RTX Blinky      ");
    GLCD_DisplayString(2, 0, __FI, "   www.keil.com     ");
    os_mut_release(mut_GLCD);
    os_dly_wait (400);

    os_mut_wait(mut_GLCD, 0xffff);
    GLCD_SetBackColor(Blue);
    GLCD_SetTextColor(Red);
    GLCD_DisplayString(0, 0, __FI, "    MCB1700 Demo    ");
    GLCD_DisplayString(1, 0, __FI, "    RTX Blinky      ");
    GLCD_DisplayString(2, 0, __FI, "   www.keil.com     ");
    os_mut_release(mut_GLCD);
    os_dly_wait (400);
  }
}

/*----------------------------------------------------------------------------
  Task 7 'init': Initialize
 *---------------------------------------------------------------------------*/
__task void init (void) {

  os_mut_init(mut_GLCD);

  t_phaseA = os_tsk_create (phaseA, 0);  /* start task phaseA                */
  t_phaseB = os_tsk_create (phaseB, 0);  /* start task phaseB                */
  t_phaseC = os_tsk_create (phaseC, 0);  /* start task phaseC                */
  t_phaseD = os_tsk_create (phaseD, 0);  /* start task phaseD                */
  t_clock  = os_tsk_create (clock, 0);   /* start task clock                 */
  t_lcd    = os_tsk_create (lcd, 0);     /* start task lcd                   */
  os_evt_set (0x0001, t_phaseA);         /* send signal event to task phaseA */
  os_tsk_delete_self ();
}

/*----------------------------------------------------------------------------
  Main: Initialize and start RTX Kernel
 *---------------------------------------------------------------------------*/
int main (void) {

  LED_Init ();                              /* Initialize the LEDs           */
  GLCD_Init();                              /* Initialize the GLCD           */

  GLCD_Clear(White);                        /* Clear the GLCD                */

  os_sys_init(init);                        /* Initialize RTX and start init */
}
