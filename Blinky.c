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
#include "joystick.h"

#define __FI        1                   /* Font index 16x24                  */

// Stepper Motor Lookup Tables
uint8_t wave_drive_lookup_table[4][4] = {
    {1, 0, 0, 0}, // Wave drive anti-clockwise sequence
    {0, 1, 0, 0},
    {0, 0, 1, 0},
    {0, 0, 0, 1}
};

uint8_t half_step_lookup_table[8][4] = {
    {1, 0, 0, 0}, // Half-step anti-clockwise sequence
    {1, 1, 0, 0},
    {0, 1, 0, 0},
    {0, 1, 1, 0},
    {0, 0, 1, 0},
    {0, 0, 1, 1},
    {0, 0, 0, 1},
    {0, 1, 0, 1}
};

uint8_t full_step_lookup_table[4][4] = {
    {1, 1, 0, 0}, // Full-step anti-clockwise sequence
    {0, 1, 1, 0},
    {0, 0, 1, 1},
    {1, 0, 0, 1}
};

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


// Global variables for stepper control
volatile uint8_t (*lookup_table)[4] = full_step_lookup_table; // Default to full-step
volatile uint8_t lookup_table_size = 4;                       // Default size for full-step
volatile uint8_t current_mode = 0;                            // 0: full-step, 1: half-step, 2: wave-drive
volatile uint32_t step_delay = (1 << 14);                     // Delay between steps

// Stepper motor control functions
void set_stepper_outputs(int index)
{
    // Clear or set GPIO pins based on lookup table
    if (lookup_table[index][0] == 0)
    {
        LPC_GPIO0->FIOCLR |= 1 << 0;
    }
    else
    {
        LPC_GPIO0->FIOSET |= 1 << 0;
    }
    if (lookup_table[index][1] == 0)
    {
        LPC_GPIO0->FIOCLR |= 1 << 1;
    }
    else
    {
        LPC_GPIO0->FIOSET |= 1 << 1;
    }
    if (lookup_table[index][2] == 0)
    {
        LPC_GPIO0->FIOCLR |= 1 << 2;
    }
    else
    {
        LPC_GPIO0->FIOSET |= 1 << 2;
    }
    if (lookup_table[index][3] == 0)
    {
        LPC_GPIO0->FIOCLR |= 1 << 3;
    }
    else
    {
        LPC_GPIO0->FIOSET |= 1 << 3;
    }
}

void change_stepper_mode(void)
{
    os_mut_wait(mut_Stepper, 0xffff);
    
    current_mode = (current_mode + 1) % 3;
    
    switch (current_mode)
    {
    case 0:
        lookup_table = full_step_lookup_table;
        lookup_table_size = 4;
        os_mut_wait(mut_GLCD, 0xffff);
        GLCD_DisplayString(3, 0, __FI, "Mode: Full-Step   ");
        os_mut_release(mut_GLCD);
        break;
    case 1:
        lookup_table = half_step_lookup_table;
        lookup_table_size = 8;
        os_mut_wait(mut_GLCD, 0xffff);
        GLCD_DisplayString(3, 0, __FI, "Mode: Half-Step   ");
        os_mut_release(mut_GLCD);
        break;
    case 2:
        lookup_table = wave_drive_lookup_table;
        lookup_table_size = 4;
        os_mut_wait(mut_GLCD, 0xffff);
        GLCD_DisplayString(3, 0, __FI, "Mode: Wave-Drive  ");
        os_mut_release(mut_GLCD);
        break;
    }
    
    os_mut_release(mut_Stepper);
}
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
  Task for Stepper Motor Control
 *---------------------------------------------------------------------------*/
__task void stepper_task(void) {
    uint32_t direction = 1; // 1 for clockwise, 0 for anti-clockwise
    uint32_t i;
    
    for (;;) {
        uint32_t joy_keys = JOYSTICK_GetKeys();
        
        // Change mode on UP/DOWN
        if (joy_keys & (JOYSTICK_UP | JOYSTICK_DOWN)) {
            change_stepper_mode();
        }
        
        // Rotate clockwise on RIGHT
        if (joy_keys & JOYSTICK_RIGHT) {
            os_mut_wait(mut_Stepper, 0xffff);
            for (i = 0; i < lookup_table_size; i++) {
                set_stepper_outputs(i);
                os_dly_wait(step_delay / 10);
            }
            os_mut_release(mut_Stepper);
        }
        
        // Rotate anti-clockwise on LEFT
        if (joy_keys & JOYSTICK_LEFT) {
            os_mut_wait(mut_Stepper, 0xffff);
            for (i = lookup_table_size; i > 0; i--) {
                set_stepper_outputs(i - 1);
                os_dly_wait(step_delay / 10);
            }
            os_mut_release(mut_Stepper);
        }
        
        // Short delay to prevent constant polling
        os_dly_wait(10);
    }
}

/*----------------------------------------------------------------------------
  Task 7 'init': Initialize
 *---------------------------------------------------------------------------*/
__task void init (void) {
    // Initialize GPIO for stepper motor
    LPC_SC->PCONP |= (1 << 15);
    LPC_GPIO0->FIODIR |= (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3);
    LPC_GPIO0->FIOCLR |= (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3);

    // Initialize mutexes
    os_mut_init(mut_GLCD);
    os_mut_init(mut_Stepper);

    // Create tasks (same as before, with added stepper task)
    t_phaseA = os_tsk_create (phaseA, 0);  /* start task phaseA                */
    t_phaseB = os_tsk_create (phaseB, 0);  /* start task phaseB                */
    t_phaseC = os_tsk_create (phaseC, 0);  /* start task phaseC                */
    t_phaseD = os_tsk_create (phaseD, 0);  /* start task phaseD                */
    t_clock  = os_tsk_create (clock, 0);   /* start task clock                 */
    t_lcd    = os_tsk_create (lcd, 0);     /* start task lcd                   */
    t_stepper = os_tsk_create(stepper_task, 0); /* start stepper motor task    */
    
    os_evt_set (0x0001, t_phaseA);         /* send signal event to task phaseA */
    os_tsk_delete_self ();
}

/*----------------------------------------------------------------------------
  Main: Initialize and start RTX Kernel
 *---------------------------------------------------------------------------*/
int main (void) {
    LED_Init ();                           /* Initialize the LEDs              */
    GLCD_Init();                           /* Initialize the GLCD              */
    JOYSTICK_Init();                       /* Initialize the Joystick          */
    
    GLCD_Clear(White);                     /* Clear the GLCD                   */

    // Initial GLCD setup for stepper mode
    GLCD_SetBackColor(Blue);
    GLCD_SetTextColor(White);
    GLCD_DisplayString(3, 0, __FI, "Mode: Full-Step   ");

    os_sys_init(init);                     /* Initialize RTX and start init    */
    
    return 0;
}
