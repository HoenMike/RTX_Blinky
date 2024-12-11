// this is adapted from the example:
// C:\Keil\ARM\Boards\Keil\MCB1700\emWin\GUIDemo

#ifndef __JOYSTICK_H
#define __JOYSTICK_H

// Joystick direction bitmasks
#define JOYSTICK_LEFT    (1 << 0)
#define JOYSTICK_RIGHT   (1 << 1)
#define JOYSTICK_CENTER  (1 << 2)
#define JOYSTICK_UP      (1 << 3)
#define JOYSTICK_DOWN    (1 << 4)

extern void JOYSTICK_Init( void);
extern uint32_t JOYSTICK_GetKeys( void);

#endif 
