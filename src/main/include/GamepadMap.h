// GamepadMap.h
//  Mapping of Gamepad axis/buttons
//
//  Author:     K. Sielski FRC Team 1507 - Warlocks (Lockport,NY)
//  Date:       10/14/1028
//
#ifndef GAMEPADMAP_H
#define GAMEPADMAP_H


//AXIS
//              1           5                0
//             0+0         4+4             0 + 1
//              1           5                1
//
//             Left        Right           Values
//
//
#define GAMEPADMAP_AXIS_L_X     0           //Left=-1   Right=1
#define GAMEPADMAP_AXIS_L_Y     1           //Up=-1     Down=1
#define GAMEPADMAP_AXIS_L_TRIG  2           //Off=0    Pressed=1
#define GAMEPADMAP_AXIS_R_TRIG  3           //Off=0    Pressed=1
#define GAMEPADMAP_AXIS_R_X     4           //Left=-1   Right=1
#define GAMEPADMAP_AXIS_R_Y     5           //Up=-1     Down=1

//Buttons
#define GAMEPADMAP_BUTTON_A     1
#define GAMEPADMAP_BUTTON_B     2
#define GAMEPADMAP_BUTTON_X     3
#define GAMEPADMAP_BUTTON_Y     4
#define GAMEPADMAP_BUTTON_LBUMP 5
#define GAMEPADMAP_BUTTON_RBUMP 6
#define GAMEPADMAP_BUTTON_BACK  7
#define GAMEPADMAP_BUTTON_START 8


#endif  //GAMEPADMAP_H