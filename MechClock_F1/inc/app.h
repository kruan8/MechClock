/*
 * app.h
 *
 *  Created on: 25. 9. 2018
 *  Author:     Priesol Vladimir
 */

#ifndef APP_H_
#define APP_H_

#include <stdbool.h>

void App_Init(void);
void App_Exec(void);
void App_MinuteImpulse();

void App_SystickCallbackINT(void);
void APP_SetStopMode();

#endif /* APP_H_ */
