#ifndef	__TASK_MANAGER_H
#define	__TASK_MANAGER_H
#include "stm32f10x.h"

/*
    First: the state-machine enum. The state machine has an interface
    relationship with the scheduling layer, so it's declared in the header
*/
typedef	enum{
    SYS_IDLE,
    SYS_CHECK,
    SYS_STARTUP,
    SYS_BALANCE,
    SYS_SLEEP,
} SYS_Event;


/* Fifth: external interface, called by the main loop */
void Task_Manager_Run(void);

void Task_Manager_SetEvent(SYS_Event event);


#endif	//!__TASK_MANAGER_H

