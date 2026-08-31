#ifndef	__TASK_MANAGER_H
#define	__TASK_MANAGER_H
#include "stm32f10x.h"

/*
    第一：状态机枚举。状态机相较于调度层为接口关系，所以在头文件声明
*/
typedef	enum{
    SYS_IDLE,
    SYS_CHECK,
    SYS_STARTUP,
    SYS_BALANCE,
    SYS_SLEEP,
} SYS_Event; 


/*第五：对外接口，主循环调用*/
void Task_Manager_Run(void);

void Task_Manager_SetEvent(SYS_Event event);


#endif	//!__TASK_MANAGER_H

