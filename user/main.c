#include <includes.h>
#include "app_tasks.h"

// Start Task TCB & Stack
static OS_TCB AppTaskStartTCB;
static CPU_STK AppTaskStartStk[256];

int main(void) {
    OS_ERR err;

    OSInit(&err);

    // Create the Start Task
    OSTaskCreate((OS_TCB *)&AppTaskStartTCB,
                 (CPU_CHAR *)"App Task Start",
                 (OS_TASK_PTR)AppTaskStart,
                 (void *)0,
                 (OS_PRIO)3,
                 (CPU_STK *)&AppTaskStartStk[0],
                 (CPU_STK_SIZE)25,
                 (CPU_STK_SIZE)256,
                 (OS_MSG_QTY)0,
                 (OS_TICK)0,
                 (void *)0,
                 (OS_OPT)(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR *)&err);

    OSStart(&err);

    return 0;
}
