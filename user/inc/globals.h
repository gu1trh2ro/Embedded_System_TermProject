#ifndef __GLOBALS_H__
#define __GLOBALS_H__

#include <includes.h> // uC/OS-III includes

// Global Queues
extern OS_Q PirDataQ;
extern OS_Q StateQ;
extern OS_Q ServoQ;

// System States
typedef enum {
    STATE_OCCUPIED = 1,
    STATE_VACANT,
    STATE_SUSPICIOUS
} SystemState;

#endif // __GLOBALS_H__
