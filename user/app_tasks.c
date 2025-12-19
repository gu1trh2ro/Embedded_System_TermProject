#include "app_tasks.h"
#include "includes.h"
#include "globals.h"
#include "bsp.h"

// Global Handles
OS_Q PirDataQ;
OS_Q StateQ;
OS_Q ServoQ;
OS_Q BluetoothRxQ;

const uint8_t MSG_TOUCH_RESET_CODE = 255;

// Task Stacks & TCBs (Defined here or extern if needed elsewhere)
OS_TCB PirTaskTCB;
CPU_STK PirTaskStk[128];

OS_TCB StatusTaskTCB;
CPU_STK StatusTaskStk[128];

OS_TCB ServoTaskTCB;
CPU_STK ServoTaskStk[128];

// Variables
volatile uint32_t idle_counter = 0;
volatile uint32_t suspicious_counter = 0;
volatile uint8_t current_state = 1; // 1: Occupied, 2: Vacant, 3: Suspicious

// --- Tasks ---

void PIR_Task(void *p_arg) {
	OS_ERR err;
    uint8_t pir_val = 0;
    
	while(1) {
        pir_val = PIR_Read();
        if (pir_val == 1) { // Motion
             OSQPost(&PirDataQ, &pir_val, sizeof(uint8_t), OS_OPT_POST_FIFO, &err);
        }
		OSTimeDlyHMSM(0, 0, 0, 50, OS_OPT_TIME_HMSM_STRICT, &err);
	}
}

void Status_Task(void *p_arg) {
	OS_ERR err;
    void *msg;
    OS_MSG_SIZE size;
    uint16_t light_val = 0;
    
	while(1) {
        // Wait for PIR data or Timeout (1s)
        msg = OSQPend(&PirDataQ, 1000, OS_OPT_PEND_BLOCKING, &size, NULL, &err);

        if (err == OS_ERR_NONE) {
            // Motion Detected -> Occupied
            idle_counter = 0;
            suspicious_counter = 0;
            if (current_state != 1) {
                current_state = 1;
                // Notify Display/BT
            }
        } else if (err == OS_ERR_TIMEOUT) {
            // No Motion for 1 sec
            idle_counter++;
            
            if (current_state == 1 && idle_counter >= 10) { // 10s demo (req 10min)
                 // Check Light
                 light_val = LightSensor_Read();
                 if (light_val > 3500) { // Bright -> Suspicious
                     current_state = 3;
                 } else { // Dark -> Vacant
                     current_state = 2;
                 }
                 // Reset counters? No, keep logic 
            }
            
            if (current_state == 3) {
                suspicious_counter++;
                if (suspicious_counter > 15) { // Demo threshold
                    // Trigger Servo
                    uint8_t servo_cmd = 1;
                    OSQPost(&ServoQ, &servo_cmd, 1, OS_OPT_POST_FIFO, &err);
                    suspicious_counter = 0;
                }
            }
        }
        
        // Output State to LED/LCD
        if(current_state == 1) {
            LED_Set(1, 1); LED_Set(2, 0); // Green ON
        } else if (current_state == 2) {
            LED_Set(1, 0); LED_Set(2, 0); // All OFF
        } else {
            LED_Set(1, 0); LED_Set(2, 1); // Red ON
        }
	}
}

void Servo_Task(void *p_arg) {
    OS_ERR err;
    void *msg;
    OS_MSG_SIZE size;
    
    while(1) {
        msg = OSQPend(&ServoQ, 0, OS_OPT_PEND_BLOCKING, &size, NULL, &err);
        if (err == OS_ERR_NONE) {
            // Wave Flag
            Servo_SetAngle(90);
            OSTimeDlyHMSM(0, 0, 1, 0, OS_OPT_TIME_HMSM_STRICT, &err);
            Servo_SetAngle(0);
        }
    }
}

void AppTaskStart(void *p_arg) {
	OS_ERR err;
	
    BSP_Init(); // Hardware Init
    
    // Create RTOS Objects
    OSQCreate(&PirDataQ, "PirQ", 10, &err);
    OSQCreate(&ServoQ, "ServoQ", 5, &err);
    OSQCreate(&BluetoothRxQ, "BluetoothRxQ", 10, &err); // Add Bluetooth Queue
    
    // Create Application Tasks
    OSTaskCreate(&PirTaskTCB, "PIR Task", PIR_Task, 0, 4, &PirTaskStk[0], 12, 128, 0, 0, 0, OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR, &err);
    OSTaskCreate(&StatusTaskTCB, "Status Task", Status_Task, 0, 5, &StatusTaskStk[0], 12, 128, 0, 0, 0, OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR, &err);
    OSTaskCreate(&ServoTaskTCB, "Servo Task", Servo_Task, 0, 6, &ServoTaskStk[0], 12, 128, 0, 0, 0, OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR, &err);
    
    // EXTI IRQ linking might be needed in main or stm32f10x_it.c
}
