#include "app_tasks.h"
#include "includes.h"
#include "globals.h"
#include "bsp.h"
#include "lcd.h" // Added LCD header

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

OS_TCB DisplayTaskTCB;
CPU_STK DisplayTaskStk[256]; // Increased Stack

OS_TCB BluetoothTaskTCB;
CPU_STK BluetoothTaskStk[128];

// Variables
volatile uint32_t idle_counter = 0;
volatile uint32_t suspicious_counter = 0;
volatile uint8_t current_state = 1; // 1: Occupied, 2: Vacant, 3: Suspicious

volatile uint8_t g_debug_pir_val = 0; // GLOBAL DEBUG VARIABLE

// --- Tasks ---

void PIR_Task(void *p_arg) {
	OS_ERR err;
    uint8_t pir_val = 0;
    
	while(1) {
        pir_val = PIR_Read();
        g_debug_pir_val = pir_val;
        
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
    uint8_t state_msg = 0;
    
    // Initial State Update
    OSQPost(&StateQ, (void *)&current_state, sizeof(uint8_t), OS_OPT_POST_FIFO, &err);

	while(1) {
        // Wait for PIR data or Timeout (1s)
        msg = OSQPend(&PirDataQ, 1000, OS_OPT_PEND_BLOCKING, &size, NULL, &err);

        if (err == OS_ERR_NONE) {
            // Motion Detected -> Occupied
            idle_counter = 0;
            suspicious_counter = 0;
            if (current_state != 1) {
                current_state = 1;
                state_msg = current_state;
                OSQPost(&StateQ, &state_msg, sizeof(uint8_t), OS_OPT_POST_FIFO, &err);
            }
        } else if (err == OS_ERR_TIMEOUT) {
            // No Motion for 1 sec
            idle_counter++;
            
            if (current_state == 1 && idle_counter >= 5) { // 5s test (req 10min)
                 // Check Light
                 light_val = LightSensor_Read();
                 if (light_val > 3500) { // Bright -> Suspicious
                     current_state = 3;
                 } else { // Dark -> Vacant
                     current_state = 2;
                 }
                 state_msg = current_state;
                 OSQPost(&StateQ, &state_msg, sizeof(uint8_t), OS_OPT_POST_FIFO, &err);
            }
            
            if (current_state == 3) {
                suspicious_counter++;
                if (suspicious_counter > 15) { // Demo threshold: 15s? (Req 5min)
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

void Display_Task(void *p_arg) {
    OS_ERR err;
    void *msg;
    OS_MSG_SIZE size;
    uint8_t rx_state = 0;
    char str_buf[30];
    char debug_buf[30];
    uint32_t live_tick = 0;
    
    LCD_Clear(WHITE);
    LCD_ShowString(10, 10, "System Ready", BLACK, WHITE);
    
    while(1) {
        // Check for State Update (Non-blocking)
        msg = OSQPend(&StateQ, 0, OS_OPT_PEND_NON_BLOCKING, &size, NULL, &err);
        
        if (err == OS_ERR_NONE) {
            rx_state = *(uint8_t*)msg;
        }
        
        // Update Screen
        if (current_state == 1) {
             LCD_ShowString(10, 40, "STATUS: OCCUPIED  ", BLACK, WHITE);
             sprintf(str_buf, "TIME: %d s        ", 0);
        } else if (current_state == 2) {
             LCD_ShowString(10, 40, "STATUS: VACANT    ", BLACK, WHITE);
             sprintf(str_buf, "TIME: %d s        ", idle_counter);
        } else if (current_state == 3) {
             LCD_ShowString(10, 40, "STATUS: SUSPICIOUS", BLACK, WHITE);
             sprintf(str_buf, "TIME: %d s        ", suspicious_counter);
        }
        LCD_ShowString(10, 70, str_buf, BLACK, WHITE);
        
        // Debug Output (Real-time PIR + Live Tick)
        sprintf(debug_buf, "PIR:%d  LIVE:%d", g_debug_pir_val, live_tick++);
        LCD_ShowString(10, 100, debug_buf, RED, WHITE);

        // Refresh Rate: 100ms
        OSTimeDlyHMSM(0, 0, 0, 100, OS_OPT_TIME_HMSM_STRICT, &err);
    }
}

void Bluetooth_Task(void *p_arg) {
    OS_ERR err;
    void *msg;
    OS_MSG_SIZE size;
    
    while(1) {
         OSTimeDlyHMSM(0, 0, 1, 0, OS_OPT_TIME_HMSM_STRICT, &err);
    }
}

void AppTaskStart(void *p_arg) {
	OS_ERR err;
	
    BSP_Init(); 
    
    // Create RTOS Objects
    OSQCreate(&PirDataQ, "PirQ", 10, &err);
    OSQCreate(&StateQ, "StateQ", 10, &err);
    OSQCreate(&ServoQ, "ServoQ", 5, &err);
    OSQCreate(&BluetoothRxQ, "BluetoothRxQ", 10, &err); 
    
    // Create Application Tasks
    OSTaskCreate(&PirTaskTCB, "PIR Task", PIR_Task, 0, 4, &PirTaskStk[0], 12, 128, 0, 0, 0, OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR, &err);
    OSTaskCreate(&StatusTaskTCB, "Status Task", Status_Task, 0, 5, &StatusTaskStk[0], 12, 128, 0, 0, 0, OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR, &err);
    // Updated Stack Size to 256
    OSTaskCreate(&DisplayTaskTCB, "Display Task", Display_Task, 0, 6, &DisplayTaskStk[0], 12, 256, 0, 0, 0, OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR, &err);
    OSTaskCreate(&BluetoothTaskTCB, "BT Task", Bluetooth_Task, 0, 6, &BluetoothTaskStk[0], 12, 128, 0, 0, 0, OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR, &err);
    OSTaskCreate(&ServoTaskTCB, "Servo Task", Servo_Task, 0, 7, &ServoTaskStk[0], 12, 128, 0, 0, 0, OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR, &err);
}
