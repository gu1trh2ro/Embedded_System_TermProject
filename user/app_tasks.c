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
const uint8_t MSG_TOUCH_RELEASE_CODE = 254;

// Task Stacks & TCBs (Defined here or extern if needed elsewhere)
OS_TCB PirTaskTCB;
CPU_STK PirTaskStk[256]; // Increased Stack due to sprintf/multiple sensors

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
    static uint8_t prev_pir_val = 255; 
    
    // Touch Sensor Variables
    uint8_t touch_val = 0;
    static uint8_t prev_touch_val = 0;
    uint8_t state_msg = 0;

	while(1) {
        // --- 1. PIR Sensor Handling ---
        pir_val = PIR_Read();
        g_debug_pir_val = pir_val;
        
        if (pir_val != prev_pir_val) {
            // Raw PIR status sending removed to prioritize State-based status (OCCUPIED/EMPTY)
            prev_pir_val = pir_val;
        }
        
        // --- 3. Light Sensor Handling (PC2) ---
        // Threshold Logic: 0~99 (Dark) vs 100~4095 (Bright)
        // Send value only when crossing the threshold.
        uint16_t light_val = LightSensor_Read();
        uint8_t current_category = (light_val >= 100) ? 1 : 0; // 0: Dark, 1: Bright
        static uint8_t prev_category = 2; // 2: Unknown (Initial status)
        
        if (prev_category == 2) {
            prev_category = current_category; // Sync initial state without sending
        } else if (current_category != prev_category) {
             if (current_category == 1) {
                 Bluetooth_SendString("ON\r\n");
             } else {
                 Bluetooth_SendString("OFF\r\n");
             }
             prev_category = current_category;
        }
        
        if (pir_val == 1) { 
             static uint8_t report_tick = 0;
             if (++report_tick >= 20) {
                 OSQPost(&PirDataQ, &pir_val, sizeof(uint8_t), OS_OPT_POST_FIFO, &err);
                 report_tick = 0;
             }
        }

        // --- 2. Touch Sensor Handling ---
        // Reading TOUCH_PIN which is now mapped to PC0
        touch_val = GPIO_ReadInputDataBit(TOUCH_PORT, TOUCH_PIN);
        
        if (touch_val != prev_touch_val) {
            if (touch_val == 1) { // 1 = Pressed
                 // Toggle Away Mode
                 if (current_state == 4) {
                     // Currently Away -> Back to Occupied
                     current_state = 1;
                     Bluetooth_SendString("BACK\r\n");
                     state_msg = current_state;
                     OSQPost(&StateQ, &state_msg, sizeof(uint8_t), OS_OPT_POST_FIFO, &err);
                 } else {
                     // Currently Occupied/Vacant/Suspicious -> Set to Away
                     current_state = 4;
                     Bluetooth_SendString("AWAY\r\n");
                     state_msg = current_state;
                     OSQPost(&StateQ, &state_msg, sizeof(uint8_t), OS_OPT_POST_FIFO, &err);
                 }
                 
                 // Reset Status Task Counters (Fix for Away Timer accumulation)
                 uint8_t reset_msg = MSG_TOUCH_RESET_CODE;
                 OSQPost(&PirDataQ, &reset_msg, sizeof(uint8_t), OS_OPT_POST_FIFO, &err);
                 
                 idle_counter = 0;
                 suspicious_counter = 0;
            } 
            prev_touch_val = touch_val;
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
    
    // New Logic Variables
    static uint32_t away_timer = 0;
    static uint8_t action_triggered = 0; // 0: Ready, 1: Triggered (Wait for reset)

    // Initial State Update (Start as BACK/Occupied = 1)
    current_state = 1; 
    OSQPost(&StateQ, (void *)&current_state, sizeof(uint8_t), OS_OPT_POST_FIFO, &err);

	while(1) {
        // Wait for PIR data or Timeout (1s)
        msg = OSQPend(&PirDataQ, 1000, OS_OPT_PEND_BLOCKING, &size, NULL, &err);

        if (err == OS_ERR_NONE) {
            uint8_t val = *(uint8_t *)msg;
            
            if (val == MSG_TOUCH_RESET_CODE) {
                // Touch Press -> Button Logic handled in PIR_Task, but status confirmed here?
                // Actually PIR_Task handles the toggle and updates current_state.
                // We just receive notification here to clear counters.
                Bluetooth_SendString("TOUCHED\r\n");
                idle_counter = 0;
                away_timer = 0;
                action_triggered = 0; // Reset trigger on user interaction
                
                // State update is handled in PIR_Task, but we ensure it matches
                // If PIR_Task toggled it, current_state is already updated.
            } else if (val == MSG_TOUCH_RELEASE_CODE) {
                 Bluetooth_SendString("TOUCH_RELEASED\r\n");
            } else {
                // PIR Motion (val == 1)
                // In BACK mode, motion resets EMPTY status
                if (current_state != 4) { 
                    idle_counter = 0;
                    if (current_state != 1) { // If was 'Vacant' logic locally
                        current_state = 1;
                        Bluetooth_SendString("OCCUPIED\r\n"); // Notify State Change
                        state_msg = current_state;
                        OSQPost(&StateQ, &state_msg, sizeof(uint8_t), OS_OPT_POST_FIFO, &err);
                    }
                }
            }
        } else if (err == OS_ERR_TIMEOUT) {
            // 1 Second Tick
            uint8_t trigger_vacant = 0;
            
            if (current_state == 4) { 
                // --- AWAY MODE ---
                away_timer++;
                if (away_timer >= 13) {
                    trigger_vacant = 1;
                }
            } else if (current_state == 1) {
                // --- BACK MODE (Occupied) ---
                idle_counter++; 
                if (idle_counter >= 5) { 
                    trigger_vacant = 1;
                }
            }
            
            // Unified State Transition to VACANT
            if (trigger_vacant == 1 && current_state != 2) {
                 current_state = 2; // Set to Vacant
                 Bluetooth_SendString("EMPTY\r\n"); 
                 
                 // 1. Fire Servo
                 uint8_t servo_cmd = 1;
                 OSQPost(&ServoQ, &servo_cmd, 1, OS_OPT_POST_FIFO, &err);
                 
                 // 2. Check Light
                 light_val = LightSensor_Read();
                 if (light_val >= 100) {
                     Bluetooth_SendString("turn off the light\r\n");
                 }
                 
                 // 3. Update Display
                 state_msg = current_state;
                 OSQPost(&StateQ, &state_msg, sizeof(uint8_t), OS_OPT_POST_FIFO, &err);
            }
        }
        
        // Output State to LED
        if(current_state == 4) {
            LED_Set(1, 1); LED_Set(2, 1); // Orange (Away)
        } else {
            // BACK Mode (Occupied or Vacant)
            if (idle_counter < 5) {
                 LED_Set(1, 1); LED_Set(2, 0); // Green (Occupied)
            } else {
                 LED_Set(1, 0); LED_Set(2, 0); // Off (Vacant)
            }
        }
	}
}

void Servo_Task(void *p_arg) {
    OS_ERR err;
    void *msg;
    OS_MSG_SIZE size;
    
    // --- One-time Run for Verification ---
    Servo_SetAngle(90);
    OSTimeDlyHMSM(0, 0, 1, 0, OS_OPT_TIME_HMSM_STRICT, &err);
    Servo_SetAngle(0);
    OSTimeDlyHMSM(0, 0, 1, 0, OS_OPT_TIME_HMSM_STRICT, &err);
    // -------------------------------------

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
    char str_buf[30];
    char debug_buf[30];
    // Uptime Tracking
    static uint32_t uptime_sec = 0;
    static uint8_t tick_100ms = 0;
    
    // Previous State Tracking for Optimization
    static uint8_t prev_state = 255; 
    static uint32_t prev_idle = 255;
    static uint32_t prev_suspicious = 255;
    static uint8_t prev_pir_val = 255;
    static uint32_t prev_uptime = 255;
    
    LCD_Clear(WHITE);
    LCD_ShowString(10, 10, "System Ready", BLACK, WHITE);
    
    while(1) {
        // Update Uptime (1s interval)
        if (++tick_100ms >= 10) {
            uptime_sec++;
            tick_100ms = 0;
        }

        // 1. Update Status/Timer Area (Only on change)
        if (current_state != prev_state || 
            (current_state == 2 && idle_counter != prev_idle) ||
            (current_state == 3 && suspicious_counter != prev_suspicious)) 
        {
            if (current_state == 1) {
                 LCD_ShowString(10, 40, "STATUS: OCCUPIED  ", BLACK, WHITE);
                 sprintf(str_buf, "VACANT: %d s      ", 0);
            } else if (current_state == 2) {
                 LCD_ShowString(10, 40, "STATUS: VACANT    ", BLACK, WHITE);
                 sprintf(str_buf, "VACANT: %d s      ", idle_counter);
            } else if (current_state == 3) {
                 LCD_ShowString(10, 40, "STATUS: SUSPICIOUS", BLACK, WHITE);
                 sprintf(str_buf, "SUSPIC: %d s      ", suspicious_counter);
            } else if (current_state == 4) {
                 LCD_ShowString(10, 40, "STATUS: AWAY      ", BLACK, WHITE);
                 sprintf(str_buf, "User Away         ");
            }
            LCD_ShowString(10, 70, str_buf, BLACK, WHITE);
            
            // Update tracking variables
            prev_state = current_state;
            prev_idle = idle_counter;
            prev_suspicious = suspicious_counter;
        }
        
        // 2. Debug Output (PIR Value - Only on change)
        if (g_debug_pir_val != prev_pir_val) {
            sprintf(debug_buf, "PIR:%d            ", g_debug_pir_val); // Spaces to clear old text
            LCD_ShowString(10, 100, debug_buf, RED, WHITE);
            prev_pir_val = g_debug_pir_val;
        }

        // 3. Uptime (Only on change - every 1s)
        if (uptime_sec != prev_uptime) {
            sprintf(str_buf, "UPTIME: %d s", uptime_sec);
            LCD_ShowString(10, 130, str_buf, BLUE, WHITE);
            prev_uptime = uptime_sec;
        }

        // Refresh Rate: 100ms
        OSTimeDlyHMSM(0, 0, 0, 100, OS_OPT_TIME_HMSM_STRICT, &err);
    }
}

void Bluetooth_Task(void *p_arg) {
    OS_ERR err;
    
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
    
    // Enable SysTick for OS Time (Critical for OSTimeDly to work)
    OS_CPU_SysTickInit(SystemCoreClock / OS_CFG_TICK_RATE_HZ);
    
    // Create Application Tasks
    // Status Task (5), PIR Task (4)
    // PIR Task Stack increased to 256
    OSTaskCreate(&PirTaskTCB, "PIR Task", PIR_Task, 0, 4, &PirTaskStk[0], 25, 256, 0, 0, 0, OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR, &err);
    OSTaskCreate(&StatusTaskTCB, "Status Task", Status_Task, 0, 5, &StatusTaskStk[0], 12, 128, 0, 0, 0, OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR, &err);
    
    // Display Task (6) - Increased Stack
    OSTaskCreate(&DisplayTaskTCB, "Display Task", Display_Task, 0, 6, &DisplayTaskStk[0], 12, 256, 0, 0, 0, OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR, &err);
    
    // Servo Task (7)
    OSTaskCreate(&ServoTaskTCB, "Servo Task", Servo_Task, 0, 7, &ServoTaskStk[0], 12, 128, 0, 0, 0, OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR, &err);

    // BT Task (8) - Lower priority than Servo/Display to avoid conflict
    OSTaskCreate(&BluetoothTaskTCB, "BT Task", Bluetooth_Task, 0, 8, &BluetoothTaskStk[0], 12, 128, 0, 0, 0, OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR, &err);
}
