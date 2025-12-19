#ifndef __APP_TASKS_H__
#define __APP_TASKS_H__

void AppTaskStart(void *p_arg);
void PIR_Task(void *p_arg);
void Light_Sensor_Task(void *p_arg);
void Status_Task(void *p_arg);
void Display_Task(void *p_arg);
void Bluetooth_Task(void *p_arg);
void Servo_Task(void *p_arg);

#endif // __APP_TASKS_H__
