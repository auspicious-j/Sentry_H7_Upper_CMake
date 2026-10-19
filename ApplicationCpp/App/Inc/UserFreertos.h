#ifndef _USER_FREERTOS_H_
#define _USER_FREERTOS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "cmsis_os.h"

extern osThreadId defaultTaskHandle;
extern osThreadId LEDTaskHandle;
extern osThreadId BeepTaskHandle;
extern osThreadId MotorTaskHandle;
extern osThreadId ErrorTaskHandle;

#ifdef __cplusplus
}
#endif

#endif
