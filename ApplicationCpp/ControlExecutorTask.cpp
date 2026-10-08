#include "app_cpp_entry.h"

#include "FreeRTOS.h"
#include "task.h"

extern "C" void AppCpp_ControlTask(void const* argument)
{
    (void)argument;
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(1U);

    for (;;) {
        AppCpp_ProcessFrame();
        vTaskDelayUntil(&last_wake, period);
    }
}
