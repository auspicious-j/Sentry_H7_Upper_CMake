#include "app_cpp_entry.h"

#include "FreeRTOS.h"
#include "task.h"

// 任务属于平台桥接层；插件本身不创建任务、不调用 vTaskDelayUntil。
extern "C" void AppCpp_ControlTask(void const* argument)
{
    // argument：FreeRTOS 任务参数，当前未使用。
    (void)argument;
    // vTaskDelayUntil 保证周期参考点固定，避免单次计算时间造成周期漂移。
    // 上一次唤醒时间，用于保持固定周期。
    TickType_t last_wake = xTaskGetTickCount();
    // 任务周期，当前为 1 ms。
    const TickType_t period = pdMS_TO_TICKS(1U);

    for (;;) {
        // 业务图在这里运行；任务调度与插件业务保持分离。
        AppCpp_ProcessFrame();
        vTaskDelayUntil(&last_wake, period);
    }
}
