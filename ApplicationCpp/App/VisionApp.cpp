extern "C"
{
#include "Vision.h"
#include "Shooter.h"
#include "cmsis_os.h"
#include "stm32h7xx_hal.h"
#include "usbd_cdc_if.h"
#include "Chassis.h"
#include "Gimbal.h"
#include "imu_temp_ctrl.h"
#include "Shooter.h"
#include "Judge.h"
#include "USER_B2B.h"
#include "arm_math.h"
#include "string.h"

#include <stdint.h>
#include <float.h>
#include <math.h>
#include <stdbool.h>

#include "VisionLinkApi.h"
}

#include "VisionLink.hpp"
#include "VisionTargetMailbox.hpp"

VisionTransmit vision_transmit;
VisionReceive vision_receive;
VisionTarget vision_target;

// 视觉接收链路已迁移到 Io/Vision：
//   USB 中断 -> VisionLink_OnUsbReceive() -> VisionTask 中的 VisionLink_Poll()
//   -> vision_frame_mailbox() -> Gimbal 图的 Vision_Target_Node。
// 旧的 Vision_DataReceive / Vision_UseData（在中断里直接写全局）已删除。
// 说明：vision_receive / vision_target 仍保留定义，因为旧 GimbalApp.cpp 的
//       未调用回调仍引用它们；新链路不再写这两个全局。

// vision打包+发送
static void Vision_DataTransmit(void)
{
    vision_transmit.header = VISION_FRAME_HEADER_TX;
    vision_transmit.detect_color = !USER_JudgeData.self_color; // 打红0 打蓝1
    vision_transmit.mode = 0; //0为打装甲板 1为打符
    vision_transmit.top_yaw = gimbal.top_yaw.totalAngle;
    vision_transmit.pitch = INS.pitch;
    vision_transmit.roll = INS.roll;
    vision_transmit.diff_yaw = (gimbal.top_yawMotor.angle - TOP_YAW_OFFSET) / 8192.0f * 360.0f;
    vision_transmit.diff_pitch = (PITCH_MOTOR_ZERO_POS - gimbal.pitchMotor.para.pos) / (2 * PI) * 360.0f;
    vision_transmit.bullet_speed = USER_JudgeData.initial_speed;
    vision_transmit.robo_status = 0xFF;  /*receive_485.Judge_Data.robo_status;*/
    memcpy(&vision_transmit.AI_Judge_data, &USER_JudgeData, sizeof(Judge_Data_e));

    // see_enemy 取 Vision_Target_Node 发布的瞄准结果（旧实现读 vision_target.vaild）。
    robot::framework::timed_snapshot<robot::io::vision::vision_target_snapshot> target_copy{};
    robot::io::vision::vision_target_mailbox().copy_to(target_copy);
    vision_transmit.see_enemy = target_copy.value.is_valid;

    vision_transmit.end_frame = VISION_FRAME_END_TX;
    CDC_Transmit_HS((uint8_t*)&vision_transmit, sizeof(vision_transmit));
}

extern "C" void OS_VisionCallback(void const * argument)
{
    for(;;)
    {
        // 从环形缓冲取字节、消帧并发布原始帧快照。
        VisionLink_Poll();
        Vision_DataTransmit();
        osDelay(1);
    }
}
