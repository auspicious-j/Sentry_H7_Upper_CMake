/*
 * ============================================================================
 * 文件：USER_RC.c
 * 作用：遥控器接收与解析实现（详见 USER_RC.h 的模块说明）。
 * ----------------------------------------------------------------------------
 * 数据流概览：
 *   串口5(DMA) 收到一帧 25 字节 → HAL_UARTEx_RxEventCallback(空闲中断回调)
 *   → 按 USER_RC_TYPE 调用对应解析函数(如 DR16_ParseSBUS)解出原始结构体
 *   → 调用对应的 *_ToUnified 映射到统一结构体 rcInfo
 *   → 上层任务/回调(Task_RC_Callback、其它业务)读 rcInfo 使用。
 * ----------------------------------------------------------------------------
 * 术语速览：
 *   帧头帧尾：一帧数据的首/尾固定字节，用来判断本帧是否合法（校验帧边界）。
 *   空闲中断(IDLE)：串口在一段静默后触发的中断，配合 DMA 用来“收完一整帧再处理”。
 *   死区(DeadZone)：摇杆回中后仍有微小抖动，设定一个阈值把抖动当 0 处理。
 *   失联/失控保护：接收机与发射机断开时置位标志，供上层做停机保护。
 * ============================================================================
 */
#include "USER_RC.h"
#include "stm32h7xx_hal.h"
#include "usart.h"
#include "cmsis_os.h"
#include "UserFreertos.h"
#include "USER_Moto.h"
#include "USER_B2B.h"


uint8_t usart5RxBuf[25]; // 串口5缓冲区（DMA 把接收机发来的一整帧存到这里）
MC6C_RC_t rcInfo_MC6C = {0};//三个遥控器各自的接口（各存对应型号的解析结果，初始化全 0）
DR16_RC_T rcInfo_DR16 = {0};
ET08_RC_t rcInfo_ET08 = {0};

RC_TypeDef rcInfo = {0};//统一接口（上层只读这个；初始化全 0，避免上电时读到脏值）

int rc_true_flag;        // 遥控器“无数据”计数：每次收到一帧清零；一直累加到阈值说明串口没收到数据

extern DMA_HandleTypeDef hdma_usart2_rx;  // 串口2 的 RX DMA 句柄（本文件未直接使用，供外部配置）
extern DMA_HandleTypeDef hdma_uart5_rx;   // 串口5 的 RX DMA 句柄


/*
 * 初始化遥控器接收。
 * 参数：无。
 * 返回：无。
 * 说明：启动串口5 的“空闲中断 + DMA”接收，之后每收到一帧会进入
 *       HAL_UARTEx_RxEventCallback 回调；并关闭 DMA 半传输中断(HT)，
 *       因为我们只在“一整帧收完”时处理，不需要半帧中断。
 */
void RC_Init()
{
    HAL_UARTEx_ReceiveToIdle_DMA(&huart5, usart5RxBuf, sizeof(usart5RxBuf));  // 启动 DMA 接收，缓冲区满或串口空闲即触发回调
    __HAL_DMA_DISABLE_IT(&hdma_uart5_rx, DMA_IT_HT);                          // 关闭半传输中断，避免一帧还没收完就被打断
}

/*
 * 把“拨杆通道原始值”换算成三位状态（上/中/下）。
 * 参数：ch —— 拨杆对应的通道原始值（未经减中位处理，范围 0~2047）。
 * 返回：SwitchState —— SWITCH_UP / SWITCH_MID / SWITCH_DOWN。
 * 说明：>1500 视为上，<500 视为下，中间其余值视为中（三位开关的中档）。
 */
SwitchState SBUS_SwitchState(int16_t ch)
{
    if (ch > 1500)       return SWITCH_UP;     // 高电平区间 → 上
    else if (ch < 500) return SWITCH_DOWN;     // 低电平区间 → 下
    else                return SWITCH_MID;     // 其余 → 中
}
/*
 * 摇杆死区处理：抑制回中后的微抖，避免静止时被误判为操作。
 * 参数：x    —— 输入值（已减中位的摇杆值）。
 *       zone —— 死区半宽（阈值）；|x|<zone 时输出 0。
 * 返回：去掉死区后的值（死区内返回 0.0f，否则原样返回 x）。
 */
float DeadZone(float x, float zone)
{
    if (x > -zone && x < zone)     // 落在 (-zone, zone) 区间
        return 0.0f;               // 视为静止
    return x;                      // 超出死区，原样输出
}
/*
 * 解析 MC6C 遥控器的一帧 SBUS 数据。
 * 参数：buf —— 指向 25 字节接收缓冲（帧头(buf[0])/通道数据/标志字节/帧尾(buf[24])）。
 *       rc  —— 输出：解析结果写入 MC6C_RC_t。
 * 返回：无（帧头帧尾不合法时直接返回，不改动 rc）。
 */
void MC6C_ParseSBUS(uint8_t *buf, MC6C_RC_t *rc)
{
    /* 帧头帧尾校验：首字节须为 0x0F、末字节须为 0x00，否则视为无效帧丢弃 */
    if (buf[0] != 0x0F || buf[24] != 0x00)
        return;
    /* 16 通道，每通道 11bit：SBUS 把 16 个 11bit 通道紧凑打包在 buf[1..22] 中，
       下面按位拼接还原每个通道，& 0x07FF 保证只保留低 11 位 */
    rc->ch[0]  = (buf[1]       | buf[2]  << 8) & 0x07FF;                  // 通道0
    rc->ch[1]  = (buf[2] >> 3  | buf[3]  << 5) & 0x07FF;                  // 通道1
    rc->ch[2]  = (buf[3] >> 6  | buf[4]  << 2 | buf[5]  << 10) & 0x07FF;  // 通道2
    rc->ch[3]  = (buf[5] >> 1  | buf[6]  << 7) & 0x07FF;                  // 通道3
    rc->ch[4]  = (buf[6] >> 4  | buf[7]  << 4) & 0x07FF;                  // 通道4（本机用作左拨杆）
    rc->ch[5]  = (buf[7] >> 7  | buf[8]  << 1 | buf[9]  << 9) & 0x07FF;   // 通道5（本机用作右拨杆）
    rc->ch[6]  = (buf[9] >> 2  | buf[10] << 6) & 0x07FF;                  // 通道6
    rc->ch[7]  = (buf[10] >> 5 | buf[11] << 3) & 0x07FF;                  // 通道7

    rc->ch[8]  = (buf[12]      | buf[13] << 8) & 0x07FF;                  // 通道8
    rc->ch[9]  = (buf[13] >> 3 | buf[14] << 5) & 0x07FF;                  // 通道9
    rc->ch[10] = (buf[14] >> 6 | buf[15] << 2 | buf[16] << 10) & 0x07FF;  // 通道10
    rc->ch[11] = (buf[16] >> 1 | buf[17] << 7) & 0x07FF;                  // 通道11
    rc->ch[12] = (buf[17] >> 4 | buf[18] << 4) & 0x07FF;                  // 通道12
    rc->ch[13] = (buf[18] >> 7 | buf[19] << 1 | buf[20] << 9) & 0x07FF;   // 通道13
    rc->ch[14] = (buf[20] >> 2 | buf[21] << 6) & 0x07FF;                  // 通道14
    rc->ch[15] = (buf[21] >> 5 | buf[22] << 3) & 0x07FF;                  // 通道15

    /* flag 字节：bit2=失联(lost)，bit3=失控保护(failsafe) */
    uint8_t flag = buf[23];                     // 第 24 字节为状态标志
    rc->lost     = (flag & (1 << 2)) ? 1 : 0;   // 取 bit2 → 失联标志
    rc->failsafe = (flag & (1 << 3)) ? 1 : 0;   // 取 bit3 → 失控保护标志

    rc->left_last = rc->left;                   // 保存旧档位，供上层检测“拨动瞬间”
    rc->right_last = rc->right;

    rc->ch1 = rc->ch[0] - 1000;                 // 通道0 减中位 1000 → 有符号摇杆值
    rc->ch2 = rc->ch[1] - 1000;                 // 通道1 减中位 1000
    rc->ch3 = rc->ch[2] - 1000;                 // 通道2 减中位 1000
    rc->ch4	= rc->ch[3] - 1000;                 // 通道3 减中位 1000
    rc->ch1 = DeadZone(rc->ch1,20);             // 去死区（阈值 20）
    rc->ch2 = DeadZone(rc->ch2,20);
    rc->ch3 = DeadZone(rc->ch3,20);
    rc->ch4 = DeadZone(rc->ch4,20);
    rc->left  = SBUS_SwitchState(rc->ch[4]);    // 通道4 → 左拨杆三位状态（用未减中位的原始值判断）
    rc->right = SBUS_SwitchState(rc->ch[5]);    // 通道5 → 右拨杆三位状态
}

/*
 * 解析 DR16 遥控器的一帧 DBUS 数据（大疆接收机格式）。
 * 参数：buff —— 指向 18 字节有效数据（18 字节后为帧尾，本函数不校验帧头帧尾）。
 *       rc   —— 输出：解析结果写入 DR16_RC_T。
 * 返回：无。
 */
void DR16_ParseSBUS(uint8_t* buff,DR16_RC_T *rc)
{
    rc->ch1 = (buff[0] | buff[1] << 8) & 0x07FF;   // 取低 11 位还原通道0
    rc->ch1 -= 1024;		//右横
    rc->ch2 = (buff[1] >> 3 | buff[2] << 5) & 0x07FF;   // 通道1
    rc->ch2 -= 1024;		//右竖
    rc->ch3 = (buff[2] >> 6 | buff[3] << 2 | buff[4] << 10) & 0x07FF;   // 通道2
    rc->ch3 -= 1024;		//左横
    rc->ch4 = (buff[4] >> 1 | buff[5] << 7) & 0x07FF;   // 通道3
    rc->ch4 -= 1024;		//左竖

  /* prevent remote control zero deviation 抑制回中偏差（死区阈值 50） */
    rc->ch1 = DeadZone(rc->ch1,50);
    rc->ch2 = DeadZone(rc->ch2,50); //设静止死区
    rc->ch3 = DeadZone(rc->ch3,50);
    rc->ch4 = DeadZone(rc->ch4,50); //设静止死区

    rc->left_last = rc->left;                      // 保存旧拨杆档位
    rc->right_last = rc->right;

    rc->left = ((buff[5] >> 4) & 0x000C) >> 2;  //sw1   中间是3，上边是1，下边是2 —— 左拨杆取高位两 bit
    rc->right = (buff[5] >> 4) & 0x0003;        //sw2   右拨杆取低位两 bit

    rc->mouse.x = buff[6] | (buff[7] << 8); // x axis 鼠标 X 轴增量（有符号）
    rc->mouse.y = buff[8] | (buff[9] << 8);        // 鼠标 Y 轴增量
    rc->mouse.z = buff[10] | (buff[11] << 8);      // 鼠标 Z 轴增量

    rc->mouse.l = buff[12];                        // 鼠标左键
    rc->mouse.r = buff[13];                        // 鼠标右键

    rc->kb.key_code = buff[14] | buff[15] << 8; // key borad code 键盘位图（16 位）
    rc->wheel = (buff[16] | buff[17] << 8) - 1024;  // 滚轮增量，减中位 1024
}


/*
 * 解析 ET08 遥控器的一帧 SBUS 数据。
 * 参数：buff —— 指向 25 字节接收缓冲（帧头/通道数据/标志字节/帧尾）。
 *       rc   —— 输出：解析结果写入 ET08_RC_t。
 * 返回：无（帧头帧尾不合法时直接返回）。
 */
void ET08_ParseSBUS(uint8_t *buff, ET08_RC_t *rc)
{
    /* 帧头帧尾校验：首字节须为 0x0F、末字节须为 0x00 */
    if (buff[0] != 0x0F || buff[24] != 0x00)
        return;
    /* 16 通道，每通道 11bit：按位拼接还原 buff[1..22] 里的 16 个通道，保留低 11 位 */
    rc->ch[0]  = (buff[1]       | buff[2]  << 8) & 0x07FF;                  // 通道0
    rc->ch[1]  = (buff[2] >> 3  | buff[3]  << 5) & 0x07FF;                  // 通道1
    rc->ch[2]  = (buff[3] >> 6  | buff[4]  << 2 | buff[5]  << 10) & 0x07FF;  // 通道2
    rc->ch[3]  = (buff[5] >> 1  | buff[6]  << 7) & 0x07FF;                  // 通道3
    rc->ch[4]  = (buff[6] >> 4  | buff[7]  << 4) & 0x07FF;                  // 通道4（本机用作拨杆 SA）
    rc->ch[5]  = (buff[7] >> 7  | buff[8]  << 1 | buff[9]  << 9) & 0x07FF;   // 通道5（本机用作拨杆 SB）
    rc->ch[6]  = (buff[9] >> 2  | buff[10] << 6) & 0x07FF;                  // 通道6（本机用作拨杆 SC）
    rc->ch[7]  = (buff[10] >> 5 | buff[11] << 3) & 0x07FF;                  // 通道7（本机用作拨杆 SD）

    rc->ch[8]  = (buff[12]      | buff[13] << 8) & 0x07FF;                  // 通道8
    rc->ch[9]  = (buff[13] >> 3 | buff[14] << 5) & 0x07FF;                  // 通道9
    rc->ch[10] = (buff[14] >> 6 | buff[15] << 2 | buff[16] << 10) & 0x07FF;  // 通道10
    rc->ch[11] = (buff[16] >> 1 | buff[17] << 7) & 0x07FF;                  // 通道11
    rc->ch[12] = (buff[17] >> 4 | buff[18] << 4) & 0x07FF;                  // 通道12
    rc->ch[13] = (buff[18] >> 7 | buff[19] << 1 | buff[20] << 9) & 0x07FF;   // 通道13
    rc->ch[14] = (buff[20] >> 2 | buff[21] << 6) & 0x07FF;                  // 通道14
    rc->ch[15] = (buff[21] >> 5 | buff[22] << 3) & 0x07FF;                  // 通道15

    uint8_t flag = buff[23];                    // 标志字节
    rc->lost     = (flag & (1 << 2)) ? 1 : 0;   // bit2 → 失联标志
    rc->failsafe = (flag & (1 << 3)) ? 1 : 0;   // bit3 → 失控保护标志

    rc->ch1 = rc->ch[0] - 1024;//右横  通道0 减中位 1024
    rc->ch2 = rc->ch[1] - 1024;//右竖  通道1 减中位 1024
    rc->ch3 = rc->ch[2] - 1024;//左横  通道2 减中位 1024
    rc->ch4 = rc->ch[3] - 1024;//左竖  通道3 减中位 1024
  /* prevent remote control zero deviation 抑制回中偏差（死区阈值 10） */
      rc->ch1 = DeadZone(rc->ch1,10);
    rc->ch2 = DeadZone(rc->ch2,10);
    rc->ch3 = DeadZone(rc->ch3,10);
    rc->ch4 = DeadZone(rc->ch4,10); //设静止死区

    rc->SA_last = rc->SA;                       // 保存四个拨杆的旧档位
    rc->SB_last = rc->SB;
    rc->SC_last = rc->SC;
    rc->SD_last = rc->SD;

    rc->SA = SBUS_SwitchState(rc->ch[4]);       // 通道4 → 拨杆 SA 三位状态
    rc->SB = SBUS_SwitchState(rc->ch[5]);       // 通道5 → 拨杆 SB 三位状态
    rc->SC = SBUS_SwitchState(rc->ch[6]);       // 通道6 → 拨杆 SC 三位状态
    rc->SD = SBUS_SwitchState(rc->ch[7]);       // 通道7 → 拨杆 SD 三位状态

}

//统一接口
/*
 * 把 MC6C 解析结果映射到统一结构体。
 * 参数：rc_MC6C —— 输入：MC6C 原始解析结果。
 *       rc      —— 输出：统一接口 rcInfo。
 * 返回：无。MC6C 没有鼠标/键盘/滚轮，故 wheel 置 0、mouse/kb 不动。
 */
void MC6C_ToUnified(MC6C_RC_t *rc_MC6C, RC_TypeDef *rc)
{
    rc->ch1 = rc_MC6C->ch1;                     // 4 个摇杆通道直接搬运
    rc->ch2 = rc_MC6C->ch2;
    rc->ch3 = rc_MC6C->ch3;
    rc->ch4	= rc_MC6C->ch4;

    rc->left = rc_MC6C->left;                   // 左右拨杆档位
    rc->right = rc_MC6C->right;
    rc->left_last = rc_MC6C->left_last;         // 拨杆旧档位
    rc->right_last = rc_MC6C->right_last;
    rc->wheel = 0;                              // MC6C 无滚轮，固定 0
    rc->failsafe = rc_MC6C->failsafe;           // 失控保护标志透传
    rc->lost = rc_MC6C->lost;                   // 失联标志透传
}

/*
 * 把 DR16 解析结果映射到统一结构体（本型号带鼠标/键盘/滚轮，全部搬运）。
 * 参数：rc_DR16 —— 输入：DR16 原始解析结果。
 *       rc      —— 输出：统一接口 rcInfo。
 * 返回：无。
 */
void DR16_ToUnified(DR16_RC_T *rc_DR16, RC_TypeDef *rc)
{
    rc->ch1 = rc_DR16->ch1;                     // 4 个摇杆通道
    rc->ch2 = rc_DR16->ch2;
    rc->ch3 = rc_DR16->ch3;
    rc->ch4	= rc_DR16->ch4;

    rc->left = rc_DR16->left;                   // 左右拨杆档位
    rc->right = rc_DR16->right;
    rc->left_last = rc_DR16->left_last;         // 拨杆旧档位
    rc->right_last = rc_DR16->right_last;

    rc->mouse.x = rc_DR16->mouse.x;             // 鼠标三轴
    rc->mouse.y = rc_DR16->mouse.y;
    rc->mouse.z = rc_DR16->mouse.z;
    rc->mouse.l = rc_DR16->mouse.l;             // 鼠标左右键
    rc->mouse.r = rc_DR16->mouse.r;
    rc->kb.key_code = rc_DR16->kb.key_code;     // 键盘位图

    rc->wheel = rc_DR16->wheel;                 // 滚轮增量
}

/*
 * 把 ET08 解析结果映射到统一结构体。
 * 参数：rc_ET08 —— 输入：ET08 原始解析结果。
 *       rc      —— 输出：统一接口 rcInfo。
 * 返回：无。
 * 说明：ET08 有四个拨杆 SA/SB/SC/SD，分别对应统一接口的
 *       lleft / left / right / rright（最左、左、右、最右）。
 */
void ET08_ToUnified(ET08_RC_t *rc_ET08, RC_TypeDef *rc)
{
    rc->ch1 = rc_ET08->ch1;                     // 4 个摇杆通道
    rc->ch2 = rc_ET08->ch2;
    rc->ch3 = rc_ET08->ch3;
    rc->ch4	= rc_ET08->ch4;

    rc->lleft = rc_ET08->SA;                    // 拨杆 SA → 最左两档开关
    rc->left = rc_ET08->SB;                     // 拨杆 SB → 左三档开关
    rc->right = rc_ET08->SC;                    // 拨杆 SC → 右三档开关
    rc->rright = rc_ET08->SD;                   // 拨杆 SD → 最右两档开关

    rc->lleft_last = rc_ET08->SA_last;          // 对应拨杆旧档位
    rc->left_last = rc_ET08->SB_last;
    rc->right_last = rc_ET08->SC_last;
    rc->rright_last = rc_ET08->SD_last;

    rc->wheel = 0;                              // ET08 无滚轮，固定 0
    rc->failsafe = rc_ET08->failsafe;           // 失控保护标志透传
    rc->lost = rc_ET08->lost;                   // 失联标志透传
}

// 串口5空闲中断回调
/*
 * 串口“空闲中断”回调：DMA 收满或串口空闲时触发，用于处理一帧接收完成。
 * 参数：huart —— 触发中断的串口句柄（用它与 huart2/huart5 比较，区分来源）。
 *       Size  —— 本次已接收的字节数（本函数未使用，仅表示一帧的长度）。
 * 返回：无。
 * 说明：这里区分串口2（B2B 通信）与串口5（遥控器接收）；串口5 收到一帧后，
 *       按编译期选的 USER_RC_TYPE 调用对应解析+映射，然后重置清零计数并重启接收。
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart == &huart2)               // 串口2：板间(B2B)通信
	{
	    B2B_Receive();                  // 交板间通信模块处理
	}
    if (huart == &huart5)               // 串口5：遥控器数据
	{
		#if (USER_RC_TYPE == USER_RC_TYPE_MC6C)
		    MC6C_ParseSBUS(usart5RxBuf, &rcInfo_MC6C);      // 解析 MC6C 原始帧
		    MC6C_ToUnified(&rcInfo_MC6C, &rcInfo);          // 映射到统一接口
		#elif (USER_RC_TYPE == USER_RC_TYPE_DR16)
		    DR16_ParseSBUS(usart5RxBuf,&rcInfo_DR16);       // 解析 DR16 原始帧
		    DR16_ToUnified(&rcInfo_DR16, &rcInfo);          // 映射到统一接口
		#else
		    ET08_ParseSBUS(usart5RxBuf, &rcInfo_ET08);      // 解析 ET08 原始帧
		    ET08_ToUnified(&rcInfo_ET08, &rcInfo);          // 映射到统一接口
		#endif
	    rc_true_flag = 0;                                   // 收到数据，清零“无数据”计数
	    HAL_UARTEx_ReceiveToIdle_DMA(&huart5, usart5RxBuf, sizeof(usart5RxBuf));  // 重启下一帧接收
	    __HAL_DMA_DISABLE_IT(&hdma_uart5_rx, DMA_IT_HT);                          // 再次关闭半传输中断
	}
}

/*
 * 遥控器特殊情况处理（当前仅实现“急停”）。
 * 参数：无。
 * 返回：无。
 * 说明：当右拨杆拨到“下”档时执行急停：先切换电机模式、再连续发送零电流以
 *       关断两路 CAN 上的电机、置停机标志、上报板间通信，最后唤醒错误任务
 *       独占 CPU（“饿死”其它任务）以保证安全。
 */
void Task_RC_Callback()
{
	/**********特殊情况处理*********************/
    if (rcInfo.right == 2) // 遥控器右拨码开关向下，急停
	{
        disable_motor_mode(&hfdcan2,0x01,MIT_MODE);              // CAN2 电机切到 MIT 模式准备关断
	    HAL_Delay(10);                                           // 短暂延时等待生效
	    USER_CAN_SetMotorCurrent(&hfdcan1, 0x1FF, 0, 0, 0, 0);  // CAN1 第一组发送零电流
	    HAL_Delay(10);
	    USER_CAN_SetMotorCurrent(&hfdcan1, 0x200, 0, 0, 0, 0); // 关断电机（CAN1 第二组零电流）
	    HAL_Delay(10);
	    USER_CAN_SetMotorCurrent(&hfdcan2,0x1FF,0,0,0,0);//关断电机（CAN2 零电流）
	    STOPFLAG = 1;                                            // 置停机标志，通知其它逻辑已急停
        B2B_Transmit();                                          // 通过板间通信上报状态
	    osThreadResume(ErrorTaskHandle); // 恢复错误任务 饿死其他任务 —— 让错误任务抢占运行以确保安全
	}
}

/************************freertos任务****************************/
/*
 * 遥控器任务（FreeRTOS 线程体）：周期性巡检遥控器状态并做异常恢复。
 * 参数：argument —— 任务参数（此处未使用）。
 * 返回：无（死循环，永不返回）。
 * 说明：每次循环把 rc_true_flag 自增（若期间收到帧会被回调清零）；
 *       若累计到 100 说明长时间没收到数据，则手动重启串口接收；
 *       随后调用 Task_RC_Callback 处理急停等特殊情况，最后 osDelay(15ms)。
 */
void OS_RcCallback(void const *argument)
{

    for (;;)
	{
	    rc_true_flag++;                     // “无数据”计数自增（收到帧会被中断回调清 0）
	    if (rc_true_flag >= 100)//长时间未接收到遥控器 手动重新接收
		{
		    HAL_UARTEx_ReceiveToIdle_DMA(&huart5, usart5RxBuf, sizeof(usart5RxBuf));  // 重启 DMA 接收
		    __HAL_DMA_DISABLE_IT(&hdma_uart5_rx, DMA_IT_HT);                          // 关闭半传输中断
		}
	    Task_RC_Callback();                 // 处理急停等特殊情况
	    osDelay(15);                        // 15ms 周期
	}
}
