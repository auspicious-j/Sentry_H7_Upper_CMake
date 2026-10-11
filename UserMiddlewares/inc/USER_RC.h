/*
 * ============================================================================
 * 文件：USER_RC.h
 * 作用：遥控器（Remote Controller，下称“遥控器”）接收与解析模块的对外头文件。
 * ----------------------------------------------------------------------------
 * 术语速览（先看这一句，再看代码）：
 *   遥控器接收机：装在车体上、通过串口把遥控器的摇杆/拨杆/按键状态送给主控的模块。
 *   SBUS / DBUS：接收机常用的串口帧格式；一帧 25 字节，内含 16 个“通道”，
 *                每个通道为 11bit（原始值 0~2047），另有 1 个标志字节。
 *   通道(channel)：遥控器上的一个连续控制量（摇杆的某个方向、旋钮等）。
 *   拨杆/开关：非连续控制量，本模块统一用三位状态表示（见下方 SwitchState）。
 * ----------------------------------------------------------------------------
 * 本模块在编译期通过宏 USER_RC_TYPE 三选一，支持三种遥控器：
 *   DR16 ：大疆官方接收机，含 4 摇杆、2 拨杆，另有鼠标、键盘、滚轮；
 *   MC6C ：6 通道航模遥控器（SBUS），仅摇杆 + 左右两个拨杆；
 *   ET08 ：16 通道遥控器（SBUS），摇杆 + SA/SB/SC/SD 四个拨杆。
 * 每种遥控器各有自己的“原始结构体”，解析后再统一映射到 RC_TypeDef
 * （统一接口）；上层逻辑只依赖 RC_TypeDef，无需关心底层是哪一款遥控器。
 * ============================================================================
 */
#ifndef _USER_RC_H_
#define _USER_RC_H_
#include "USER_RC.h"
#include "stdint.h"
#include "usart.h"
#include "main.h"
#include "stdbool.h"
#include <stdint.h>

/* 遥控器类型编号：与 USER_RC_TYPE 宏配合，在编译期选择底层驱动 */
#define USER_RC_TYPE_DR16  1    /* 大疆 DR16 接收机 */
#define USER_RC_TYPE_MC6C  2    /* MC6C 6 通道航模遥控器 */
#define USER_RC_TYPE_ET08  3    /* ET08 16 通道遥控器 */

/*
 * 切换遥控器类型：修改 USER_RC_TYPE 即可（编译期生效，无需改其它代码）。
 * 可选值：USER_RC_TYPE_DR16 / USER_RC_TYPE_MC6C / USER_RC_TYPE_ET08
 */
#ifndef USER_RC_TYPE
#define USER_RC_TYPE USER_RC_TYPE_DR16    /* 未显式定义时，默认使用 DR16 */
#endif

/* 合法性检查：若 USER_RC_TYPE 被写成非法值，编译期直接报错，避免跑错解析分支 */
#if (USER_RC_TYPE != USER_RC_TYPE_DR16) && \
    (USER_RC_TYPE != USER_RC_TYPE_MC6C) && \
    (USER_RC_TYPE != USER_RC_TYPE_ET08)
#error "Invalid USER_RC_TYPE"
#endif

/*
 * 拨杆/开关的三位状态（“上 / 中 / 下”）。
 * 注意取值刻意不是 0/1/2，而是映射成 1/2/3，与接收机原始档位对应，
 * 便于直接与硬件档位比较（例如判断“拨杆向下 == SWITCH_DOWN”）。
 */
typedef enum
{
    SWITCH_UP = 2,      /* 拨杆在上 */
    SWITCH_MID = 3,     /* 拨杆居中（三位开关的中档） */
    SWITCH_DOWN = 1     /* 拨杆在下 */
} SwitchState;


typedef struct
{  /* rocker channel information 摇杆通道信息 */
  int16_t ch1;            /* 通道1：右横，已减去中位 1000 并做死区，范围约 -1000 ~ +1000 */
  int16_t ch2;            /* 通道2：右竖，已减去中位 1000 并做死区 */
  int16_t ch3;            /* 通道3：左横，已减去中位 1000 并做死区 */
    int16_t ch4;          /* 通道4：左竖，已减去中位 1000 并做死区 */
  /* left and right lever information 左右拨杆信息 */
  uint8_t left;           /* 左拨杆当前档位（三位状态，见 SwitchState） */
  uint8_t right;          /* 右拨杆当前档位（三位状态） */
    uint8_t left_last;    /* 左拨杆“上一次”档位，用于检测拨动瞬间（旧状态） */
    uint8_t right_last;   /* 右拨杆“上一次”档位（旧状态） */

    int16_t ch[16];        // ch1 ~ ch16 左竖1 左横3  右竖2 右横0（16 个原始通道；解析后 ch[0..3] 减中位得到 ch1..ch4）

    uint8_t lost;          // 失联标志：非 0 表示接收机报告信号丢失
    uint8_t failsafe;      // 失控保护：非 0 表示接收机已进入失控保护（输出被固定/切断）
} MC6C_RC_t;

typedef struct
{  /* rocker channel information 摇杆通道信息 */
  int16_t ch1;            /* 通道1：右横，减去中位 1024 并去死区 */
  int16_t ch2;            /* 通道2：右竖，减去中位 1024 并去死区 */
  int16_t ch3;            /* 通道3：左横，减去中位 1024 并去死区 */
    int16_t ch4;          /* 通道4：左竖，减去中位 1024 并去死区 */
  /* left and right lever information 四个拨杆信息 */
  uint8_t SA;             /* 拨杆 SA 当前档位（三位状态，见 SwitchState） */
  uint8_t SB;             /* 拨杆 SB 当前档位 */
  uint8_t SC;             /* 拨杆 SC 当前档位 */
  uint8_t SD;             /* 拨杆 SD 当前档位 */

  uint8_t SA_last;        /* SA 上一次档位（旧状态，用于检测拨动瞬间） */
  uint8_t SB_last;        /* SB 上一次档位 */
  uint8_t SC_last;        /* SC 上一次档位 */
  uint8_t SD_last;        /* SD 上一次档位 */

    int16_t ch[16];        // ch1 ~ ch16（16 个原始通道；ch[4..7] 用于解析 SA~SD）

    uint8_t lost;          // 失联标志：非 0 表示接收机报告信号丢失
    uint8_t failsafe;      // 失控保护：非 0 表示接收机已进入失控保护
} ET08_RC_t;


typedef struct
{
  /* rocker channel information 摇杆通道信息（均为原始值减中位 1024 并去死区） */
  int16_t ch1;          /* 通道1：右横，范围约 -1024 ~ +1023 */
  int16_t ch2;          /* 通道2：右竖 */
  int16_t ch3;          /* 通道3：左横 */
  int16_t ch4;          /* 通道4：左竖 */
  /* left and right lever information 左右拨杆信息 */
  uint8_t left;         /* 左拨杆当前档位（三位状态，见 SwitchState） */
  uint8_t right;	//中间是3，上边是1，下边是2 —— 右拨杆当前档位
  uint8_t left_last;    /* 左拨杆上一次档位（旧状态） */
  uint8_t right_last;   /* 右拨杆上一次档位（旧状态） */
  /* mouse movement and button information 鼠标移动量与按键信息 */
  struct
  {
    int16_t x;          /* 鼠标 X 轴增量（相对移动，带符号） */
    int16_t y;          /* 鼠标 Y 轴增量（相对移动，带符号） */
    int16_t z;          /* 鼠标 Z 轴增量（滚轮，带符号） */

    uint8_t l;          /* 鼠标左键：非 0 表示按下 */
    uint8_t r;          /* 鼠标右键：非 0 表示按下 */
  } mouse;
  /* keyboard key information 键盘按键信息 */
  union {
    uint16_t key_code;  /* 16 位键盘位图，整体访问；与下方 bit 共用同一块内存 */
    struct
    {
      uint16_t W : 1;       /* W 键按下 */
      uint16_t S : 1;       /* S 键按下 */
      uint16_t A : 1;       /* A 键按下 */
      uint16_t D : 1;       /* D 键按下 */
      uint16_t SHIFT : 1;   /* Shift 键按下 */
      uint16_t CTRL : 1;    /* Ctrl 键按下 */
      uint16_t Q : 1;       /* Q 键按下 */
      uint16_t E : 1;       /* E 键按下 */
      uint16_t R : 1;       /* R 键按下 */
      uint16_t F : 1;       /* F 键按下 */
      uint16_t G : 1;       /* G 键按下 */
      uint16_t Z : 1;       /* Z 键按下 */
      uint16_t X : 1;       /* X 键按下 */
      uint16_t C : 1;       /* C 键按下 */
      uint16_t V : 1;       /* V 键按下 */
      uint16_t B : 1;       /* B 键按下 */
    } bit;
  } kb;
  int16_t wheel;        /* 滚轮增量（原始值减 1024，带符号） */
}DR16_RC_T;

typedef struct //统一接口：三种遥控器解析后统一映射到本结构，上层只依赖它
{
  int16_t ch1;        /* 摇杆通道1（右横），已减中位并去死区 */
  int16_t ch2;        /* 摇杆通道2（右竖） */
  int16_t ch3;        /* 摇杆通道3（左横） */
  int16_t ch4;        /* 摇杆通道4（左竖） */

  uint8_t lleft; //最左边 两档开关（仅 ET08 用到，对应拨杆 SA）
  uint8_t left; //左 三挡开关 上中下 132 //左 三挡开关（MC6C/DR16 用 left，ET08 用 SB）
  uint8_t right; //右 三挡开关 上中下 132 //右 三挡开关（MC6C/DR16 用 right，ET08 用 SC）
  uint8_t rright;//最右边 两档开关（仅 ET08 用到，对应拨杆 SD）

  uint8_t lleft_last; //开关旧状态：lleft 上一次档位
  uint8_t left_last;  //开关旧状态：left 上一次档位
  uint8_t right_last; //开关旧状态：right 上一次档位
  uint8_t rright_last;//开关旧状态：rright 上一次档位

  struct
  {
    int16_t x;      /* 鼠标 X 轴增量（仅 DR16 有效） */
    int16_t y;      /* 鼠标 Y 轴增量（仅 DR16 有效） */
    int16_t z;      /* 鼠标 Z 轴增量（滚轮，仅 DR16 有效） */

    uint8_t l;      /* 鼠标左键状态（仅 DR16 有效） */
    uint8_t r;      /* 鼠标右键状态（仅 DR16 有效） */
  } mouse;

  union {
    uint16_t key_code;  /* 键盘位图整体访问（仅 DR16 有效）；与 bit 共用内存 */
    struct
    {
      uint16_t W : 1;       /* W 键 */
      uint16_t S : 1;       /* S 键 */
      uint16_t A : 1;       /* A 键 */
      uint16_t D : 1;       /* D 键 */
      uint16_t SHIFT : 1;   /* Shift 键 */
      uint16_t CTRL : 1;    /* Ctrl 键 */
      uint16_t Q : 1;       /* Q 键 */
      uint16_t E : 1;       /* E 键 */
      uint16_t R : 1;       /* R 键 */
      uint16_t F : 1;       /* F 键 */
      uint16_t G : 1;       /* G 键 */
      uint16_t Z : 1;       /* Z 键 */
      uint16_t X : 1;       /* X 键 */
      uint16_t C : 1;       /* C 键 */
      uint16_t V : 1;       /* V 键 */
      uint16_t B : 1;       /* B 键 */
    } bit;
  } kb;

  int16_t wheel;      /* 滚轮增量（DR16 有效；MC6C/ET08 恒为 0） */
  int16_t ch[16];     /* 16 个原始通道（0~2047），供需要原始值的逻辑使用 */

  uint8_t lost;       /* 失联标志：非 0 表示接收机报告信号丢失 */
  uint8_t failsafe;   /* 失控保护：非 0 表示接收机已进入失控保护 */
} RC_TypeDef;

/* 初始化遥控器接收：启动串口空闲中断 + DMA 接收（在 USER_RC.c 中实现） */
void RC_Init(void);

extern DMA_HandleTypeDef hdma_uart5_rx;  /* 串口5 的 RX DMA 句柄（接收遥控器数据用） */
extern uint8_t usart5RxBuf[25]; // 串口5缓冲区（一帧 SBUS/DBUS 为 25 字节）
extern MC6C_RC_t rcInfo_MC6C;    /* MC6C 遥控器解析结果（原始结构体） */
extern DR16_RC_T rcInfo_DR16;    /* DR16 遥控器解析结果（原始结构体） */
extern ET08_RC_t rcInfo_ET08;    /* ET08 遥控器解析结果（原始结构体） */
extern RC_TypeDef rcInfo;        /* 统一接口：上层都读这个 */

#endif
