#ifndef _SHOOTER_H_
#define _SHOOTER_H_

#include "Moto.h"
#include "USER_Moto.h"

#include <stdbool.h>
#include <stdint.h>

// 发射模式
enum
{
    IDLE = 0,           // 空闲
    TRIGGER,            // 拨弹
    TRIGGER_REVERSE,    // 反转
    TRIGGER_CONTINUE,   // 连续拨弹
    TRIGGER_DOUBLE      // 双发
};//! 计划给他取个名，之后再说

// 拨弹电机和摩擦轮电机结构体
typedef struct
{
    uint8_t command;        // 发射模式

    int8_t idToTrigger;     // 需要拨弹的符文id
    int8_t triggeringId;    // 正在拨弹的符文id
    uint32_t triggeringHitTick; // 正在拨弹的符文命中时长
    uint32_t hitAtTick[5];      // 每个符文的命中时刻，单位ms
	
    DJI_Motor_t triggerMotor;   // 拨弹电机
    DJI_Motor_t fricMotor[2];   // 摩擦轮电机
	
    bool fricOpen;	  // 摩擦轮开启标志  0开1关

    int16_t fricSpd;	    // 摩擦轮速度
    float ave_bullet_speed; //? 无调用
    float bullet_speed;     // 实测子弹速度

    float distance;		        // 测距 用来补偿
    float last_bullet_speed;    // 上一次子弹速度
    uint8_t box;		        //? 无写无调用
	
    struct
	{
	    uint16_t judgeCnt;          // 拨弹电机堵转判定计数器
        uint16_t reverseCnt;        // 反转脱困阶段的计数器
	    uint16_t fric_judgeCnt;     // 摩擦轮堵转判定计数器
        uint16_t fric_reverseCnt;   // 摩擦轮反转脱困计数器
	    bool state; // 0正常 1堵转
	} block; // 堵转处理相关数据

} Shooter;

extern Shooter shooter;

void Shooter_Init(void);

#endif
