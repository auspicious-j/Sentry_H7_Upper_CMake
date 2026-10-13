#ifndef _SHOOTER_H_
#define _SHOOTER_H_

#include "Moto.h"
#include "USER_Moto.h"

#include <stdbool.h>
#include <stdint.h>

enum
{
    IDLE = 0,
    TRIGGER,
    TRIGGER_REVERSE,
    TRIGGER_CONTINUE,
    TRIGGER_DOUBLE
};

typedef struct
{
    uint8_t command; // 后workstate

    int8_t idToTrigger;// 视觉传来的符id，我这边打完之后回答视觉这个id的符可以屏蔽了
    int8_t triggeringId;// 当前触发的符id
    uint32_t triggeringHitTick;// 当前准备发射的预期命中时长
    uint32_t hitAtTick[5];// 预计的具体命中时刻
	
    DJI_Motor_t triggerMotor; // 拨弹电机和摩擦轮电机
    DJI_Motor_t fricMotor[2];
	
    bool fricOpen;	  // 摩擦轮开启标志  0开1关

    int16_t fricSpd;	  // 摩擦轮速度
    float ave_bullet_speed;
    float bullet_speed;

    float distance;		 // 测距 用来补偿
    float last_bullet_speed;
    uint8_t box;
	
    struct
	{
	    uint16_t judgeCnt, reverseCnt; // 堵转判定计数器,反转计数器
	    uint16_t fric_judgeCnt, fric_reverseCnt;
	    bool state;
	} block; // 堵转处理相关数据

} Shooter;

extern Shooter shooter;

void Shooter_Init(void);

#endif
