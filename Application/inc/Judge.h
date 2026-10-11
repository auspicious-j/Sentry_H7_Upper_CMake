/*
 * Judge.h —— 裁判系统(RoboMaster 官方裁判系统)数据解析与访问接口。
 *
 * 【名词解释】
 *  - 裁判系统：比赛场地上的官方主机，通过串口 1(USART1) 以固定协议向机器人下发比赛信息
 *    (比赛状态、血量、热量、弹量、RFID 增益点等)，机器人只接收、不发送。
 *  - 帧(frame)：一次通信的最小数据包，由“帧头 + 命令码 + 数据段 + 帧尾”组成。
 *  - 命令码(CmdID)：帧内标识“这一段数据是什么内容”的编号，例如 0x0001 表示比赛状态。
 *
 * 【帧结构(本文件按此解析)】
 *  偏移 0         : SOF         起始字节，固定 0xA5
 *  偏移 1~2       : DataLength  数据段长度(不含帧头/命令码/帧尾)
 *  偏移 3         : Seq         包序号
 *  偏移 4         : CRC8        帧头校验(仅校验前 5 字节)
 *  偏移 5~6       : CmdID       命令码，小端(低字节在前)
 *  偏移 7 起      : DATA        数据段，长度 = DataLength
 *  末尾 2 字节    : CRC16       整帧校验(帧头+命令码+数据段)
 *
 * 【字节序】数据段内多字节整数/浮点数均为小端序(低字节在前)，与 STM32 一致，
 * 故可直接 memcpy 到结构体，无需额外转换。
 *
 * 【本文件内容】
 *  1) 协议长度常量与偏移定义；
 *  2) 命令码(内容)枚举 CmdID 及其数据长度枚举 JudgeDataLength；
 *  3) 各命令码对应的数据段结构体(ext_xxx_t)，成员含单位/范围；
 *  4) 对外访问函数声明与全局数据 extern 声明。
 *
 * 说明：本文件为纯头文件，只做类型与接口定义，不含任何可执行逻辑。
 */
#ifndef _JUDGEMENT_H_
#define _JUDGEMENT_H_

#include "main.h"
#include "stdbool.h"
#include <stdint.h>

/* 数据有效性判定返回值：由 JUDGE_Read_Data 返回，表示这一帧是否可信 */
#define    JUDGE_DATA_ERROR      0
#define    JUDGE_DATA_CORRECT    1

/* 帧字段固定长度，单位：字节 */
#define    LEN_HEADER    5        //帧头长(SOF 1 + DataLength 2 + Seq 1 + CRC8 1)
#define    LEN_CMDID     2        //命令码长度(2 字节，小端)
#define    LEN_TAIL      2	      //帧尾CRC16

//起始字节,协议固定为0xA5。每帧第一个字节，用于定位一帧的起点
#define    JUDGE_FRAME_HEADER         (0xA5)

//发送缓冲区长度，单位：字节(单个发送帧最大 64 字节)
#define JUDGE_MAX_TX_LENGTH     64
//接收缓冲区长度，单位：字节(一次接收可能含多帧，预留 1536 字节)
#define JUDGE_MAX_RX_LENGTH     1536
//发送队列长度，单位：帧(待发送帧最多缓存 25 个)
#define JUDGE_QUEUE_SIZE 25

//机器人阵营颜色(由自身 ID 是否大于 10 判定，见 JUDGE_GetSelfColor)
typedef enum
{
    RobotColor_Red = 0,   //红方(自身 ID 1~7)
    RobotColor_Blue = 1   //蓝方(自身 ID 11~17)
}RobotColor;

//帧内各字段相对帧首字节的偏移量，单位：字节
typedef enum
{
    FRAME_HEADER         = 0,  //帧头起始偏移
    CMD_ID               = 5,  //命令码偏移(帧头 5 字节之后)
    DATA                 = 7,  //数据段偏移(帧头 + 命令码 2 字节之后)
}JudgeFrameOffset;

//5 字节帧头内部各字段的偏移位置(相对帧头起始字节)
typedef enum
{
    SOF          = 0,//起始位，固定 0xA5
    DATA_LENGTH  = 1,//数据段长度(2 字节)，据此计算整帧长度
    SEQ          = 3,//包序号，用于识别丢包/乱序
    CRC8         = 4 //帧头 CRC8 校验字节(仅校验前 5 字节)

}FrameHeaderOffset;

/***************命令码ID********************/

/*

    ID: 0x0001  Byte:  11    比赛状态数据       		    发送频率 1Hz
    ID: 0x0002  Byte:  1    比赛结果数据         	    比赛结束后发送
    ID: 0x0003  Byte:  16    比赛机器人血量数据   	    1Hz发送    //2026.3.14修改 16byte
    ID: 0x0101  Byte:  4    场地事件数据   			    事件改变后发送
    ID: 0x0104    Byte:     3	    裁判警告信息				//2026.3.14修改 3byte
    ID: 0x0105    Byte:     3	    飞镖发射口倒计时		//2026.3.14修改 3byte
    ID: 0X0201  Byte: 13    机器人状态数据        	    10Hz		//2026.3.14修改 13byte
    ID: 0X0202  Byte: 14    实时功率热量数据   		    50Hz
    ID: 0x0203  Byte: 12    机器人位置数据               10Hz
    ID: 0x0204  Byte:  8    机器人增益数据               增益状态改变后发送
    ID: 0x0206  Byte:  1    伤害状态数据           	    伤害发生后发送
    ID: 0x0207  Byte:  7    实时射击数据           	    子弹发射后发送
    ID: 0x0208  Byte:  8    子弹剩余发射数					//2026.3.14修改 6byte
    ID: 0x0209  Byte:  5    机器人RFID状态					//2026.3.14修改 5byte
    ID: 0x020A  Byte:  6    飞镖机器人客户端指令数据
    ID: 0x020D Byte:  6	    哨兵机器人当前状态
    ID: 0x0301  Byte:  118    机器人间交互数据               发送方触发发送,10Hz  //2026.3.14修改 16byte
*/


//命令码(CmdID)枚举：JUDGE_Read_Data 依据它决定把数据段写入哪个结构体
/*
	ID: 0x0001  Byte:  11    比赛状态数据       			发送频率 1Hz
	ID: 0x0002  Byte:  1    比赛结果数据         		比赛结束后发送
	ID: 0x0003  Byte:  16    比赛机器人血量数据   		1Hz发送    //2026.3.14修改 16byte
	ID: 0x0101  Byte:  4    场地事件数据   				事件改变后发送
	ID: 0x0104	Byte: 	3		裁判警告信息				//2026.3.14修改 3byte
	ID: 0x0105	Byte: 	3		飞镖发射口倒计时		//2026.3.14修改 3byte
	ID: 0X0201  Byte: 13    机器人状态数据        		10Hz		//2026.3.14修改 13byte
	ID: 0X0202  Byte: 14    实时功率热量数据   			50Hz
	ID: 0x0203  Byte: 12    机器人位置数据           	10Hz
	ID: 0x0204  Byte:  8    机器人增益数据           	增益状态改变后发送
	ID: 0x0206  Byte:  1    伤害状态数据           		伤害发生后发送
	ID: 0x0207  Byte:  7    实时射击数据           		子弹发射后发送
	ID: 0x0208  Byte:  8    子弹剩余发射数					//2026.3.14修改 6byte
	ID: 0x0209  Byte:  5    机器人RFID状态					//2026.3.14修改 5byte
	ID: 0x020A  Byte:  6    飞镖机器人客户端指令数据
	ID: 0x020D Byte:  6		哨兵机器人当前状态
	ID: 0x0301  Byte:  118    机器人间交互数据           	发送方触发发送,10Hz  //2026.3.14修改 16byte
*/
typedef enum
{
    ID_game_state       				= 0x0001,//比赛状态数据
    ID_game_result 	   					= 0x0002,//比赛结果数据
    ID_game_robot_HP      			= 0x0003,//比赛机器人血量数据
    ID_event_data  							= 0x0101,//场地事件数据 *
    ID_referee_warning					= 0x0104,//裁判警告信息
    ID_dart_remaining_time			= 0x0105,//飞镖发射口倒计时
    ID_game_robot_state    			= 0x0201,//机器人状态数据  *
    ID_power_heat_data    			= 0x0202,//实时功率热量数据
    ID_game_robot_pos        		= 0x0203,//机器人位置数据  *
    ID_buff_musk								= 0x0204,//机器人增益数据//
    ID_robot_hurt								= 0x0206,//伤害状态数据
    ID_shoot_data								= 0x0207,//实时射击数据
    ID_bullet_remaining					= 0x0208,//子弹剩余发射数
    ID_rfid_status							= 0x0209,//机器人RFID状态  *
    ID_sentry_status          	= 0x020D,//哨兵机器人当前状态数据
} CmdID;



//各命令码对应的数据段长度(单位：字节)，须与官方协议一致，用作 memcpy 的拷贝长度
typedef enum
{
    LEN_game_state       					= 11,	//0x0001 比赛状态数据
    LEN_game_result       				= 1,	//0x0002 比赛结果数据
    LEN_game_robot_HP							= 16,	//0x0003 比赛机器人血量数据
    LEN_event_data  							= 4,	//0x0101 场地事件数据
    LEN_referee_warning						= 3,	//0x0104 裁判警告信息
    LEN_dart_remaining_time				= 3,	//0x0105 飞镖发射口倒计时
    LEN_game_robot_state    			= 13,	//0x0201 机器人状态数据
    LEN_power_heat_data   				= 14,	//0x0202 实时功率热量数据
    LEN_game_robot_pos        		= 12,	//0x0203 机器人位置数据
    LEN_buff_musk        					= 8,	//0x0204 机器人增益数据
    LEN_robot_hurt        				= 1,	//0x0206 伤害状态数据
    LEN_shoot_data       					= 7,	//0x0207 实时射击数据
    LEN_bullet_remaining					= 6,	//0x0208 子弹剩余发射数
    LEN_rfid_status								= 5,	//0x0209 机器人RFID状态
    LEN_sentry_status   					= 6,  //0x020D 哨兵机器人当前状态数据
} JudgeDataLength;


////机器人交互信息
//typedef struct __attribute__((packed))

//{
//    xFrameHeader   						    txFrameHeader;//帧头
//    uint16_t							    CmdID;//命令码
//    ext_student_interactive_header_data_t   dataFrameHeader;//数据段头结构
//    robot_interactive_data_t  	 		    interactData;//数据段
//    uint16_t		 					    FrameTail;//帧尾
//}ext_CommunatianData_t;


/* 自定义帧头 */
//	xFrameHeader   							txFrameHeader;//帧头
//	uint16_t								CmdID;//命令码
//	ext_student_interactive_header_data_t   dataFrameHeader;//数据段头结构
//	robot_interactive_data_t  	 			interactData;//数据段
//	uint16_t		 						FrameTail;//帧尾
typedef struct __attribute__((packed)) //裁判系统帧头(共 5 字节)，packed 保证按 1 字节对齐、无填充
{
    uint8_t  SOF;        //起始字节，固定 0xA5(JUDGE_FRAME_HEADER)
    uint16_t DataLength; //数据段长度，单位：字节(不含帧头/命令码/帧尾)，小端
    uint8_t  Seq;        //包序号，用于识别丢包/乱序
    uint8_t  CRC8;       //帧头 CRC8 校验字节(仅校验帧头前 5 字节)
} xFrameHeader;

/* 机器人间交互数据(0x0301)数据段头结构：标明内容 ID、发送者与接收者 */
typedef struct __attribute__((packed))
{
    uint16_t data_cmd_id;   //内容 ID(队伍自定义，范围 0x0200~0x02FF)
    uint16_t send_ID;       //发送者机器人 ID
    uint16_t receiver_ID;   //接收者机器人 ID
    uint32_t 	    sentry_cmd;
}sentry_cmd_t;



/* ID: 0x0001  Byte:  11    比赛状态数据(1Hz) */
typedef struct __attribute__((packed))
{
    uint8_t game_type : 4;       //比赛类型(4bit)：1 超级对抗赛，2 单项赛，3 通用赛制
    uint8_t game_progress : 4;   //比赛阶段(4bit)：0 未开始 1 准备 2 自检 3 五秒倒计时 4 比赛中 5 结算
    uint16_t stage_remain_time;  //当前阶段剩余时间，单位：秒
    uint64_t SyncTimeStamp;      //裁判系统时间戳，单位：毫秒(与裁判系统对时用)
} ext_game_status_t;

/* ID: 0x0002  Byte:  1    比赛结果数据(比赛结束后发送) */
typedef struct __attribute__((packed))
{
    uint8_t winner;   //获胜方：0 平局，1 红方胜，2 蓝方胜
} ext_game_result_t;

/* ID: 0x0003  Byte:  32    比赛机器人血量数据 */   //2026.3.14 改动
typedef struct __attribute__((packed))
{
    uint16_t ally_1_robot_HP;   //己方 1 号机器人(英雄)血量
    uint16_t ally_2_robot_HP;   //己方 2 号机器人(工程)血量
    uint16_t ally_3_robot_HP;   //己方 3 号机器人(步兵)血量
    uint16_t ally_4_robot_HP;   //己方 4 号机器人(步兵)血量
    uint16_t reserved;          //保留位(对应 5 号机位)
    uint16_t ally_7_robot_HP;   //己方 7 号机器人(哨兵)血量
    uint16_t ally_outpost_HP;   //己方前哨站血量
    uint16_t ally_base_HP;      //己方基地血量
} ext_game_robot_HP_t;

/* ID: 0x0101  Byte:  4    场地事件数据 */
typedef struct __attribute__((packed)) 					//2026.3.14修改
{
    uint32_t event_data;  //场地事件位域，各 bit 含义见下方注释(bit0~bit29)

	//以下是队内自定义结构体
	//0：未占领/未激活  1：已占领/已激活
	/*bit 0-2：
	    bit 0：己方与兑换区不重叠的补给区占领状态，1为已占领
	    bit 1：己方与兑换区重叠的补给区占领状态，1为已占领
	    bit 2：己方补给区的占领状态，1为已占领（仅 RMUL 适用）
	    bit 3-6：己方能量机关状态
	    bit 3-4：己方小能量机关的激活状态，0为未激活，1为已激活，2为正在激活
	    bit 5-6：己方大能量机关的激活状态，0为未激活，1为已激活，2为正在激活
	    bit 7-8：己方中央高地的占领状态，1为被己方占领，2为被对方占领
	    bit 9-10：己方梯形高地的占领状态，1为已占领
	    bit 11-19：对方飞镖最后一次击中己方前哨站或基地的时间（0-420，开局默认为0）
	    bit 20-22：对方飞镖最后一次击中己方前哨站或基地的具体目标，开局默认为0，
							 1为击中前哨站，2为击中基地固定目标，3为击中基地随机
							 固定目标，4为击中基地随机移动目标 5为击中基地末端移动目标
	    bit 23-24：中心增益点的占领状态，0为未被占领，1为被己方占领，2
							 为被对方占领，3为被双方占领。（仅RMUL适用）
	    bit 25-26：己方堡垒增益点的占领状态，0为未被占领，1为被己方占
							 领，2为被对方占领，3为被双方占领。
	    bit 27-28：己方前哨站增益点的占领状态， 0 为未被占领， 1 为被己方
							 占领， 2 为被对方占领
	    bit 29：己方基地增益点的占领状态， 1 为已占领							 */

//    uint8_t supply_area_state : 3;
//    uint8_t own_buff_state : 4;
//    uint8_t own_central_land_state : 2;
//    uint8_t own_trapezoidal_land_state : 2;
//    uint16_t darts_hitted_time : 9;
//    uint8_t darts_hitted_target : 3;
//    uint8_t center_gain_state : 2;
//    uint8_t own_fortress_state :2;
//    uint8_t own_outpost_state : 2;   // bit27-28 己方前哨站增益点状态
//    uint8_t own_base_state : 1;      // bit29 己方基地增益点状态
//
//    uint32_t reserved : 2;

/*bit 0-2：
		bit 0：己方与兑换区不重叠的补给区占领状态，1为已占领
		bit 1：己方与兑换区重叠的补给区占领状态，1为已占领
		bit 2：己方补给区的占领状态，1为已占领（仅 RMUL 适用）
		bit 3-6：己方能量机关状态
		bit 3-4：己方小能量机关的激活状态，0为未激活，1为已激活，2为正在激活
		bit 5-6：己方大能量机关的激活状态，0为未激活，1为已激活，2为正在激活
		bit 7-8：己方中央高地的占领状态，1为被己方占领，2为被对方占领
		bit 9-10：己方梯形高地的占领状态，1为已占领
		bit 11-19：对方飞镖最后一次击中己方前哨站或基地的时间（0-420，开局默认为0）
		bit 20-22：对方飞镖最后一次击中己方前哨站或基地的具体目标，开局默认为0，
							 1为击中前哨站，2为击中基地固定目标，3为击中基地随机
							 固定目标，4为击中基地随机移动目标 5为击中基地末端移动目标
		bit 23-24：中心增益点的占领状态，0为未被占领，1为被己方占领，2
							 为被对方占领，3为被双方占领。（仅RMUL适用）
		bit 25-26：己方堡垒增益点的占领状态，0为未被占领，1为被己方占
							 领，2为被对方占领，3为被双方占领。
		bit 27-28：己方前哨站增益点的占领状态， 0 为未被占领， 1 为被己方
							 占领， 2 为被对方占领
		bit 29：己方基地增益点的占领状态， 1 为已占领							 */
//	uint8_t supply_area_state : 3;
//	uint8_t own_buff_state : 4;
//	uint8_t own_central_land_state : 2;
//	uint8_t own_trapezoidal_land_state : 2;
//	uint16_t darts_hitted_time : 9;
//	uint8_t darts_hitted_target : 3;
//	uint8_t center_gain_state : 2;
//	uint8_t own_fortress_state :2;
//	uint8_t own_outpost_state : 2;   // bit27-28 己方前哨站增益点状态
//	uint8_t own_base_state : 1;      // bit29 己方基地增益点状态
//	uint32_t reserved : 2;
} ext_event_data_t;



/* ID: 0x104    Byte: 3    裁判警告信息 */
typedef struct __attribute__((packed))
{
  uint8_t level;              //警告等级：1 黄牌，2 红牌，3 判负
  uint8_t offending_robot_id; //违规机器人 ID
  uint8_t count;              //违规累计次数
} ext_referee_warning_t;

/* ID: 0x105    Byte: 3    飞镖发射口倒计时 */
typedef struct __attribute__((packed))
{
  uint8_t dart_remaining_time; //飞镖发射口倒计时，单位：秒
  uint16_t dart_info;          //飞镖状态位域(当前目标等，详见官方协议)
}ext_dart_info_t;

/* ID: 0X0201  Byte: 13    机器人状态数据(10Hz) */
typedef struct __attribute__((packed))
{
    uint8_t robot_id;                        //机器人 ID(1~7 红方，11~17 蓝方)
  uint8_t robot_level;                       //机器人等级
  uint16_t current_HP;                       //当前血量
  uint16_t maximum_HP;                       //最大血量
  uint16_t shooter_barrel_cooling_value;     //枪管冷却速度，单位：热量/秒
  uint16_t shooter_barrel_heat_limit;        //枪管热量上限(热量限制)
  uint16_t chassis_power_limit;              //底盘功率上限，单位：W
  uint8_t power_management_gimbal_output : 1;//云台电源输出状态：1 允许输出
  uint8_t power_management_chassis_output : 1;//底盘电源输出状态：1 允许输出
  uint8_t power_management_shooter_output : 1;//发射机构电源输出状态：1 允许输出
} ext_game_robot_status_t;


/* ID: 0X0202  Byte: 16    实时功率热量数据(50Hz) */
typedef struct __attribute__((packed))
{
    uint16_t reserved_1;                    //保留(旧版为底盘电压，已废弃)
    uint16_t reserved_2;                    //保留(旧版为底盘电流，已废弃)
    float reserved_3;                       //保留(旧版为底盘功率，已废弃)
    uint16_t chassis_power_buffer;          //底盘缓冲能量(能量环剩余额度)，单位：J
    uint16_t shooter_id1_17mm_cooling_heat; //17mm 枪管(枪口 1)当前热量
    uint16_t shooter_id1_42mm_cooling_heat; //42mm 枪管(枪口 1)当前热量
} ext_power_heat_data_t;


/* ID: 0x0203  Byte: 12    机器人位置数据(10Hz) */
typedef struct __attribute__((packed))
{
  float x;     //机器人 X 坐标，单位：m(以场地中心为原点)
  float y;     //机器人 Y 坐标，单位：m
  float angle; //机器人朝向角，单位：度(0~360)
} ext_game_robot_pos_t;


/* ID: 0x0204  Byte:  8    机器人增益数据(增益状态改变后发送) */
typedef struct __attribute__((packed))

{
  uint8_t recovery_buff;     //回血增益：恢复血量百分比
  uint16_t cooling_buff;     //冷却增益：冷却速度提升值
  uint8_t defence_buff;      //防御增益：减免伤害百分比
  uint8_t vulnerability_buff;//负防御(易伤)：增伤百分比
  uint16_t attack_buff;      //攻击增益：增伤百分比
    uint8_t remaining_energy;//剩余能量百分比(0~100；特殊值 0x80 表示能量 >30%，见 Judge.c)
} ext_buff_musk_t;


/* ID: 0x0206  Byte:  2    伤害状态数据(受到伤害后发送) */
typedef struct __attribute__((packed))

{
    uint8_t armor_id : 4;  //被击中的装甲板 ID(0 表示被撞击扣血)
    uint8_t hurt_type : 4; //扣血原因：0 弹丸 1 撞击 3 前哨站被击毁 4 基地被击毁 5 装甲模块离线/超时
} ext_robot_hurt_t;


/* ID: 0x0207  Byte:  7    实时射击数据(子弹发射后发送) */
typedef struct __attribute__((packed))

{
    uint8_t bullet_type;          //弹丸类型：1 17mm，2 42mm
  uint8_t shooter_number;         //发射机构编号：1 枪口 1，2 枪口 2
  uint8_t launching_frequency;    //发射射频，单位：Hz
  float initial_speed;            //弹丸初速度，单位：m/s
} ext_shoot_data_t;

/* ID: 0x0208  Byte:  8   子弹剩余发射数 */
typedef struct __attribute__((packed))

{
    uint16_t projectile_allowance_17mm;      //17mm 弹丸剩余允许发弹量，单位：发
  uint16_t projectile_allowance_42mm;        //42mm 弹丸剩余允许发弹量，单位：发
  uint16_t remaining_gold_coin;              //剩余金币，单位：个
  uint16_t projectile_allowance_fortress;    //堡垒剩余允许发弹量，单位：发
} ext_bullet_remaining_t;

/* ID: 0x0209  Byte:  5    机器人RFID状态 */ //2026.3.14修改
typedef struct __attribute__((packed))

{
    uint32_t rfid_status;    //RFID 状态位 1：各 bit 表示是否检测到对应增益点(如 bit17 堡垒、bit18 对方前哨站等)
    uint8_t rfid_status_2;   //RFID 状态位 2：扩展 bit(高 8 位)，补充 rfid_status 未覆盖的检测点
} ext_rfid_status_t;

/* ID: 0x020A  Byte:  6    飞镖机器人客户端指令数据 */
typedef struct __attribute__((packed))

{
    uint8_t dart_launch_opening_status;   //飞镖发射口状态：1 关闭，2 正在开启，3 已开启
  uint8_t reserved;                        //保留位
  uint16_t target_change_time;             //飞镖目标切换时间，单位：秒
  uint16_t latest_launch_cmd_time;         //最近一次飞镖发射指令时间，单位：秒
} ext_dart_client_cmd_t;

/* ID: 0x020D  Byte:  6    哨兵机器人当前状态数据 */
typedef struct __attribute__((packed))

{
    uint32_t sentry_info;    //哨兵状态位：含已兑换发弹量、远程兑换次数、复活/兑换金币数等
    uint16_t sentry_info_2;  //哨兵状态扩展位：脱战状态、可兑换发弹量、姿态、能量机关可激活等
}sentry_info_t;

/*

    交互数据，包括一个统一的数据段头结构，
    包含了内容 ID，发送者以及接受者的 ID 和内容数据段，
    整个交互数据的包总共长最大为 128 个字节，
    减去 frame_header,cmd_id,frame_tail 以及数据段头结构的 6 个字节，
    故而发送的内容数据段最大为 113。
    整个交互数据 0x0301 的包上行频率为 10Hz。

    机器人 ID：
    1，英雄(红)；
    2，工程(红)；
    3/4/5，步兵(红)；
    6，空中(红)；
    7，哨兵(红)；
    11，英雄(蓝)；
    12，工程(蓝)；
    13/14/15，步兵(蓝)；
    16，空中(蓝)；
    17，哨兵(蓝)。
    客户端 ID：
    0x0101 为英雄操作手客户端( 红) ；
    0x0102 ，工程操作手客户端 ((红 )；
    0x0103/0x0104/0x0105，步兵操作手客户端(红)；
    0x0106，空中操作手客户端((红)；
    0x0111，英雄操作手客户端(蓝)；
    0x0112，工程操作手客户端(蓝)；
    0x0113/0x0114/0x0115，操作手客户端步兵(蓝)；
    0x0116，空中操作手客户端(蓝)。
*/
/* 交互数据接收信息：0x0301  */
/*

	交互数据，包括一个统一的数据段头结构，
	包含了内容 ID，发送者以及接受者的 ID 和内容数据段，
	整个交互数据的包总共长最大为 128 个字节，
	减去 frame_header,cmd_id,frame_tail 以及数据段头结构的 6 个字节，
	故而发送的内容数据段最大为 113。
	整个交互数据 0x0301 的包上行频率为 10Hz。
	机器人 ID：
	1，英雄(红)；
	2，工程(红)；
	3/4/5，步兵(红)；
	6，空中(红)；
	7，哨兵(红)；
	11，英雄(蓝)；
	12，工程(蓝)；
	13/14/15，步兵(蓝)；
	16，空中(蓝)；
	17，哨兵(蓝)。
	客户端 ID：
	0x0101 为英雄操作手客户端( 红) ；
	0x0102 ，工程操作手客户端 ((红 )；
	0x0103/0x0104/0x0105，步兵操作手客户端(红)；
	0x0106，空中操作手客户端((红)；
	0x0111，英雄操作手客户端(蓝)；
	0x0112，工程操作手客户端(蓝)；
	0x0113/0x0114/0x0115，操作手客户端步兵(蓝)；
	0x0116，空中操作手客户端(蓝)。
*/
typedef struct __attribute__((packed))
	//2026.3.14修改
{
    uint16_t data_cmd_id;  //数据内容 ID(0x0200~0x02FF，队伍自定义)
    uint16_t send_ID;      //发送者 ID
    uint16_t receiver_ID;  //接收者 ID
} ext_student_interactive_header_data_t;
/*
    学生机器人间通信 cmd_id 0x0301，内容 ID:0x0200~0x02FF
    交互数据 机器人间通信：0x0301。
    发送频率：上限 10Hz

    字节偏移量     大小     说明 		    备注
    0 		    2 	    数据的内容 ID     0x0200~0x02FF
									    可以在以上 ID 段选取，具体 ID 含义由参赛队自定义

    2 		    2 	    发送者的 ID     需要校验发送者的 ID 正确性，

    4 		    2 	    接收者的 ID     需要校验接收者的 ID 正确性，
									    例如不能发送到敌对机器人的ID

    6 		    n 	    数据段 		    n 需要小于 113

*/
/*
	学生机器人间通信 cmd_id 0x0301，内容 ID:0x0200~0x02FF
	交互数据 机器人间通信：0x0301。
	发送频率：上限 10Hz
	字节偏移量 	大小 	说明 			备注
	0 			2 		数据的内容 ID 	0x0200~0x02FF
										可以在以上 ID 段选取，具体 ID 含义由参赛队自定义

	2 			2 		发送者的 ID 	需要校验发送者的 ID 正确性，

	4 			2 		接收者的 ID 	需要校验接收者的 ID 正确性，
										例如不能发送到敌对机器人的ID

	6 			n 		数据段 			n 需要小于 113
*/
typedef struct __attribute__((packed))

{
    uint8_t data[106]; //数据段(内容数据)，协议上限 113 字节，此处预留 106 字节
} robot_interactive_data_t;



//机器人交互信息
typedef struct __attribute__((packed))

{
    xFrameHeader   						    txFrameHeader;//帧头
    uint16_t							    CmdID;//命令码
    ext_student_interactive_header_data_t   dataFrameHeader;//数据段头结构
    robot_interactive_data_t  	 		    interactData;//数据段
    uint16_t		 					    FrameTail;//帧尾
}ext_CommunatianData_t;



//裁判系统发送数据帧(软件发送队列中的一个元素)
typedef struct
{
    uint8_t data[JUDGE_MAX_TX_LENGTH]; //待发送的整帧字节(含帧头/命令码/数据/帧尾)
    uint16_t frameLength;              //该帧实际字节数(发送长度)
}JudgeTxFrame;

/* 汇总后的裁判数据(供其他模块读取)：把分散在多个命令码里的关键信息集中成一份，单位见各成员注释 */
typedef struct __attribute__((packed))
{
    uint8_t game_progress; // 当前比赛状态 0:未开始比赛 1:准备阶段 2:自检阶段 3:五秒倒计时 4:比赛中 5:比赛结算中
    uint16_t remain_time;  // 比赛剩余时间 单位:s
    uint16_t current_hp;   // 当前血量
    uint16_t projectile;   // 弹仓还剩多少蛋
    uint32_t sentry_info; 	//bit0-10：除远程兑换外，哨兵机器人成功兑换的允许发弹量，开局为0，在哨兵机器人成功兑换一定允许发弹量后，该值将变为哨兵机器人成功兑换的允许发弹量值
							//bit11-14：哨兵机器人成功远程兑换允许发弹量的次数，开局为0，在哨兵机器人成功远程兑换允许发弹量后，该值将变为哨兵机器人成功远程兑换允许发弹量的次数
							//bit15-18：哨兵机器人成功远程兑换血量的次数，开局为0，在哨兵机器人成功远程兑换血量后，该值将变为哨兵机器人成功远程兑换血量的次数
							//bit19：哨兵机器人当前是否可以确认免费复活，可以确认免费复活时值为1，否则为0
							//bit20：哨兵机器人当前是否可以兑换立即复活，可以兑换立即复活时值为1，否则为0
							//bit21-30：哨兵机器人当前若兑换立即复活需要花费的金币数。
							//bit31 保留
    uint16_t sentry_info_2;	// bit0：哨兵当前是否处于脱战状态，处于脱战状态时为1，否则为0
							// bit1-11：队伍17mm允许发弹量的剩余可兑换数
							// bit12-13:哨兵当前姿态，1为进攻姿态，2为防御姿态，3为移动姿态
							// bit14：己方能量机关是否能够进入正在激活状态，1为当前可激活
							// bit 15：保留位

    uint8_t sentry_info_3;  // bit 0: 装甲板是否被攻击 0:否 1:是
							// bit 1: RFID 是否检测到堡垒 0:否 1:是
							// bit 2: RFID 是否检测到补给区(与兑换站不重叠) 0:否 1:是
							// bit 3: RFID 是否检测到补给区(与兑换站重叠) 0:否 1:是
							// bit 4: 当前剩余能量值是否小于30% 0:否 1:是
							// bit 5：RFID 是否检测到对方前哨站增益点
							// bit 6：RFID 是否检测到对方堡垒增益点
    uint16_t ally_outpost_hp;  // 己方前哨站血量
    uint16_t ally_base_hp;     // 己方基地血量
/***********以上为发给ai的**************/
    uint16_t shooter_barrel_cooling_value;  // 冷却速度
    uint16_t shooter_barrel_heat_limit; 		//热量限制
    uint16_t shooter_id1_17mm_cooling_heat; //17mm枪口热量
    uint8_t power_management_gimbal_output : 1;   //云台电源输出是否允许(1 允许)
    uint8_t power_management_chassis_output : 1;  //底盘电源输出是否允许(1 允许)
    uint8_t power_management_shooter_output : 1;  //发射机构电源输出是否允许(1 允许)
    float initial_speed;										//弹速
    uint8_t self_color;                                          //自身颜色(0 红方，1 蓝方)
}JudgeData_t;

/* 哨兵机器人指令结构：本机决策后用于向裁判系统申报/兑换的意图(单位见各成员注释) */
typedef struct __attribute__((packed))
{
    uint8_t sentry_mode; //1为进攻 2为防守 3为移动 默认为3
    uint8_t energy_activation; //0为不激活 1为激活小符 2为激活大符 激活小还是大和比赛开始时间有关
    uint16_t buy_projectile; //哨兵要买多少发弹 开局为0 修改后烧饼在补血点就能兑换 只能单增 如0->100买100发 100->101买1发 依此类推
    uint8_t buy_life; 	//0为不买活 1为买活
    uint8_t remote_buy_blood;	//0到1为买一次 1到2为买一次 依此类推
    uint8_t remote_buy_bullet; //0到1为买一次 1到2为买一次 依此类推
}USER_SentryCmd_t; //哨兵机器人指令数据

/****************函数声明***************/
void JUDGE_Init(void);                        //初始化：启动 USART1 的 DMA 空闲接收
bool JUDGE_Read_Data(uint8_t *ReadFromUsart);//解析一帧裁判数据(含帧头/CRC 校验)，成功返回 true
RobotColor JUDGE_GetSelfColor(void);          //获取己方颜色
uint8_t JUDGE_GetSelfID(void);                //获取自身机器人 ID
uint16_t JUDGE_GetClientID(void);             //获取己方操作手客户端 ID(0x100 + 机器人 ID)
bool JUDGE_IsValid(void);                     //裁判数据当前是否有效(Judge_Data_TF)
void JUDGE_GetPosition(float *x,float *y);    //获取机器人坐标，单位：m
uint8_t JUDGE_GetChassisPowerLimit(void);     //获取底盘功率上限，单位：W
bool JUDGE_GetShooterOutputState(void);       //获取发射机构电源输出是否允许
bool JUDGE_GetGimbalOutputState(void);        //获取云台电源输出是否允许
uint16_t JUDGE_GetHeatLimit(void);            //获取枪管热量上限
uint16_t JUDGE_GetShootSpeedLimit(void);      //获取射速上限(恒返回 25)
uint16_t JUDGE_GetPowerBuffer(void);          //获取底盘缓冲能量，单位：J
int16_t JUDGE_GetRemainHeat(void);            //获取剩余可用热量(热量上限 - 当前热量)
uint8_t HP_deduction_reason(void);            //获取扣血原因(hurt_type)
uint16_t JUDGE_GetHP(void);                   //获取当前血量
uint16_t JUDGE_GetCoolingValue(void);         //获取枪管冷却速度


//任务回调
//串口6中断回调
void Task_Judge_Callback(void);   //裁判系统发送任务回调
//掉线回调
void Judge_UartLostCallback(void); //裁判系统串口掉线回调：重新挂起 DMA 接收

void Judge_Receive(void);          //从接收缓冲区解析一帧并清空缓冲区(供接收中断/任务调用)
/****************外部引用***************/
extern ext_game_robot_pos_t		    GameRobotPos;    //机器人位置
extern ext_shoot_data_t			    ShootData;       //实时射击数据
extern ext_game_robot_status_t	    GameRobotState;  //机器人状态(血量/热量上限/功率上限等)
extern JudgeData_t				    USER_JudgeData;  //汇总后的裁判数据(对外使用)
extern USER_SentryCmd_t			    USER_SentryCmd;  //哨兵机器人指令

#endif //头文件
