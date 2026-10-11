/**
************************************************************************
* @file    BMI088driver.h
* @brief   BMI088 六轴 IMU 驱动头文件。
*          （术语）IMU = Inertial Measurement Unit，惯性测量单元：
*          本芯片把 3 轴加速度计 + 3 轴陀螺仪集成在一起。
*          加速度计测三轴线加速度（单位 g，1 g 约 9.8 m/s^2）；
*          陀螺仪测三轴角速度（单位 deg/s，即度每秒）。
*          驱动通过 SPI 读写芯片寄存器，完成量程配置、数据/温度读取
*          以及数据就绪（DRDY）中断的使能。
*          重要：本文件里的量程选择与刻度系数必须与 .c 中实际写入芯片
*          的量程保持一致，否则换算出的物理量会错误。
************************************************************************
**/
#ifndef BMI088DRIVER_H
#define BMI088DRIVER_H

#include "stdint.h"
#include "main.h"

// 温度换算：实际摄氏度 = 原始值 A* 系数 + 偏移（见 .c 的 BMI088_read 末尾）
#define BMI088_TEMP_FACTOR 0.125f    // 温度刻度系数：每个 LSB 代表 0.125 摄氏度
#define BMI088_TEMP_OFFSET 23.0f     // 温度偏移量（摄氏度），用于零点补偿

// 初始化时需要写入并回读校验的寄存器条目数（对应 .c 中的配置表行数）
#define BMI088_WRITE_ACCEL_REG_NUM 6   // 加速度计需配置的寄存器项数
#define BMI088_WRITE_GYRO_REG_NUM 6    // 陀螺仪需配置的寄存器项数

// 中断状态字节里“数据就绪”各标志所在的位号（读出状态后按位判断）
#define BMI088_GYRO_DATA_READY_BIT 0         // 位0：陀螺仪数据就绪
#define BMI088_ACCEL_DATA_READY_BIT 1        // 位1：加速度计数据就绪
#define BMI088_ACCEL_TEMP_DATA_READY_BIT 2   // 位2：加速度计温度数据就绪

#define BMI088_LONG_DELAY_TIME 80        // 软复位后的长延时（单位 ms），等待芯片内部自检
#define BMI088_COM_WAIT_SENSOR_TIME 150  // 每次 SPI 收发后等待传感器响应的时间（单位 us）


// I2C 从机地址（左移 1 位是给读/写位预留占位）；本工程走 SPI，这里仅为兼容保留
#define BMI088_ACCEL_IIC_ADDRESSE (0x18 << 1)   // 加速度计 I2C 地址 0x18
#define BMI088_GYRO_IIC_ADDRESSE (0x68 << 1)    // 陀螺仪 I2C 地址 0x68

// 加速度计量程选择：四选一（取消注释切换，其余保持注释）；当前为 ±3g。
// 量程越大可测范围越大，但分辨率越低；更换量程后必须同步修改下方的刻度系数。
#define BMI088_ACCEL_RANGE_3G
//#define BMI088_ACCEL_RANGE_6G
//#define BMI088_ACCEL_RANGE_12G
//#define BMI088_ACCEL_RANGE_24G

// 陀螺仪量程选择：五选一（取消注释切换，其余保持注释）；当前为 ±2000 deg/s。
// 更换量程后必须同步修改下方的刻度系数。
#define BMI088_GYRO_RANGE_2000
//#define BMI088_GYRO_RANGE_1000
//#define BMI088_GYRO_RANGE_500
//#define BMI088_GYRO_RANGE_250
//#define BMI088_GYRO_RANGE_125


// 加速度计刻度系数（把 16 位原始值换算为 g）：数值 = 满量程(g) / 32768
#define BMI088_ACCEL_3G_SEN 0.0008974358974f    // ±3g ：每 LSB 约 0.000897 g
#define BMI088_ACCEL_6G_SEN 0.00179443359375f    // ±6g ：每 LSB 约 0.001794 g
#define BMI088_ACCEL_12G_SEN 0.0035888671875f    // ±12g：每 LSB 约 0.003589 g
#define BMI088_ACCEL_24G_SEN 0.007177734375f     // ±24g：每 LSB 约 0.007178 g


// 陀螺仪刻度系数（把 16 位原始值换算为 deg/s）：数值 = 满量程(deg/s) / 32768
#define BMI088_GYRO_2000_SEN 0.00106526443603169529841533860381f   // ±2000 deg/s：每 LSB 约 0.001065 deg/s
#define BMI088_GYRO_1000_SEN 0.00053263221801584764920766930190693f // ±1000 deg/s：每 LSB 约 0.000533 deg/s
#define BMI088_GYRO_500_SEN 0.00026631610900792382460383465095346f  // ±500  deg/s：每 LSB 约 0.000266 deg/s
#define BMI088_GYRO_250_SEN 0.00013315805450396191230191732547673f  // ±250  deg/s：每 LSB 约 0.000133 deg/s
#define BMI088_GYRO_125_SEN 0.000066579027251980956150958662738366f // ±125  deg/s：每 LSB 约 0.0000666 deg/s


// 传感器原始数据结构（packed 紧凑排布，和寄存器读出字节一一对应）
typedef struct __attribute__((packed))
{
    uint8_t status;      // 状态字节：各“数据就绪”标志位（见 *_READY_BIT 宏）
    int16_t accel[3];    // 三轴加速度计原始值（有符号 16 位，单位 LSB，需 A* 刻度系数转 g）
    int16_t temp;        // 温度原始值（12 位有效，需在 BMI088_read 中换算成摄氏度）
    int16_t gyro[3];     // 三轴陀螺仪原始值（有符号 16 位，单位 LSB，需 A* 刻度系数转 deg/s）
} bmi088_raw_data_t;

// 换算后的物理量数据结构（供上层算法直接使用）
typedef struct BMI088_REAL_DATA
{
    uint8_t status;    // 状态字节（同原始数据，常用于判断数据是否就绪）
    float accel[3];    // 三轴加速度，单位 g（已经乘以刻度系数）
    float temp;        // 温度，单位摄氏度
    float gyro[3];     // 三轴角速度，单位 deg/s（已经乘以刻度系数）
    float time;        // 采样时间戳，单位秒（由上层填充，驱动本身不写）
} bmi088_real_data_t;


// 初始化返回的错误码。0x01~0x0D 对应某个寄存器写入/回读校验失败；
// 0x80/0x40 为自检失败；0xFF 表示未检测到传感器。多个错误按位或累加返回。
enum
{
    BMI088_NO_ERROR = 0x00,                    // 无错误
    BMI088_ACC_PWR_CTRL_ERROR = 0x01,          // 加速度计电源控制寄存器配置失败
    BMI088_ACC_PWR_CONF_ERROR = 0x02,          // 加速度计电源模式寄存器配置失败
    BMI088_ACC_CONF_ERROR = 0x03,              // 加速度计输出数据率/带宽寄存器配置失败
    BMI088_ACC_SELF_TEST_ERROR = 0x04,         // 加速度计自检寄存器配置失败
    BMI088_ACC_RANGE_ERROR = 0x05,             // 加速度计量程寄存器配置失败
    BMI088_INT1_IO_CTRL_ERROR = 0x06,          // 加速度计 INT1 引脚电气属性配置失败
    BMI088_INT_MAP_DATA_ERROR = 0x07,          // 加速度计中断映射寄存器配置失败
    BMI088_GYRO_RANGE_ERROR = 0x08,            // 陀螺仪量程寄存器配置失败
    BMI088_GYRO_BANDWIDTH_ERROR = 0x09,        // 陀螺仪带宽寄存器配置失败
    BMI088_GYRO_LPM1_ERROR = 0x0A,             // 陀螺仪电源模式寄存器配置失败
    BMI088_GYRO_CTRL_ERROR = 0x0B,             // 陀螺仪数据就绪使能寄存器配置失败
    BMI088_GYRO_INT3_INT4_IO_CONF_ERROR = 0x0C,// 陀螺仪 INT3/INT4 引脚电气属性配置失败
    BMI088_GYRO_INT3_INT4_IO_MAP_ERROR = 0x0D, // 陀螺仪中断映射寄存器配置失败

    BMI088_SELF_TEST_ACCEL_ERROR = 0x80,       // 加速度计自检未通过
    BMI088_SELF_TEST_GYRO_ERROR = 0x40,        // 陀螺仪自检未通过
    BMI088_NO_SENSOR = 0xFF,                   // 未检测到传感器（CHIP_ID 不匹配）
};





// BMI088 总初始化：初始化 GPIO/SPI，再分别初始化加速度计与陀螺仪。
// 返回：按位或累加的错误码，BMI088_NO_ERROR(0) 表示全部成功。
extern uint8_t BMI088_init(void);
// 仅初始化加速度计：通信自检 → 软复位 → 写配置并回读校验。
// 返回：0 成功，非 0 为对应错误码（见上方 enum）。
extern uint8_t bmi088_accel_init(void);
// 仅初始化陀螺仪：通信自检 → 软复位 → 写配置并回读校验。
// 返回：0 成功，非 0 为对应错误码（见上方 enum）。
extern uint8_t bmi088_gyro_init(void);

// 读取一组完整数据（阻塞式 SPI 读）。
// 参数：gyro     - 输出，三轴角速度 [deg/s]
//       accel    - 输出，三轴加速度 [g]
//       temperate- 输出，温度 [摄氏度]（通过指针返回）
// 返回：void
extern void BMI088_read(float gyro[3], float accel[3], float *temperate);



#endif
