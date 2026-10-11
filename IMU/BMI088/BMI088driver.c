/**
************************************************************************
* @file    BMI088driver.c
* @brief   BMI088 六轴 IMU 驱动实现（SPI 读写）。
*          （术语）IMU = 惯性测量单元，含 3 轴加速度计 + 3 轴陀螺仪。
*          本文件负责：SPI 底层收发、量程/带宽等寄存器配置、数据与温度读取，
*          并把 16 位原始值换算成物理量（加速度 g、角速度 deg/s、温度 摄氏度）。
*          说明：本驱动不含零偏(bias)校准，零偏标定通常由上层算法或中间层完成。
************************************************************************
**/
#include "BMI088driver.h"
#include "BMI088reg.h"
#include "BMI088Middleware.h"


// 加速度计刻度系数（LSB→g）：默认取 ±3g 量程的系数；改量程需同步改此初值
float BMI088_ACCEL_SEN = BMI088_ACCEL_3G_SEN;
// 陀螺仪刻度系数（LSB→deg/s）：默认取 ±2000 量程的系数
float BMI088_GYRO_SEN = BMI088_GYRO_2000_SEN;



#if defined(BMI088_USE_SPI)
/**
************************************************************************
* @brief:          BMI088_accel_write_single_reg(reg, data)
* @param:       reg - 寄存器地址
*               data - 写入的数据
* @retval:         void
* @details:        通过BMI088加速度计的SPI总线写入单个寄存器的宏定义
************************************************************************
**/
/**
************************************************************************
* @brief:      	BMI088_accel_write_single_reg(reg, data)
* @param:       reg - 寄存器地址
*               data - 写入的数据
* @retval:     	void
* @details:    	通过BMI088加速度计的SPI总线写入单个寄存器的宏定义
************************************************************************
**/
// 加速度计 SPI 单寄存器写：片选拉低 → 写寄存器地址与数据 → 片选拉高（每行末尾反斜杠为续行符）
#define BMI088_accel_write_single_reg(reg, data) \
    {                                            \
        BMI088_ACCEL_NS_L();                     \
        BMI088_write_single_reg((reg), (data));  \
        BMI088_ACCEL_NS_H();                     \
    }
/**
************************************************************************
* @brief:          BMI088_accel_read_single_reg(reg, data)
* @param:       reg - 寄存器地址
*               data - 读取的寄存器数据
* @retval:         void
* @details:        通过BMI088加速度计的SPI总线读取单个寄存器的宏定义
************************************************************************
**/
/**
************************************************************************
* @brief:      	BMI088_accel_read_single_reg(reg, data)
* @param:       reg - 寄存器地址
*               data - 读取的寄存器数据
* @retval:     	void
* @details:    	通过BMI088加速度计的SPI总线读取单个寄存器的宏定义
************************************************************************
**/
// 加速度计 SPI 单寄存器读：片选拉低 → 发读命令(reg|0x80)+dummy → 回读 1 字节 → 片选拉高
#define BMI088_accel_read_single_reg(reg, data) \
    {                                           \
        BMI088_ACCEL_NS_L();                    \
        BMI088_read_write_byte((reg) | 0x80);   \
        BMI088_read_write_byte(0x55);           \
        (data) = BMI088_read_write_byte(0x55);  \
        BMI088_ACCEL_NS_H();                    \
    }
/**
************************************************************************
* @brief:          BMI088_accel_read_muli_reg(reg, data, len)
* @param:       reg - 起始寄存器地址
*               data - 存储读取数据的缓冲区
*               len - 要读取的字节数
* @retval:         void
* @details:        通过BMI088加速度计的SPI总线连续读取多个寄存器的宏定义
************************************************************************
**/
/**
************************************************************************
* @brief:      	BMI088_accel_read_muli_reg(reg, data, len)
* @param:       reg - 起始寄存器地址
*               data - 存储读取数据的缓冲区
*               len - 要读取的字节数
* @retval:     	void
* @details:    	通过BMI088加速度计的SPI总线连续读取多个寄存器的宏定义
************************************************************************
**/
// 加速度计 SPI 连续读多寄存器：以 reg 为起始地址，读 len 字节存入 data 缓冲区
#define BMI088_accel_read_muli_reg(reg, data, len) \
    {                                              \
        BMI088_ACCEL_NS_L();                       \
        BMI088_read_write_byte((reg) | 0x80);      \
        BMI088_read_muli_reg(reg, data, len);      \
        BMI088_ACCEL_NS_H();                       \
    }
/**
************************************************************************
* @brief:          BMI088_gyro_write_single_reg(reg, data)
* @param:       reg - 寄存器地址
*               data - 写入的数据
* @retval:         void
* @details:        通过BMI088陀螺仪的SPI总线写入单个寄存器的宏定义
************************************************************************
**/
/**
************************************************************************
* @brief:      	BMI088_gyro_write_single_reg(reg, data)
* @param:       reg - 寄存器地址
*               data - 写入的数据
* @retval:     	void
* @details:    	通过BMI088陀螺仪的SPI总线写入单个寄存器的宏定义
************************************************************************
**/
// 陀螺仪 SPI 单寄存器写：片选拉低 → 写寄存器地址与数据 → 片选拉高
#define BMI088_gyro_write_single_reg(reg, data) \
    {                                           \
        BMI088_GYRO_NS_L();                     \
        BMI088_write_single_reg((reg), (data)); \
        BMI088_GYRO_NS_H();                     \
    }
/**
************************************************************************
* @brief:          BMI088_gyro_read_single_reg(reg, data)
* @param:       reg - 寄存器地址
*               data - 读取的寄存器数据
* @retval:         void
* @details:        通过BMI088陀螺仪的SPI总线读取单个寄存器的宏定义
************************************************************************
**/
/**
************************************************************************
* @brief:      	BMI088_gyro_read_single_reg(reg, data)
* @param:       reg - 寄存器地址
*               data - 读取的寄存器数据
* @retval:     	void
* @details:    	通过BMI088陀螺仪的SPI总线读取单个寄存器的宏定义
************************************************************************
**/
// 陀螺仪 SPI 单寄存器读：片选拉低 → 读单寄存器 → 片选拉高（data 以取地址方式传入）
#define BMI088_gyro_read_single_reg(reg, data)  \
    {                                           \
        BMI088_GYRO_NS_L();                     \
        BMI088_read_single_reg((reg), &(data)); \
        BMI088_GYRO_NS_H();                     \
    }
/**
************************************************************************
* @brief:          BMI088_gyro_read_muli_reg(reg, data, len)
* @param:       reg - 起始寄存器地址
*               data - 存储读取数据的缓冲区
*               len - 要读取的字节数
* @retval:         void
* @details:        通过BMI088陀螺仪的SPI总线连续读取多个寄存器的宏定义
************************************************************************
**/
/**
************************************************************************
* @brief:      	BMI088_gyro_read_muli_reg(reg, data, len)
* @param:       reg - 起始寄存器地址
*               data - 存储读取数据的缓冲区
*               len - 要读取的字节数
* @retval:     	void
* @details:    	通过BMI088陀螺仪的SPI总线连续读取多个寄存器的宏定义
************************************************************************
**/
// 陀螺仪 SPI 连续读多寄存器：以 reg 为起始地址，读 len 字节存入 data 缓冲区
#define BMI088_gyro_read_muli_reg(reg, data, len)   \
    {                                               \
        BMI088_GYRO_NS_L();                         \
        BMI088_read_muli_reg((reg), (data), (len)); \
        BMI088_GYRO_NS_H();                         \
    }

// SPI 底层读写函数前置声明（函数体定义在文件末尾 #if defined(BMI088_USE_SPI) 段内）
static void BMI088_write_single_reg(uint8_t reg, uint8_t data);
static void BMI088_read_single_reg(uint8_t reg, uint8_t *return_data);
//static void BMI088_write_muli_reg(uint8_t reg, uint8_t* buf, uint8_t len );
static void BMI088_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len);

#elif defined(BMI088_USE_IIC)


#endif
/**
************************************************************************
* @brief:          write_BMI088_accel_reg_data_error_init(void)
* @param:       void
* @retval:         void
* @details:        BMI088加速度传感器寄存器数据写入错误处理初始化
************************************************************************
**/
/**
************************************************************************
* @brief:      	write_BMI088_accel_reg_data_error_init(void)
* @param:       void
* @retval:     	void
* @details:    	BMI088加速度传感器寄存器数据写入错误处理初始化
************************************************************************
**/
// 加速度计初始化配置表：每行 [0]=寄存器地址 / [1]=要写入的值 / [2]=失败时返回的错误码。
// 初始化时逐行“写入→回读”，回读值不等于写入值就返回该行错误码。
static uint8_t write_BMI088_accel_reg_data_error[BMI088_WRITE_ACCEL_REG_NUM][3] =
    {
        {BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON, BMI088_ACC_PWR_CTRL_ERROR},
        {BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE, BMI088_ACC_PWR_CONF_ERROR},
        {BMI088_ACC_CONF,  BMI088_ACC_NORMAL| BMI088_ACC_800_HZ | BMI088_ACC_CONF_MUST_Set, BMI088_ACC_CONF_ERROR},
        {BMI088_ACC_RANGE, BMI088_ACC_RANGE_3G, BMI088_ACC_RANGE_ERROR},
        {BMI088_INT1_IO_CTRL, BMI088_ACC_INT1_IO_ENABLE | BMI088_ACC_INT1_GPIO_PP | BMI088_ACC_INT1_GPIO_LOW, BMI088_INT1_IO_CTRL_ERROR},
        {BMI088_INT_MAP_DATA, BMI088_ACC_INT1_DRDY_INTERRUPT, BMI088_INT_MAP_DATA_ERROR}

};
/**
************************************************************************
* @brief:          write_BMI088_gyro_reg_data_error_init(void)
* @param:       void
* @retval:         void
* @details:        BMI088陀螺仪传感器寄存器数据写入错误处理初始化
************************************************************************
**/
/**
************************************************************************
* @brief:      	write_BMI088_gyro_reg_data_error_init(void)
* @param:       void
* @retval:     	void
* @details:    	BMI088陀螺仪传感器寄存器数据写入错误处理初始化
************************************************************************
**/
// 陀螺仪初始化配置表：格式同上（[0]=地址 / [1]=写入值 / [2]=错误码）
static uint8_t write_BMI088_gyro_reg_data_error[BMI088_WRITE_GYRO_REG_NUM][3] =
    {
        {BMI088_GYRO_RANGE, BMI088_GYRO_2000, BMI088_GYRO_RANGE_ERROR},
        {BMI088_GYRO_BANDWIDTH, BMI088_GYRO_1000_116_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set, BMI088_GYRO_BANDWIDTH_ERROR},
        {BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE, BMI088_GYRO_LPM1_ERROR},
        {BMI088_GYRO_CTRL, BMI088_DRDY_ON, BMI088_GYRO_CTRL_ERROR},
        {BMI088_GYRO_INT3_INT4_IO_CONF, BMI088_GYRO_INT3_GPIO_PP | BMI088_GYRO_INT3_GPIO_LOW, BMI088_GYRO_INT3_INT4_IO_CONF_ERROR},
        {BMI088_GYRO_INT3_INT4_IO_MAP, BMI088_GYRO_DRDY_IO_INT3, BMI088_GYRO_INT3_INT4_IO_MAP_ERROR}

};
/**
************************************************************************
* @brief:          BMI088_init(void)
* @param:       void
* @retval:         uint8_t - 错误代码
* @details:        BMI088传感器初始化函数，包括GPIO和SPI初始化，以及加速度和陀螺仪的初始化
************************************************************************
**/
/**
************************************************************************
* @brief:      	BMI088_init(void)
* @param:       void
* @retval:     	uint8_t - 错误代码
* @details:    	BMI088传感器初始化函数，包括GPIO和SPI初始化，以及加速度和陀螺仪的初始化
************************************************************************
**/
// 总初始化：先配置 GPIO/SPI，再依次初始化加速度计与陀螺仪。
uint8_t BMI088_init(void)
{
    // 错误累加器：初值 0，各子步骤的错误码按位或进来
    uint8_t error = BMI088_NO_ERROR;
    // —— 底层硬件初始化 ——
    // GPIO and SPI  Init .
    // 初始化片选/中断等 GPIO
    BMI088_GPIO_init();
    // 初始化 SPI 外设
    BMI088_com_init();

    // 初始化加速度计，错误码按位或累加
    error |= bmi088_accel_init();
    // 初始化陀螺仪，错误码按位或累加
    error |= bmi088_gyro_init();

    return error;
}
/**
************************************************************************
* @brief:          bmi088_accel_init(void)
* @param:       void
* @retval:         uint8_t - 错误代码
* @details:        BMI088加速度传感器初始化函数，包括通信检查、软件复位、配置寄存器写入及检查
************************************************************************
**/
/**
************************************************************************
* @brief:      	bmi088_accel_init(void)
* @param:       void
* @retval:     	uint8_t - 错误代码
* @details:    	BMI088加速度传感器初始化函数，包括通信检查、软件复位、配置寄存器写入及检查
************************************************************************
**/
// 初始化加速度计：通信自检 → 软复位 → 逐项写配置并回读校验。
uint8_t bmi088_accel_init(void)
{
    // res：读写寄存器用的临时缓存/回读值；write_reg_num：配置表行号
    uint8_t res = 0;
    uint8_t write_reg_num = 0;

    // —— 通信自检：连续两次读 CHIP_ID，用于唤醒 SPI 接口并确认链路正常 ——
    //check commiunication
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    // —— 软复位加速度计；复位后需长延时等待芯片内部上电自检完成 ——
    //accel software reset
    BMI088_accel_write_single_reg(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
    BMI088_delay_ms(BMI088_LONG_DELAY_TIME);

    // —— 复位后再确认一次通信正常 ——
    //check commiunication is normal after reset
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    // —— 校验芯片 ID，不匹配说明传感器不在线 ——
    // check the "who am I"
    if (res != BMI088_ACC_CHIP_ID_VALUE)
    {
        return BMI088_NO_SENSOR;
    }

    // —— 逐项写入配置寄存器并回读校验，写入值不等于期望值即返回该错误码 ——
    //set accel sonsor config and check
    for (write_reg_num = 0; write_reg_num < BMI088_WRITE_ACCEL_REG_NUM; write_reg_num++)
    {

        BMI088_accel_write_single_reg(write_BMI088_accel_reg_data_error[write_reg_num][0], write_BMI088_accel_reg_data_error[write_reg_num][1]);
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        BMI088_accel_read_single_reg(write_BMI088_accel_reg_data_error[write_reg_num][0], res);
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        if (res != write_BMI088_accel_reg_data_error[write_reg_num][1])
        {
            return write_BMI088_accel_reg_data_error[write_reg_num][2];
        }
    }
    return BMI088_NO_ERROR;
}
/**
************************************************************************
* @brief:          bmi088_gyro_init(void)
* @param:       void
* @retval:         uint8_t - 错误代码
* @details:        BMI088陀螺仪传感器初始化函数，包括通信检查、软件复位、配置寄存器写入及检查
************************************************************************
**/
/**
************************************************************************
* @brief:      	bmi088_gyro_init(void)
* @param:       void
* @retval:     	uint8_t - 错误代码
* @details:    	BMI088陀螺仪传感器初始化函数，包括通信检查、软件复位、配置寄存器写入及检查
************************************************************************
**/
// 初始化陀螺仪：通信自检 → 软复位 → 逐项写配置并回读校验。
uint8_t bmi088_gyro_init(void)
{
    // write_reg_num：配置表行号；res：读寄存器用的临时缓存/回读值
    uint8_t write_reg_num = 0;
    uint8_t res = 0;

    // —— 通信自检：连续两次读 CHIP_ID，用于唤醒 SPI 接口并确认链路正常 ——
    //check commiunication
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    // —— 软复位陀螺仪；复位后需长延时等待内部上电自检完成 ——
    //reset the gyro sensor
    BMI088_gyro_write_single_reg(BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
    BMI088_delay_ms(BMI088_LONG_DELAY_TIME);
    // —— 复位后再确认一次通信正常 ——
    //check commiunication is normal after reset
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    // —— 校验芯片 ID，不匹配说明传感器不在线 ——
    // check the "who am I"
    if (res != BMI088_GYRO_CHIP_ID_VALUE)
    {
        return BMI088_NO_SENSOR;
    }

    // —— 逐项写入配置寄存器并回读校验，写入值不等于期望值即返回该错误码 ——
    //set gyro sonsor config and check
    for (write_reg_num = 0; write_reg_num < BMI088_WRITE_GYRO_REG_NUM; write_reg_num++)
    {

        BMI088_gyro_write_single_reg(write_BMI088_gyro_reg_data_error[write_reg_num][0], write_BMI088_gyro_reg_data_error[write_reg_num][1]);
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        BMI088_gyro_read_single_reg(write_BMI088_gyro_reg_data_error[write_reg_num][0], res);
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        if (res != write_BMI088_gyro_reg_data_error[write_reg_num][1])
        {
            return write_BMI088_gyro_reg_data_error[write_reg_num][2];
        }
    }

    return BMI088_NO_ERROR;
}
/**
************************************************************************
* @brief:          BMI088_read(float gyro[3], float accel[3], float *temperate)
* @param:       gyro - 陀螺仪数据数组 (x, y, z)
* @param:       accel - 加速度计数据数组 (x, y, z)
* @param:       temperate - 温度数据指针
* @retval:         void
* @details:        读取BMI088传感器数据，包括加速度、陀螺仪和温度
************************************************************************
**/
/**
************************************************************************
* @brief:      	BMI088_read(float gyro[3], float accel[3], float *temperate)
* @param:       gyro - 陀螺仪数据数组 (x, y, z)
* @param:       accel - 加速度计数据数组 (x, y, z)
* @param:       temperate - 温度数据指针
* @retval:     	void
* @details:    	读取BMI088传感器数据，包括加速度、陀螺仪和温度
************************************************************************
**/
// 读取一组完整数据：加速度计三轴 → 陀螺仪三轴 → 温度，均换算为物理量。
// 参数：gyro 输出角速度[deg/s]；accel 输出加速度[g]；temperate 输出温度[摄氏度]。
void BMI088_read(float gyro[3], float accel[3], float *temperate)
{
    // 收发缓冲：先用于读加速度/陀螺仪原始字节，后复用于读温度
    uint8_t buf[8] = {0, 0, 0, 0, 0, 0};
    // 临时保存 16 位原始读数（加速度/陀螺仪分量或温度原始值）
    int16_t bmi088_raw_temp;

    // 读加速度计 X/Y/Z 三轴原始数据（每轴 2 字节，共 6 字节，小端：低字节在前）
    BMI088_accel_read_muli_reg(BMI088_ACCEL_XOUT_L, buf, 6);

    // 小端拼接为有符号 16 位（高字节在前、低字节在后），再乘刻度系数转成 g
    bmi088_raw_temp = (int16_t)((buf[1]) << 8) | buf[0];
    accel[0] = bmi088_raw_temp * BMI088_ACCEL_SEN;
    bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
    accel[1] = bmi088_raw_temp * BMI088_ACCEL_SEN;
    bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
    accel[2] = bmi088_raw_temp * BMI088_ACCEL_SEN;

    // 连续读 8 字节：首字节是陀螺仪的 CHIP_ID，随后是 X/Y/Z 原始数据
    BMI088_gyro_read_muli_reg(BMI088_GYRO_CHIP_ID, buf, 8);
    // 校验 CHIP_ID，正确才解析陀螺仪数据，避免读到无效值
    if(buf[0] == BMI088_GYRO_CHIP_ID_VALUE)
    {
        // 每轴同样小端拼接后乘刻度系数转成 deg/s
        bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
        gyro[0] = bmi088_raw_temp * BMI088_GYRO_SEN;
        bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
        gyro[1] = bmi088_raw_temp * BMI088_GYRO_SEN;
        bmi088_raw_temp = (int16_t)((buf[7]) << 8) | buf[6];
        gyro[2] = bmi088_raw_temp * BMI088_GYRO_SEN;
    }
    // 读温度原始值（2 字节，来自加速度计的温度寄存器）
    BMI088_accel_read_muli_reg(BMI088_TEMP_M, buf, 2);

    // 温度是 12 位：buf[0] 的 8 位作高 8 位，buf[1] 的高 5 位作低 5 位
    bmi088_raw_temp = (int16_t)((buf[0] << 3) | (buf[1] >> 5));

    // 12 位有符号还原：最高位是符号位，读数 >1023 表示负值，减 2048 转回有符号
    if (bmi088_raw_temp > 1023)
    {
        bmi088_raw_temp -= 2048;
    }

    // 换算为摄氏度：原始值 A* 0.125 + 23.0
    *temperate = bmi088_raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
}

#if defined(BMI088_USE_SPI)
/**
************************************************************************
* @brief:          BMI088_write_single_reg(uint8_t reg, uint8_t data)
* @param:       reg - 寄存器地址
* @param:       data - 写入的数据
* @retval:         void
* @details:        向BMI088传感器写入单个寄存器的数据
************************************************************************
**/
/**
************************************************************************
* @brief:      	BMI088_write_single_reg(uint8_t reg, uint8_t data)
* @param:       reg - 寄存器地址
* @param:       data - 写入的数据
* @retval:     	void
* @details:    	向BMI088传感器写入单个寄存器的数据
************************************************************************
**/
// 写单个寄存器：先发寄存器地址，再发数据（片选由调用处的宏控制）
static void BMI088_write_single_reg(uint8_t reg, uint8_t data)
{
    // 第 1 字节：寄存器地址（写操作地址最高位为 0）
    BMI088_read_write_byte(reg);
    // 第 2 字节：要写入的数据
    BMI088_read_write_byte(data);
}
/**
************************************************************************
* @brief:          BMI088_read_single_reg(uint8_t reg, uint8_t *return_data)
* @param:       reg - 寄存器地址
* @param:       return_data - 读取的寄存器数据
* @retval:         void
* @details:        从BMI088传感器读取单个寄存器的数据
************************************************************************
**/
/**
************************************************************************
* @brief:      	BMI088_read_single_reg(uint8_t reg, uint8_t *return_data)
* @param:       reg - 寄存器地址
* @param:       return_data - 读取的寄存器数据
* @retval:     	void
* @details:    	从BMI088传感器读取单个寄存器的数据
************************************************************************
**/
// 读单个寄存器：先发 0x80|reg（置位最高位为读命令），发 1 字节 dummy，再回读 1 字节数据
static void BMI088_read_single_reg(uint8_t reg, uint8_t *return_data)
{
    BMI088_read_write_byte(reg | 0x80);
    // 发 dummy 字节(0x55)产生时钟，同时接收器件返回的寄存器值
    *return_data = BMI088_read_write_byte(0x55);
}

//static void BMI088_write_muli_reg(uint8_t reg, uint8_t* buf, uint8_t len )
//{
//    BMI088_read_write_byte( reg );
//    while( len != 0 )
//    {

//        BMI088_read_write_byte( *buf );
//        buf ++;
//        len --;
//    }

//}
/**
************************************************************************
* @brief:          BMI088_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len)
* @param:       reg - 起始寄存器地址
*               buf - 存储读取数据的缓冲区
*               len - 要读取的字节数
* @retval:         void
* @details:        从BMI088传感器连续读取多个寄存器的数据
************************************************************************
**/
/**
************************************************************************
* @brief:      	BMI088_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len)
* @param:       reg - 起始寄存器地址
*               buf - 存储读取数据的缓冲区
*               len - 要读取的字节数
* @retval:     	void
* @details:    	从BMI088传感器连续读取多个寄存器的数据
************************************************************************
**/
// 连续读多个寄存器：先发 0x80|reg（读命令），再依次读出 len 字节存入 buf
static void BMI088_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len)
{
    // 发读命令（地址最高位置 1 表示读）
    BMI088_read_write_byte(reg | 0x80);

    // 循环 len 次：每读 1 字节存入缓冲区，指针后移、计数递减
    while (len != 0)
    {

        *buf = BMI088_read_write_byte(0x55);
        buf++;
        len--;
    }
}
#elif defined(BMI088_USE_IIC)

#endif
