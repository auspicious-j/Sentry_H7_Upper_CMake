/*
 * BMI088reg.h —— BMI088 六轴惯性测量单元(IMU)寄存器地址与常量定义
 *
 * 本文件只做宏定义,不含任何代码逻辑。BMI088 内部由两颗独立裸片组成:
 *   - 加速度计(ACC):由 CSB1 片选;SPI 读时首个地址字节需置读标志 0x80。
 *   - 陀螺仪(GYRO):由 CSB2 片选;SPI 读时寄存器地址需或上读标志 0x80。
 *   两颗裸片共用同一条 SPI 总线,但寄存器地址空间各自独立——
 *   即使地址数值相同(如 0x00),对 ACC 与 GYRO 也代表完全不同的寄存器。
 *
 * 命名约定:
 *   - xxx_CHIP_ID     芯片身份寄存器("Who Am I"),用于确认 SPI 通信与芯片型号。
 *   - xxx_SHFITS(xxx_SHIFTS) 位域在该寄存器字节中的起始位序号。
 *   - xxx_yyy (0xN << SHFITS) 已左移到目标位的取值常量,可直接写入寄存器。
 *   说明: SHFITS 为原文件拼写(应为 SHIFTS),此处保持宏名不变以免破坏引用。
 *
 * 本文件仅收录本工程实际用到的寄存器,并非数据手册全集。
 */

#ifndef BMI088REG_H
#define BMI088REG_H

// ===== 加速度计(ACC)寄存器地址与位域 =====

// 芯片 ID 寄存器(只读 0x00): "Who Am I",固定返回 0x1E,用于校验通信与芯片型号
#define BMI088_ACC_CHIP_ID 0x00 // the register is  " Who am I "
#define BMI088_ACC_CHIP_ID_VALUE 0x1E

// 错误寄存器(只读 0x02): 芯片工作异常时置位
#define BMI088_ACC_ERR_REG 0x02
// bit2: accel_config_error,加速度计配置错误(如非法量程/带宽设置)
#define BMI088_ACCEL_CONGIF_ERROR_SHFITS 0x2
#define BMI088_ACCEL_CONGIF_ERROR (1 << BMI088_ACCEL_CONGIF_ERROR_SHFITS)
// bit0: fatal_error,致命错误(如自检失败),需软复位才能恢复
#define BMI088_FATAL_ERROR_SHFITS 0x0
#define BMI088_FATAL_ERROR (1 << BMI088_FATAL_ERROR)

// 状态寄存器(只读 0x03): 反映加速度计数据就绪等状态
#define BMI088_ACC_STATUS 0x03
// bit7: accel_data_ready,新加速度数据可读时置 1
#define BMI088_ACCEL_DRDY_SHFITS 0x7
#define BMI088_ACCEL_DRDY (1 << BMI088_ACCEL_DRDY_SHFITS)

// --- 加速度计数据输出寄存器(只读) ---
// 每个轴为 16 位有符号数,分低字节(L)/高字节(M,即 middle/high)两次读取;
// 常按 X->Y->Z 顺序连读(0x12~0x17),低字节在前。
#define BMI088_ACCEL_XOUT_L 0x12
#define BMI088_ACCEL_XOUT_M 0x13
#define BMI088_ACCEL_YOUT_L 0x14
#define BMI088_ACCEL_YOUT_M 0x15
#define BMI088_ACCEL_ZOUT_L 0x16
#define BMI088_ACCEL_ZOUT_M 0x17

// 传感器时间(只读): 24 位内部时基计数(0x18~0x1A),单位 39.0625us,
// 用于给采样打时间戳、做数据对齐
#define BMI088_SENSORTIME_DATA_L 0x18
#define BMI088_SENSORTIME_DATA_M 0x19
#define BMI088_SENSORTIME_DATA_H 0x1A

// 加速度计中断状态寄存器 1(只读 0x1D)
#define BMI088_ACC_INT_STAT_1 0x1D
// bit7: accel_drdy_int,加速度数据就绪中断触发标志
#define BMI088_ACCEL_DRDY_INTERRUPT_SHFITS 0x7
#define BMI088_ACCEL_DRDY_INTERRUPT (1 << BMI088_ACCEL_DRDY_INTERRUPT_SHFITS)

// --- 温度输出寄存器(只读) ---
// 11 位温度值,先读高字节 0x22、后读低字节 0x23;
// 换算式: 温度[℃] = (Temp_MSB A* 8 + Temp_LSB) A* 0.125 + 23
#define BMI088_TEMP_M 0x22

#define BMI088_TEMP_L 0x23

// 加速度计配置寄存器(0x40): 设定低通滤波带宽与输出数据率(ODR)
#define BMI088_ACC_CONF 0x40
// bit7 必须置 1(固定写 0x80),复位默认值,不置位配置可能不生效
#define BMI088_ACC_CONF_MUST_Set 0x80
// bit6~4: acc_bwp,加速度计低通滤波(带宽)选择
#define BMI088_ACC_BWP_SHFITS 0x4
#define BMI088_ACC_OSR4 (0x0 << BMI088_ACC_BWP_SHFITS) // 过采样 x4(带宽最窄)
#define BMI088_ACC_OSR2 (0x1 << BMI088_ACC_BWP_SHFITS) // 过采样 x2
#define BMI088_ACC_NORMAL (0x2 << BMI088_ACC_BWP_SHFITS) // 标准模式(带宽较宽)

// bit3~0: acc_odr,加速度计输出数据率
#define BMI088_ACC_ODR_SHFITS 0x0
#define BMI088_ACC_12_5_HZ (0x5 << BMI088_ACC_ODR_SHFITS)
#define BMI088_ACC_25_HZ (0x6 << BMI088_ACC_ODR_SHFITS)
#define BMI088_ACC_50_HZ (0x7 << BMI088_ACC_ODR_SHFITS)
#define BMI088_ACC_100_HZ (0x8 << BMI088_ACC_ODR_SHFITS)
#define BMI088_ACC_200_HZ (0x9 << BMI088_ACC_ODR_SHFITS)
#define BMI088_ACC_400_HZ (0xA << BMI088_ACC_ODR_SHFITS)
#define BMI088_ACC_800_HZ (0xB << BMI088_ACC_ODR_SHFITS)
#define BMI088_ACC_1600_HZ (0xC << BMI088_ACC_ODR_SHFITS)

// 加速度计量程寄存器(0x41)
#define BMI088_ACC_RANGE 0x41

// bit1~0: acc_range,量程(满量程范围);数值越大可测加速度越大、分辨率越低
#define BMI088_ACC_RANGE_SHFITS 0x0
#define BMI088_ACC_RANGE_3G (0x0 << BMI088_ACC_RANGE_SHFITS)
#define BMI088_ACC_RANGE_6G (0x1 << BMI088_ACC_RANGE_SHFITS)
#define BMI088_ACC_RANGE_12G (0x2 << BMI088_ACC_RANGE_SHFITS)
#define BMI088_ACC_RANGE_24G (0x3 << BMI088_ACC_RANGE_SHFITS)

// 加速度计 INT1 引脚 IO 控制寄存器(0x53): 配置 INT1 引脚的使能/输出类型/有效电平
#define BMI088_INT1_IO_CTRL 0x53
// bit3: int1_io_en,INT1 引脚输出使能(置 1 才对外输出信号)
#define BMI088_ACC_INT1_IO_ENABLE_SHFITS 0x3
#define BMI088_ACC_INT1_IO_ENABLE (0x1 << BMI088_ACC_INT1_IO_ENABLE_SHFITS)
// bit2: int1_lvl_pp_od,INT1 输出类型(推挽/开漏)
#define BMI088_ACC_INT1_GPIO_MODE_SHFITS 0x2
#define BMI088_ACC_INT1_GPIO_PP (0x0 << BMI088_ACC_INT1_GPIO_MODE_SHFITS) // 推挽输出
#define BMI088_ACC_INT1_GPIO_OD (0x1 << BMI088_ACC_INT1_GPIO_MODE_SHFITS) // 开漏输出
// bit1: int1_lvl,INT1 有效电平
#define BMI088_ACC_INT1_GPIO_LVL_SHFITS 0x1
#define BMI088_ACC_INT1_GPIO_LOW (0x0 << BMI088_ACC_INT1_GPIO_LVL_SHFITS) // 低电平有效
#define BMI088_ACC_INT1_GPIO_HIGH (0x1 << BMI088_ACC_INT1_GPIO_LVL_SHFITS) // 高电平有效

// 加速度计 INT2 引脚 IO 控制寄存器(0x54): 位域含义同 INT1,对应 INT2 引脚
#define BMI088_INT2_IO_CTRL 0x54
// bit3: int2_io_en,INT2 引脚输出使能
#define BMI088_ACC_INT2_IO_ENABLE_SHFITS 0x3
#define BMI088_ACC_INT2_IO_ENABLE (0x1 << BMI088_ACC_INT2_IO_ENABLE_SHFITS)
// bit2: int2_lvl_pp_od,INT2 输出类型(推挽/开漏)
#define BMI088_ACC_INT2_GPIO_MODE_SHFITS 0x2
#define BMI088_ACC_INT2_GPIO_PP (0x0 << BMI088_ACC_INT2_GPIO_MODE_SHFITS) // 推挽输出
#define BMI088_ACC_INT2_GPIO_OD (0x1 << BMI088_ACC_INT2_GPIO_MODE_SHFITS) // 开漏输出
// bit1: int2_lvl,INT2 有效电平
#define BMI088_ACC_INT2_GPIO_LVL_SHFITS 0x1
#define BMI088_ACC_INT2_GPIO_LOW (0x0 << BMI088_ACC_INT2_GPIO_LVL_SHFITS) // 低电平有效
#define BMI088_ACC_INT2_GPIO_HIGH (0x1 << BMI088_ACC_INT2_GPIO_LVL_SHFITS) // 高电平有效

// 中断映射寄存器(0x58): 决定哪些事件信号路由到 INT1/INT2 引脚
#define BMI088_INT_MAP_DATA 0x58
// bit6: int2_drdy,把"加速度数据就绪"信号映射到 INT2 引脚
#define BMI088_ACC_INT2_DRDY_INTERRUPT_SHFITS 0x6
#define BMI088_ACC_INT2_DRDY_INTERRUPT (0x1 << BMI088_ACC_INT2_DRDY_INTERRUPT_SHFITS)
// bit2: int1_drdy,把"加速度数据就绪"信号映射到 INT1 引脚
#define BMI088_ACC_INT1_DRDY_INTERRUPT_SHFITS 0x2
#define BMI088_ACC_INT1_DRDY_INTERRUPT (0x1 << BMI088_ACC_INT1_DRDY_INTERRUPT_SHFITS)

// 加速度计自检寄存器(0x6D): 触发内部自检并输出规定激励信号
#define BMI088_ACC_SELF_TEST 0x6D
#define BMI088_ACC_SELF_TEST_OFF 0x00 // 关闭自检
#define BMI088_ACC_SELF_TEST_POSITIVE_SIGNAL 0x0D // 输出正激励信号
#define BMI088_ACC_SELF_TEST_NEGATIVE_SIGNAL 0x09 // 输出负激励信号

// 加速度计电源配置寄存器(0x7C): 切换挂起/工作模式
#define BMI088_ACC_PWR_CONF 0x7C
#define BMI088_ACC_PWR_SUSPEND_MODE 0x03 // 挂起模式(低功耗,不采样)
#define BMI088_ACC_PWR_ACTIVE_MODE 0x00 // 正常工作模式(采样并使能加速度计)

// 加速度计电源控制寄存器(0x7D): 打开/关闭加速度计测量单元
#define BMI088_ACC_PWR_CTRL 0x7D
#define BMI088_ACC_ENABLE_ACC_OFF 0x00 // 关闭加速度计
#define BMI088_ACC_ENABLE_ACC_ON 0x04 // 使能加速度计(需置于 active 模式下才生效)

// 加速度计软复位寄存器(0x7E): 写入固定值 0xB6 触发复位
#define BMI088_ACC_SOFTRESET 0x7E
#define BMI088_ACC_SOFTRESET_VALUE 0xB6 // 复位命令字,固定值

// ===== 陀螺仪(GYRO)寄存器地址与位域 =====
// 注意: 陀螺仪寄存器地址与加速度计独立,如 0x00 在此处是陀螺仪芯片 ID

// 陀螺仪芯片 ID 寄存器(只读 0x00): 固定返回 0x0F,用于校验陀螺仪通信
#define BMI088_GYRO_CHIP_ID 0x00
#define BMI088_GYRO_CHIP_ID_VALUE 0x0F

// --- 陀螺仪数据输出寄存器(只读) ---
// 每个轴为 16 位有符号数,分低字节(L)/高字节(H)两次读取(0x02~0x07),
// 低字节在前;数值为角速度原始值,需按量程换算为 deg/s
#define BMI088_GYRO_X_L 0x02
#define BMI088_GYRO_X_H 0x03
#define BMI088_GYRO_Y_L 0x04
#define BMI088_GYRO_Y_H 0x05
#define BMI088_GYRO_Z_L 0x06
#define BMI088_GYRO_Z_H 0x07

// 陀螺仪中断状态寄存器 1(只读 0x0A)
#define BMI088_GYRO_INT_STAT_1 0x0A
// bit7: data_ready(drdy),新陀螺数据可读时置 1
#define BMI088_GYRO_DYDR_SHFITS 0x7
#define BMI088_GYRO_DYDR (0x1 << BMI088_GYRO_DYDR_SHFITS)

// 陀螺仪量程寄存器(0x0F)
#define BMI088_GYRO_RANGE 0x0F
// bit2~0: gyro_range,量程(单位 deg/s),数值越大可测角速度越大、分辨率越低
#define BMI088_GYRO_RANGE_SHFITS 0x0
#define BMI088_GYRO_2000 (0x0 << BMI088_GYRO_RANGE_SHFITS) // ±2000 deg/s
#define BMI088_GYRO_1000 (0x1 << BMI088_GYRO_RANGE_SHFITS) // ±1000 deg/s
#define BMI088_GYRO_500 (0x2 << BMI088_GYRO_RANGE_SHFITS) // ±500 deg/s
#define BMI088_GYRO_250 (0x3 << BMI088_GYRO_RANGE_SHFITS) // ±250 deg/s
#define BMI088_GYRO_125 (0x4 << BMI088_GYRO_RANGE_SHFITS) // ±125 deg/s

// 陀螺仪带宽寄存器(0x10): 同时设定输出数据率与低通滤波带宽
#define BMI088_GYRO_BANDWIDTH 0x10
// 常量名中前一个数值为输出数据率(ODR),后一个数值为低通滤波带宽
#define BMI088_GYRO_BANDWIDTH_MUST_Set 0x80 // bit7 必须置 1,否则写配置可能不生效
#define BMI088_GYRO_2000_532_HZ 0x00 // ODR 2000Hz / 带宽 532Hz
#define BMI088_GYRO_2000_230_HZ 0x01 // ODR 2000Hz / 带宽 230Hz
#define BMI088_GYRO_1000_116_HZ 0x02 // ODR 1000Hz / 带宽 116Hz
#define BMI088_GYRO_400_47_HZ 0x03 // ODR 400Hz / 带宽 47Hz
#define BMI088_GYRO_200_23_HZ 0x04 // ODR 200Hz / 带宽 23Hz
#define BMI088_GYRO_100_12_HZ 0x05 // ODR 100Hz / 带宽 12Hz
#define BMI088_GYRO_200_64_HZ 0x06 // ODR 200Hz / 带宽 64Hz
#define BMI088_GYRO_100_32_HZ 0x07 // ODR 100Hz / 带宽 32Hz

// 陀螺仪低功耗模式寄存器 LPM1(0x11): 切换正常/挂起/深度挂起
#define BMI088_GYRO_LPM1 0x11
#define BMI088_GYRO_NORMAL_MODE 0x00 // 正常工作模式(采样并使能陀螺仪)
#define BMI088_GYRO_SUSPEND_MODE 0x80 // 挂起模式(低功耗,快速唤醒)
#define BMI088_GYRO_DEEP_SUSPEND_MODE 0x20 // 深度挂起模式(更低功耗,唤醒较慢)

// 陀螺仪软复位寄存器(0x14): 写入固定值 0xB6 触发复位
#define BMI088_GYRO_SOFTRESET 0x14
#define BMI088_GYRO_SOFTRESET_VALUE 0xB6 // 复位命令字,固定值

// 陀螺仪控制寄存器(0x15): 控制是否使能新数据中断(drdy)信号
#define BMI088_GYRO_CTRL 0x15
#define BMI088_DRDY_OFF 0x00 // 关闭数据就绪中断
#define BMI088_DRDY_ON 0x80 // 使能数据就绪中断

// 陀螺仪 INT3/INT4 引脚 IO 配置寄存器(0x16): 配置两引脚的输出类型与有效电平
#define BMI088_GYRO_INT3_INT4_IO_CONF 0x16
// bit3: int4_lvl_pp_od,INT4 输出类型(推挽/开漏)
#define BMI088_GYRO_INT4_GPIO_MODE_SHFITS 0x3
#define BMI088_GYRO_INT4_GPIO_PP (0x0 << BMI088_GYRO_INT4_GPIO_MODE_SHFITS) // 推挽输出
#define BMI088_GYRO_INT4_GPIO_OD (0x1 << BMI088_GYRO_INT4_GPIO_MODE_SHFITS) // 开漏输出
// bit2: int4_lvl,INT4 有效电平
#define BMI088_GYRO_INT4_GPIO_LVL_SHFITS 0x2
#define BMI088_GYRO_INT4_GPIO_LOW (0x0 << BMI088_GYRO_INT4_GPIO_LVL_SHFITS) // 低电平有效
#define BMI088_GYRO_INT4_GPIO_HIGH (0x1 << BMI088_GYRO_INT4_GPIO_LVL_SHFITS) // 高电平有效
// bit1: int3_lvl_pp_od,INT3 输出类型(推挽/开漏)
#define BMI088_GYRO_INT3_GPIO_MODE_SHFITS 0x1
#define BMI088_GYRO_INT3_GPIO_PP (0x0 << BMI088_GYRO_INT3_GPIO_MODE_SHFITS) // 推挽输出
#define BMI088_GYRO_INT3_GPIO_OD (0x1 << BMI088_GYRO_INT3_GPIO_MODE_SHFITS) // 开漏输出
// bit0: int3_lvl,INT3 有效电平
#define BMI088_GYRO_INT3_GPIO_LVL_SHFITS 0x0
#define BMI088_GYRO_INT3_GPIO_LOW (0x0 << BMI088_GYRO_INT3_GPIO_LVL_SHFITS) // 低电平有效
#define BMI088_GYRO_INT3_GPIO_HIGH (0x1 << BMI088_GYRO_INT3_GPIO_LVL_SHFITS) // 高电平有效

// 陀螺仪 INT3/INT4 数据中断映射寄存器(0x18): 将"数据就绪"信号路由到 INT3/INT4
#define BMI088_GYRO_INT3_INT4_IO_MAP 0x18

// 数据就绪输出目标选择(配合 0x18 使用)
#define BMI088_GYRO_DRDY_IO_OFF 0x00 // 不输出到任何引脚
#define BMI088_GYRO_DRDY_IO_INT3 0x01 // 输出到 INT3
#define BMI088_GYRO_DRDY_IO_INT4 0x80 // 输出到 INT4
#define BMI088_GYRO_DRDY_IO_BOTH (BMI088_GYRO_DRDY_IO_INT3 | BMI088_GYRO_DRDY_IO_INT4) // 两引脚同时输出

// 陀螺仪自检寄存器(0x3C): 触发内建自检(BIST)并读取结果
#define BMI088_GYRO_SELF_TEST 0x3C
// bit4: rate_ok,自检得到的角速率结果有效
#define BMI088_GYRO_RATE_OK_SHFITS 0x4
#define BMI088_GYRO_RATE_OK (0x1 << BMI088_GYRO_RATE_OK_SHFITS)
// bit2: bist_fail,自检失败标志
#define BMI088_GYRO_BIST_FAIL_SHFITS 0x2
#define BMI088_GYRO_BIST_FAIL (0x1 << BMI088_GYRO_BIST_FAIL_SHFITS)
// bit1: bist_rdy,自检完成、结果可读
#define BMI088_GYRO_BIST_RDY_SHFITS 0x1
#define BMI088_GYRO_BIST_RDY (0x1 << BMI088_GYRO_BIST_RDY_SHFITS)
// bit0: trig_bist,置 1 触发一次内建自检
#define BMI088_GYRO_TRIG_BIST_SHFITS 0x0
#define BMI088_GYRO_TRIG_BIST (0x1 << BMI088_GYRO_TRIG_BIST_SHFITS)

#endif
