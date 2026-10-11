/*
 * 文件：Crc.h
 * 功能：RoboMaster 哨兵上位机使用的两种循环冗余校验（CRC）算法接口声明。
 *
 * 术语说明（CRC，Cyclic Redundancy Check，循环冗余校验）：
 *   在数据末尾附加一小段“校验码”，接收方用同样算法重算一遍并与收到的校验码比对：
 *   一致 ⇒ 认为数据在传输/存储中没有发生位翻转；不一致 ⇒ 判定数据损坏并丢弃。
 *   它只能发现错误，不能纠正错误，因此常用来保护裁判系统与内部通信的数据帧。
 *
 * 本文件提供两套表驱动（查表法）实现，差别只在生成多项式和校验码宽度：
 *   - CRC8 ：8 位校验码，生成多项式 G(x) = x8 + x5 + x4 + 1，用于帧头校验。
 *   - CRC16：16 位校验码，反射式查表（等价多项式 0x8005），用于整帧校验。
 *   “表驱动”即预先算出全部 256 种中间结果放进查表数组，运行时只做异或与查表，
 *   速度快、适合单片机实时场合。
 *
 * 命名约定（沿用 ST 官方示例）：
 *   Append_xxx : 计算校验码并写入消息末尾预留的位置（发送方使用）。
 *   Verify_xxx : 校验整条消息（数据 + 末尾校验码）是否匹配，返回真/假（接收方使用）。
 *   Get_xxx    : 纯计算函数，返回校验码本身，不修改消息内容。
 */

#ifndef _CRC_H_
#define _CRC_H_

#include "stdint.h"
#include "string.h"

/*
 * ===================== CRC8 接口（8 位校验，生成多项式 G(x) = x8 + x5 + x4 + 1）=====================
 */

/*
 * 函数：Append_CRC8_Check_Sum
 * 用途：计算消息前 (dwLength - 1) 个字节的 CRC8，并把结果写入第 (dwLength - 1) 个字节（即末尾）。
 * 参数：
 *   pchMessage —— 指向消息缓冲区首地址；缓冲区至少 dwLength 字节，最后 1 字节留给校验码。
 *   dwLength   —— 消息总长度（数据字节数 + 末尾 1 字节校验码）。
 * 返回：无。
 * 说明：内部使用初始值 CRC8_INIT(0xFF)；若指针为空或长度过短则直接返回，不做任何写入。
 */
void Append_CRC8_Check_Sum( uint8_t *pchMessage, uint16_t dwLength);

/*
 * 函数：Verify_CRC8_Check_Sum
 * 用途：校验整条消息（数据 + 末尾 1 字节 CRC8）是否正确。
 * 参数：
 *   pchMessage —— 消息缓冲区首地址。
 *   dwLength   —— 消息总长度（数据字节数 + 末尾 1 字节校验码）。
 * 返回：非 0 表示校验通过（重算值等于消息末尾携带的校验码）；0 表示不通过或入参非法。
 * 说明：入参非法（空指针或长度不大于 2）时直接返回 0（失败）。
 */
uint32_t Verify_CRC8_Check_Sum( uint8_t *pchMessage, uint16_t dwLength);

/*
 * 函数：Get_CRC8_Check_Sum
 * 用途：纯计算 CRC8 校验码，不修改消息内容。
 * 参数：
 *   pchMessage —— 待计算的数据首地址。
 *   dwLength   —— 参与计算的字节数。
 *   ucCRC8     —— 初始值（CRC 累加的种子，本工程取 CRC8_INIT = 0xFF）。
 * 返回：计算得到的 8 位校验码。
 */
uint8_t Get_CRC8_Check_Sum( uint8_t *pchMessage, uint16_t dwLength, uint8_t ucCRC8 );

/*
 * ===================== CRC16 接口（16 位校验，反射式查表，等价多项式 0x8005）=====================
 */

/*
 * 函数：Append_CRC16_Check_Sum
 * 用途：计算消息前 (dwLength - 2) 个字节的 CRC16，并把结果以“低字节在前”写入消息末尾 2 字节。
 * 参数：
 *   pchMessage —— 指向消息缓冲区首地址；缓冲区至少 dwLength 字节，最后 2 字节留给校验码。
 *   dwLength   —— 消息总长度（数据字节数 + 末尾 2 字节校验码）。
 * 返回：无。
 * 说明：内部使用初始值 CRC_INIT(0xFFFF)；若指针为空或长度过短则直接返回。
 */
void Append_CRC16_Check_Sum(uint8_t * pchMessage,uint32_t dwLength);

/*
 * 函数：Verify_CRC16_Check_Sum
 * 用途：校验整条消息（数据 + 末尾 2 字节 CRC16）是否正确。
 * 参数：
 *   pchMessage —— 消息缓冲区首地址。
 *   dwLength   —— 消息总长度（数据字节数 + 末尾 2 字节校验码）。
 * 返回：非 0 表示校验通过；0 表示不通过或入参非法。
 * 说明：入参非法（空指针或长度不大于 2）时直接返回 0。
 */
uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);

/*
 * 函数：Get_CRC16_Check_Sum
 * 用途：纯计算 CRC16 校验码，不修改消息内容。
 * 参数：
 *   pchMessage —— 待计算的数据首地址。
 *   dwLength   —— 参与计算的字节数。
 *   wCRC       —— 初始值（CRC 累加的种子，本工程取 CRC_INIT = 0xFFFF）。
 * 返回：计算得到的 16 位校验码；若指针为空则返回 0xFFFF。
 */
uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC);

#endif
