// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#include "rm_serial_driver/crc.hpp"

#include <cstdint>

namespace crc16
{
constexpr uint16_t CRC16_INIT = 0xFFFF;

constexpr uint16_t W_CRC_TABLE[256] = {
  0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf, 0x8c48, 0x9dc1, 0xaf5a, 0xbed3,
  0xca6c, 0xdbe5, 0xe97e, 0xf8f7, 0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
  0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876, 0x2102, 0x308b, 0x0210, 0x1399,
  0x6726, 0x76af, 0x4434, 0x55bd, 0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
  0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c, 0xbdcb, 0xac42, 0x9ed9, 0x8f50,
  0xfbef, 0xea66, 0xd8fd, 0xc974, 0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
  0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3, 0x5285, 0x430c, 0x7197, 0x601e,
  0x14a1, 0x0528, 0x37b3, 0x263a, 0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
  0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9, 0xef4e, 0xfec7, 0xcc5c, 0xddd5,
  0xa96a, 0xb8e3, 0x8a78, 0x9bf1, 0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
  0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70, 0x8408, 0x9581, 0xa71a, 0xb693,
  0xc22c, 0xd3a5, 0xe13e, 0xf0b7, 0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
  0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036, 0x18c1, 0x0948, 0x3bd3, 0x2a5a,
  0x5ee5, 0x4f6c, 0x7df7, 0x6c7e, 0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
  0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd, 0xb58b, 0xa402, 0x9699, 0x8710,
  0xf3af, 0xe226, 0xd0bd, 0xc134, 0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
  0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3, 0x4a44, 0x5bcd, 0x6956, 0x78df,
  0x0c60, 0x1de9, 0x2f72, 0x3efb, 0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
  0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a, 0xe70e, 0xf687, 0xc41c, 0xd595,
  0xa12a, 0xb0a3, 0x8238, 0x93b1, 0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
  0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330, 0x7bc7, 0x6a4e, 0x58d5, 0x495c,
  0x3de3, 0x2c6a, 0x1ef1, 0x0f78};

/**
  * @brief CRC16 Caculation function
  * @param[in] pchMessage : Data to Verify,
  * @param[in] dwLength : Stream length = Data + checksum
  * @param[in] wCRC : CRC16 init value(default : 0xFFFF)
  * @return : CRC16 checksum
  */
/**
 * 计算CRC16校验和
 * 
 * @param pchMessage 指向待计算校验和的数据消息的指针
 * @param dwLength 数据消息的长度
 * @param wCRC 初始CRC值
 * @return 计算得到的CRC16校验和
 * 
 * 此函数通过遍历数据消息中的每个字节，并使用预定义的CRC表来更新CRC值，
 * 从而计算出数据消息的CRC16校验和。CRC表（W_CRC_TABLE）未在此函数中定义，
 * 需要在其他地方定义和初始化。
 * 
 * 为什么这么做：
 * 使用CRC16校验和可以检测数据在传输过程中是否发生了错误。通过在发送端计算CRC值并将其
 * 附加到消息中，接收端可以重新计算接收到的消息的CRC值，并将其与原始CRC值进行比较，
 * 以确定数据的完整性。
 * 
 * 特殊情况处理：
 * 如果传入的指针pchMessage为nullptr，表示没有有效的数据消息，函数将返回0xFFFF，
 * 表示CRC校验失败。
 */
uint16_t Get_CRC16_Check_Sum(const uint8_t * pchMessage, uint32_t dwLength, uint16_t wCRC)
{
  uint8_t ch_data;

  // 检查输入指针是否有效，如果为nullptr，则返回表示错误的CRC值
  if (pchMessage == nullptr) return 0xFFFF;

  // 遍历数据消息中的每个字节
  while (dwLength--) {
    // 获取当前字节的数据并移动指针到下一个字节
    ch_data = *pchMessage++;
    // 更新CRC值：右移当前CRC值并与CRC表中的相应值进行异或操作
    (wCRC) =
      ((uint16_t)(wCRC) >> 8) ^ W_CRC_TABLE[((uint16_t)(wCRC) ^ (uint16_t)(ch_data)) & 0x00ff];
  }

  // 返回计算得到的CRC16校验和
  return wCRC;
}
/**
  * @brief CRC16 Verify function
  * @param[in] pchMessage : Data to Verify,
  * @param[in] dwLength : Stream length = Data + checksum
  * @return : True or False (CRC Verify Result)
  */
/**
 * 验证CRC16校验和的正确性
 * 
 * @param pchMessage 指向消息的指针，消息末尾包含待验证的CRC16校验和
 * @param dwLength 消息的长度，包括CRC16校验和在内的总字节数
 * @return 返回一个布尔值，表示CRC16校验和是否有效
 * 
 * 此函数通过计算消息的CRC16校验和并与消息末尾提供的CRC16校验和进行比较来验证消息的完整性
 * 如果消息内容或长度不符合预期，校验和不匹配，则返回false；否则返回true
 */
uint32_t Verify_CRC16_Check_Sum(const uint8_t * pchMessage, uint32_t dwLength)
{
  uint16_t w_expected = 0;

  // 检查输入参数的有效性，确保消息指针非空且长度至少能包含CRC16校验和
  if ((pchMessage == nullptr) || (dwLength <= 2)) return false;

  // 计算预期的CRC16校验和，不包括消息中的CRC16校验和部分
  w_expected = Get_CRC16_Check_Sum(pchMessage, dwLength - 2, CRC16_INIT);

  // 比较计算得到的CRC16校验和与消息末尾提供的CRC16校验和是否一致
  return (
    (w_expected & 0xff) == pchMessage[dwLength - 2] &&
    ((w_expected >> 8) & 0xff) == pchMessage[dwLength - 1]);
}
/**
  * @brief Append CRC16 value to the end of the buffer
  * @param[in] pchMessage : Data to Verify,
  * @param[in] dwLength : Stream length = Data + checksum
  * @return none
  */
/**
 * 在消息末尾附加CRC16校验和
 * 
 * 该函数计算给定消息的CRC16校验和，并将其附加到消息的末尾两位。用于确保消息的完整性。
 * 
 * @param pchMessage 指向消息的指针
 * @param dwLength 消息的长度，不包括将要附加的CRC16校验和
 * 
 * 注意：如果消息指针为nullptr或消息长度小于等于2，函数将直接返回，不执行任何操作。
 *       这是因为CRC16校验和需要至少两个字节来存储，而且对空指针或长度不合逻辑的消息进行操作没有意义。
 */
void Append_CRC16_Check_Sum(uint8_t * pchMessage, uint32_t dwLength)
{
  uint16_t w_crc = 0;

  // 检查输入参数的有效性
  if ((pchMessage == nullptr) || (dwLength <= 2)) return;

  // 计算CRC16校验和，不包括最后两个字节，因为这两个字节将用于存储校验和
  w_crc = Get_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(pchMessage), dwLength - 2, CRC16_INIT);

  // 将计算出的CRC16校验和附加到消息的末尾
  pchMessage[dwLength - 2] = (uint8_t)(w_crc & 0x00ff);
  pchMessage[dwLength - 1] = (uint8_t)((w_crc >> 8) & 0x00ff);
}
}  // namespace crc16
