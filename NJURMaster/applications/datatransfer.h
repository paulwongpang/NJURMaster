#ifndef _DATATRANSFER_H_
#define _DATATRANSFER_H_
#include "stm32f4xx.h"
#include "RefereeSystem.h"

void DatatransferTask(u32 _time_sys);
void Usart2_DataPrepare(u8 data);
void Usart3_DataPrepare(u8 data);
void Usart6_DataPrepare(u8* pData);
void ANO_DT_Send_Check(u8 head, u8 check_sum);
void ANO_DT_Send_Status(float angle_rol, float angle_pit, float angle_yaw, s32 alt, u8 fly_model, u8 armed);
void ANO_DT_Send_Senser(s16 a_x,s16 a_y,s16 a_z,s16 g_x,s16 g_y,s16 g_z,s16 m_x,s16 m_y,s16 m_z);
void ANO_DT_Send_RCData(u16 ch0,u16 ch1,u16 ch2,u16 ch3,u16 aux1,u16 aux2,u16 aux3,u16 aux4,u16 aux5,u16 aux6);
void ANO_DT_Send_PID(u8 group,float p1_p,float p1_i,float p1_d,float p2_p,float p2_i,float p2_d,float p3_p,float p3_i,float p3_d);
void ANO_DT_Send_MotoPWM(u16 m_1,u16 m_2,u16 m_3,u16 m_4,u16 m_5,u16 m_6,u16 m_7,u16 m_8);
void RefereeSys_Send_Data(extUserData_t userdata);

//CRC check
unsigned char Get_CRC8_Check_Sum(u8* pMessage, u32 dwLength, u8 ucCRC8);
unsigned char Verify_CRC8_Check_Sum(u8* pMessage, u32 dwLength, u8 ucCRC8);
uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC);
unsigned char Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);
#endif
