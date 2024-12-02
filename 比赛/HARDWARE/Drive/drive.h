#ifndef __DRIVE_H_
#define __DRIVE_H_
#include "sys.h"

void Gate_Open_Zigbee(void);			// ��բբ�ſ���
void Gate_Show_Zigbee(char *Licence);	// ��բϵͳ��־����ʾ����
void LED_Date_Zigbee(uint8_t One, uint8_t Two, uint8_t Three, uint8_t rank);	// LED��ʾ��־����ʾ����
void LED_Dis_Zigbee(uint16_t dis);		// LED��ʾ��־����ʾ�����Ϣ
void Rotate_show_Inf(char* src, char x, char y);	// ������ʾ��־����ʾ��������
void Rotate_Dis_Inf(uint16_t dis);      // ������ʾ��־����ʾ������Ϣ
void YY_Play_Zigbee(char *p);			// ����������־�ﲥ��ָ���ı���Ϣ
void YY_Comm_Zigbee(uint8_t Primary, uint8_t Secondary); // ��������������־�ﲥ��������������
void TFT_Test_Zigbee(char Device,uint8_t Pri,uint8_t Sec1,uint8_t Sec2,uint8_t Sec3);	// TFT��ʾ��־�����ָ��
void TFT_Dis_Zigbee(char Device,uint16_t dis);		// ����TFT��ʾ����ʾ������Ϣ
void TFT_Show_Zigbee(char Device,char *Licence);	// TFT��ʾ����ʾ����
void Garage_Test_Zigbee(char Device,uint8_t Pri,uint8_t Sec1);		// ���峵���־����ƺ���
void Garage_Cont_Zigbee(char Device,uint8_t Floor);		// ���峵�⵽��ָ������
void RegtLight_Inf(uint8_t gear);		// �Զ����ڹ���ǿ�Ⱥ���
uint8_t Light_Inf(uint8_t gear);		// �Զ����ڹ���ǿ�Ⱥ���
void ETC_Get_Zigbee(void);				// ETCϵͳ���
void Test_Zigbee(uint8_t Head,uint8_t Pri,uint8_t Sec1,uint8_t Sec2,uint8_t Sec3);
void Send_Android(uint8_t Pri,uint8_t Sec);

void Car_Go(uint8_t speed, uint16_t temp);   // ����ǰ�� �������ٶ�/����
void Car_Back(uint8_t speed, uint16_t temp); // �������� �������ٶ�/����
void Car_L45(int8_t speed, uint16_t times);		// ����ת ��������תʱ��
void Car_Left(uint8_t speed);       // ������ת �������ٶ�
void Car_R45(int8_t speed, uint16_t tims);		// ����ת ��������תʱ��
void Car_Right(uint8_t speed);       // ������ת �������ٶ�
void Car_Time_Track(uint8_t speed, uint16_t tims);		// С��ѭ�� �������ٶȣ�ʱ��
void Car_Track(uint8_t speed);   // ����ѭ�� �������ٶ�
void CarThread_Go(uint8_t speed, uint16_t temp);   // ����ǰ�� �������ٶ�/����
void CarThread_Track(uint8_t speed);   // ����ѭ�� �������ٶ�
void AGV_Thread(uint8_t mode);		// AGV�Զ���ʻ
void AGV_GetThread(uint8_t mode);		// AGVȫ�Զ���ɱ�־��ȡ
void Track_duo_RFID(uint8_t speed,uint8_t card);



#endif

