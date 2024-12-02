#include "drive.h"
#include <stdio.h>
#include "stm32f4xx.h"
#include "delay.h"
#include "infrared.h"
#include "cba.h"
#include "ultrasonic.h"
#include "canp_hostcom.h"
#include "hard_can.h"
#include "bh1750.h"
#include "syn7318.h"
#include "power_check.h"
#include "can_user.h"
#include "data_base.h"
#include "roadway_check.h"
#include "tba.h"
#include "data_base.h"
#include "swopt_drv.h"
#include "uart_a72.h"
#include "Can_check.h"
#include "delay.h"
#include "can_user.h"
#include "Timer.h"
#include "Rc522.h"
#include "string.h"


/*************************** �Զ������ *******************************/
static uint16_t AGV_Time = 0, AGV_next = 0;	// ʱ����
static uint8_t AGV_Zigbee[8]={0x55,0x02,0x00,0x00,0x00,0x00,0x00,0xBB};
static uint8_t time[10] = {"time:00s\n"};
static uint8_t Zigbee[8];           // Zigbee�������ݻ���
static uint8_t Infrared[6];         // ���ⷢ�����ݻ���
static uint8_t YY_Init[5] = {0xFD, 0x00, 0x00, 0x01, 0x01};
static uint8_t YY2_Init[8] = {0xAF, 0x06, 0x01, 0x02, 0x00, 0x00, 0x01, 0xBB};
uint16_t tim_a,tim_b;

/**********************************************************************/

/**********************************************************************
 * �� �� �� ��  ��բϵͳբ�ſ�������
 * ��    �� ��  ��
 * �� �� ֵ ��  ��
 * ��    �� ��  Gate_Open_Zigbee();     // ��բբ�� -> ����
**********************************************************************/
void Gate_Open_Zigbee(void)
{
    Send_ZigbeeData_To_Fifo(Gate_Open, 8);		// ��բ -> ����
	delay_ms(300);
	Send_ZigbeeData_To_Fifo(Gate_GetStatus, 8);	// ��բ ״̬��ѯ
	delay_ms(100);
#if 0
	tim_a = 0;
	tim_b = 0;
    Stop_Flag = 0;
    while(Stop_Flag != 0x05)
    {
        delay_ms(1);
        tim_a++;
        if(tim_a >= 500)
        {
            tim_a = 0;
			tim_b++;
			if (tim_b >= 5) { break;}
			Send_ZigbeeData_To_Fifo(Gate_Open, 8);		// ��բ -> ����
			delay_ms(300);
			Send_ZigbeeData_To_Fifo(Gate_GetStatus, 8);	// ��բ ״̬��ѯ
			delay_ms(100);
        }
        if(Zigbee_Rx_flag)      // �ж�Zigbee���ݻش�
        {
			Zigbee_Rx_flag = 0;
            if(Zigb_Rx_Buf[1] == 0x03)      // ��բ
            {
                if(Zigb_Rx_Buf[2] == 0x01)
                {
                    Stop_Flag = Zigb_Rx_Buf[4];
                }
            }
        }
    }
#endif
}

/**********************************************************************
 * �� �� �� ��  ��բϵͳ��־����ʾ����
 * ��    �� ��  *Licence -> �������ݣ�ASICC��
 * �� �� ֵ ��  ��
 * ��    �� ��  Gate_Show_Zigbee("A123B4");
**********************************************************************/
void Gate_Show_Zigbee(char *Licence)
{
    Zigbee[0] = 0x55;
    Zigbee[1] = 0x03;
    Zigbee[2] = 0x10;
    Zigbee[3] = *(Licence + 0);     // �������ݡ�1��
    Zigbee[4] = *(Licence + 1);     // �������ݡ�2��
    Zigbee[5] = *(Licence + 2);     // �������ݡ�3��
    Zigbee[6] = (Zigbee[2] + Zigbee[3] + Zigbee[4] + Zigbee[5]) % 256;
    Zigbee[7] = 0xBB;
    Send_ZigbeeData_To_Fifo(Zigbee, 8);     // ����Zigbee����
    delay_ms(400);
    Zigbee[2] = 0x11;
    Zigbee[3] = *(Licence + 3);     // �������ݡ�4��
    Zigbee[4] = *(Licence + 4);     // �������ݡ�5��
    Zigbee[5] = *(Licence + 5);     // �������ݡ�6��
    Zigbee[6] = (Zigbee[2] + Zigbee[3] + Zigbee[4] + Zigbee[5]) % 256;
    Zigbee[7] = 0xBB;
    Send_ZigbeeData_To_Fifo(Zigbee, 8);     // ����Zigbee����
    delay_ms(100);
}

/**********************************************************************
 * �� �� �� ��  LED��ʾ��־����ʾ����
 * ��    �� ��  One,Two,Three   ���ݣ�ʮ��������ʾ��ʽ��
                rank    1 -> ��һ���������ʾ����
                        2 -> �ڶ����������ʾ���ݣ�Ĭ�ϣ�
 * �� �� ֵ ��  ��
 * ��    �� ��  LED_Date_Zigbee(0x12,0x34,0x56,0x01);
**********************************************************************/
void LED_Date_Zigbee(uint8_t One, uint8_t Two, uint8_t Three, uint8_t rank)
{
    Zigbee[0] = 0x55;
    Zigbee[1] = 0x04;
    if(rank == 1)  Zigbee[2] = 0x01;
    else  Zigbee[2] = 0x02;
    Zigbee[3] = One;
    Zigbee[4] = Two;
    Zigbee[5] = Three;
    Zigbee[6] = (Zigbee[2] + Zigbee[3] + Zigbee[4] + Zigbee[5]) % 256;
    Zigbee[7] = 0xBB;
    Send_ZigbeeData_To_Fifo(Zigbee, 8);
    delay_ms(100);
}

/**********************************************************************
 * �� �� �� ��  LED��ʾ��־����ʾ�����Ϣ
 * ��    �� ��  dis ����ֵ
 * �� �� ֵ ��  ��
 * ��    �� ��  LED_Dis_Zigbee(123);
**********************************************************************/
void LED_Dis_Zigbee(uint16_t dis)
{
    Zigbee[0] = 0x55;
    Zigbee[1] = 0x04;
    Zigbee[2] = 0x04;
    Zigbee[3] = 0x00;
    Zigbee[4] = dis / 100 % 10;
    Zigbee[5] = ((dis / 10 % 10) * 16 + dis % 10);
    Zigbee[6] = (Zigbee[2] + Zigbee[3] + Zigbee[4] + Zigbee[5]) % 256;
    Zigbee[7] = 0xBB;
    Send_ZigbeeData_To_Fifo(Zigbee, 8);  //����Zigbee����
    delay_ms(100);
}

/**********************************************************************
 * �� �� �� ��  ������ʾ��־����ʾ��������
 * ��    �� ��  *src    �������ݣ�ASICC��
                x,y     ������Ϣ
 * �� �� ֵ ��  ��
 * ��    �� ��  Rotate_show_Inf("A123B4",'C','5');
**********************************************************************/
void Rotate_show_Inf(char* src, char x, char y)
{
    Infrared[0] = 0xFF;			// ��ʼλ
    Infrared[1] = 0x20;			// ģʽ
    Infrared[2] = *(src + 0);	// ���ݡ�1��
    Infrared[3] = *(src + 1);	// ���ݡ�2��
    Infrared[4] = *(src + 2);	// ���ݡ�3��
    Infrared[5] = *(src + 3);	// ���ݡ�4��
    Infrared_Send(Infrared, 6);
    delay_ms(500);
    Infrared[1] = 0x10;			// ģʽ
    Infrared[2] = *(src + 4);	// ���ݡ�1��
    Infrared[3] = *(src + 5);	// ���ݡ�2��
    Infrared[4] = x;			// ���ݡ�3��
    Infrared[5] = y;			// ���ݡ�4��
    Infrared_Send(Infrared, 6);
    delay_ms(10);
}

/**********************************************************************
 * �� �� �� ��  ������ʾ��־����ʾ������Ϣ����λ��ms��
 * ��    �� ��  dis  �����Ϣ���������룩
 * �� �� ֵ ��  ��
 * ��    �� ��  Rotate_Dis_Inf(123);
**********************************************************************/
void Rotate_Dis_Inf(uint16_t dis)
{
    uint16_t csb = dis; //���泬��������ֵ

    csb += 5;   //��������
    Infrared[0] = 0xFF;
    Infrared[1] = 0x11; //��ʾ����ģʽ
    Infrared[2] = 0x30 + (uint8_t)(csb / 100 % 10); //����ʮλ--cm
    Infrared[3] = 0x30 + (uint8_t)(csb / 10 % 10); //�����λ--cm
    Infrared[4] = 0x00;
    Infrared[5] = 0x00;
    Infrared_Send(Infrared, 6);
    delay_ms(100);
}

/**********************************************************************
 * �� �� �� ��  ��������������־�ﲥ��ָ���ı���Ϣ
 * ��    �� ��  *p  --> ��Ҫ���͵�����
 * �� �� ֵ ��  ��
 * ��    �� ��  YY_Play_Zigbee("������ӭ��");
**********************************************************************/
void YY1_Play_Zigbee(char *p)
{
    uint16_t p_len = strlen(p);             // �ı�����

    YY_Init[1] = 0xff & ((p_len + 2) >> 8); // ���������ȸ߰�λ
    YY_Init[2] = 0xff & (p_len + 2);        // ���������ȵͰ�λ
    Send_ZigbeeData_To_Fifo(YY_Init, 5);
    Send_ZigbeeData_To_Fifo((uint8_t *)p, p_len);
    delay_ms(100);
}

/**********************************************************************
 * �� �� �� ��  ��������������־�ﲥ��ָ���ı���Ϣ
 * ��    �� ��  *p  --> ��Ҫ���͵�����
 * �� �� ֵ ��  ��
 * ��    �� ��  YY_Play_Zigbee("������ӭ��");
**********************************************************************/
void YY2_Play_Zigbee(uint8_t One)
{
		
		switch(One)
		{
			case 1 : YY2_Init[2] = 0x01; break;
			case 2 : YY2_Init[2] = 0x02; break;
			case 3 : YY2_Init[2] = 0x03; break;
			case 4 : YY2_Init[2] = 0x04; break;
			case 5 : YY2_Init[2] = 0x05; break;
			case 6 : YY2_Init[2] = 0x06; break;
			case 7 : YY2_Init[2] = 0x07; break;
			default: YY2_Init[2] = 0x01; break;
				
		}
			
    Send_ZigbeeData_To_Fifo(YY2_Init, 8);
    Send_ZigbeeData_To_Fifo(YY2_Init, 8);
    delay_ms(100);
}

/**********************************************************************
 * �� �� �� ��  ��������������־�ﲥ��������������
 * ��    �� ��  Primary   -> ��ָ��
                Secondary -> ��ְ��
                �����¼1
 * �� �� ֵ ��  ��
 * ��    �� ��  YY_Comm_Zigbee(0x20, 0x01);     // �������������������

��¼1��
-----------------------------------------------------------------------
| Primary | Secondary | ˵��
|---------|-----------|------------------------------------------------
|  0x10   |  0x02     | ����ת��
|         |  0x03     | ��ֹ��ת
|         |  0x04     | �����ʻ
|         |  0x05     | ���б���
|         |  0x06     | ԭ�ص�ͷ
|---------|-----------|------------------------------------------------
|  0x20   |  0x01     | ���ָ��
|---------|-----------|------------------------------------------------
***********************************************************************/
void YY1_Comm_Zigbee(uint8_t Primary, uint8_t Secondary)
{
    Zigbee[0] = 0x55;
    Zigbee[1] = 0x06;
    Zigbee[2] = Primary;
    Zigbee[3] = Secondary;
    Zigbee[4] = 0x00;
    Zigbee[5] = 0x00;
    Zigbee[6] = (Zigbee[2] + Zigbee[3] + Zigbee[4] + Zigbee[5]) % 256;
    Zigbee[7] = 0xBB;
    Send_ZigbeeData_To_Fifo(Zigbee, 8);
}

/********************************************************************************
 * �� �� �� ��  TFT��ʾ��־�����ָ��
 * ��    �� ��  Device -> ѡ���豸
				Pri	   -> ��ָ��
                Sec1   -> ��ְ�1��
				Sec2   -> ��ְ�2��
				Sec3   -> ��ְ�3��
                �����¼1
 * �� �� ֵ ��  ��
 * ��    �� ��  TFT_Test_Zigbee('A',0x40,0xA1,0xB2,0xC3);    // TFT��ʾ����ʾͼ����Ϣ

��¼1��
--------------------------------------------------------------------------------
| ��ָ�� | ��ָ��[1] | ��ָ��[2] | ��ָ��[3] |			˵��
|--------|-----------|-----------|-----------|-----------------------------------
|  0x10  |	 0x00	 | 0x01~0x20 |   0x00    | �ɵڶ���ָ��ָ����ʾ����ͼƬ
|        |	 0x01    |	 0x00	 |   0x00    | ͼƬ���Ϸ�ҳ
|        |   0x02    |	 0x00	 |   0x00    | ͼƬ���·�ҳ
|		 |   0x03    |	 0x00    |   0x00    | ͼƬ�Զ����·�ҳ��ʾ�����ʱ�� 10S
|--------|-----------|-----------|-----------|-----------------------------------
|  0x20  |	 0xXX    |	 0xXX	 |   0xXX	 | ����ǰ��λ���ݣ�ASCII��
|--------|-----------|-----------|-----------|-----------------------------------
|  0x21  |	 0xXX	 |	 0xXX	 |   0xXX	 | ���ƺ���λ���ݣ�ASCII��
|--------|-----------|-----------|-----------|-----------------------------------
|  0x30  |	 0x00	 |	 0x00	 |   0x00	 | ��ʱģʽ�ر�
|	 	 |	 0x01	 |	 0x00 	 |   0x00	 | ��ʱģʽ��
| 		 |	 0x02	 |	 0x00	 |   0x00	 | ��ʱģʽ����
|--------|-----------|-----------|-----------|-----------------------------------
|  0x40  |	 0xXX	 |	 0xXX    |   0xXX	 | ��λ��ʾ���ݣ�HEX��ʽ��
|--------|-----------|-----------|-----------|-----------------------------------
|  0x50  |	 0x00	 |	 0x0X	 |   0xXX	 | ������ʾģʽ��ʮ���ƣ�
--------------------------------------------------------------------------------
********************************************************************************/
void TFT_Test_Zigbee(char Device,uint8_t Pri,uint8_t Sec1,uint8_t Sec2,uint8_t Sec3)
{
	Zigbee[0] = 0x55;
	if (Device == 'B') { Zigbee[1] = 0x08; }
	else { Zigbee[1] = 0x0B; }
	Zigbee[2] = Pri;
	Zigbee[3] = Sec1;
	Zigbee[4] = Sec2;
	Zigbee[5] = Sec3;
	Zigbee[6] = (Zigbee[2]+Zigbee[3]+Zigbee[4]+Zigbee[5])%256;
	Zigbee[7] = 0xBB;
	Send_ZigbeeData_To_Fifo(Zigbee, 8);
	delay_ms(100);
}

/**********************************************************************
 * �� �� �� ��  ����TFT��ʾ����ʾ������Ϣ
 * ��    �� ��  dis	����ֵ
 * �� �� ֵ ��  ��
 * ��    �� ��  TFT_Dis_Zigbee('A',123);
**********************************************************************/
void TFT_Dis_Zigbee(char Device,uint16_t dis)
{
	Zigbee[0] = 0x55;
	if (Device == 'B') { Zigbee[1] = 0x08; }
	else { Zigbee[1] = 0x0B; }
	Zigbee[2] = 0x50;
	Zigbee[3] = 0x00;
	Zigbee[4] = dis/100%10;
	Zigbee[5] = ((dis/10%10)*16+dis%10);
	Zigbee[6] = (Zigbee[2]+Zigbee[3]+Zigbee[4]+Zigbee[5])%256;
	Zigbee[7] = 0xBB;
	Send_ZigbeeData_To_Fifo(Zigbee, 8);  //����Zigbee����
	delay_ms(100);
}


/**********************************************************************
 * �� �� �� ��  TFT��ʾ����ʾ����
 * ��    �� ��  *Licence -> �������ݣ�ASICC��
 * �� �� ֵ ��  ��
 * ��    �� ��  TFT_Show_Zigbee('A',"A123B4");
**********************************************************************/
void TFT_Show_Zigbee(char Device,char *Licence)
{
    Zigbee[0] = 0x55;
    if (Device == 'B') { Zigbee[1] = 0x08; }
	else { Zigbee[1] = 0x0B; }
    Zigbee[2] = 0x20;
    Zigbee[3] = *(Licence + 0);     // �������ݡ�1��
    Zigbee[4] = *(Licence + 1);     // �������ݡ�2��
    Zigbee[5] = *(Licence + 2);     // �������ݡ�3��
    Zigbee[6] = (Zigbee[2] + Zigbee[3] + Zigbee[4] + Zigbee[5]) % 256;
    Zigbee[7] = 0xBB;
    Send_ZigbeeData_To_Fifo(Zigbee, 8);     // ����Zigbee����
    delay_ms(500);
    Zigbee[2] = 0x21;
    Zigbee[3] = *(Licence + 3);     // �������ݡ�4��
    Zigbee[4] = *(Licence + 4);     // �������ݡ�5��
    Zigbee[5] = *(Licence + 5);     // �������ݡ�6��
    Zigbee[6] = (Zigbee[2] + Zigbee[3] + Zigbee[4] + Zigbee[5]) % 256;
    Zigbee[7] = 0xBB;
    Send_ZigbeeData_To_Fifo(Zigbee, 8);     // ����Zigbee����
    delay_ms(100);
}

/********************************************************************************
 * �� �� �� ��  ���峵���־����ƺ���
 * ��    �� ��  Device -> ѡ���豸
				Pri	   -> ��ָ��
                Sec1   -> ��ְ�1��
                �����¼1
 * �� �� ֵ ��  ��
 * ��    �� ��  Garage_Test_Zigbee('A',0x01,0x01);    // ���峵��A �����һ��

��¼1��
--------------------------------------------------------------------------------
| ��ָ�� | ��ָ��[1] | ��ָ��[2] | ��ָ��[3] |			˵��
|--------|-----------|-----------|-----------|----------------------------------
|  0x01  |   0x01    |   0x00    |   0x00    | �����һ��
|        |   0x02    |   0x00    |   0x00    | ����ڶ���
|        |   0x03    |   0x00    |   0x00    | ���������
|        |   0x04    |   0x00    |   0x00    | ������Ĳ�
|--------|-----------|-----------|-----------|----------------------------------
|  0x02  |   0x01    |   0x00    |   0x00    | ���󷵻س���λ�ڵڼ���
|        |   0x02    |   0x00    |   0x00    | ���󷵻�ǰ������״̬
|--------|-----------|-----------|-----------|----------------------------------
|  0x03  |   0x01    |   0x01    |   0x00    | ���س���λ�ڵ�һ��
|        |           |   0x02    |   0x00    | ���س���λ�ڵڶ���
|        |           |   0x03    |   0x00    | ���س���λ�ڵ�����
|        |           |   0x04    |   0x00    | ���س���λ�ڵ��Ĳ�
|        |-----------|-----------|-----------|----------------------------------
|        |   0x02    |0x01 ����  |0x01 ����  | ���س���ǰ������Թ�״̬
|        |           |0x02 δ����|0x02 δ����| 
|        |           | - ǰ�� -  | - ��� -  | 
--------------------------------------------------------------------------------
********************************************************************************/
void Garage_Test_Zigbee(char Device,uint8_t Pri,uint8_t Sec1)
{
	Zigbee[0] = 0x55;
	if (Device == 'B') { Zigbee[1] = 0x08; }
	else { Zigbee[1] = 0x0D; }
	Zigbee[2] = Pri;
	Zigbee[3] = Sec1;
	Zigbee[4] = 0x00;
	Zigbee[5] = 0x00;
	Zigbee[6] = (Zigbee[2]+Zigbee[3]+Zigbee[4]+Zigbee[5])%256;
	Zigbee[7] = 0xBB;
	Send_ZigbeeData_To_Fifo(Zigbee, 8);
	delay_ms(100);
}

/**********************************************************************
 * �� �� �� ��  ���峵�⵽��ָ������
 * ��    �� ��  Device -> ѡ���豸
				Floor  -> ����
 * �� �� ֵ ��  ��
 * ��    �� ��  Garage_Cont_Zigbee('A', 1);	// ���峵��A �����һ��
**********************************************************************/
void Garage_Cont_Zigbee(char Device,uint8_t Floor)
{
	tim_a = 150;
	tim_b = 0;
	while(1)
	{
		tim_a++;
		delay_ms(10);
		if(!(tim_a%80))
		{
			if (Device == 'B')
			{
				Send_ZigbeeData_To_Fifo(GarageB_Get_Floor, 8);	// ���󳵿�B����λ�ڵڼ���
			} else 
			{
				Send_ZigbeeData_To_Fifo(GarageA_Get_Floor, 8);	// ���󳵿�A����λ�ڵڼ���
			}
		}
		if(tim_a > 200)
		{
			tim_a = 0;
			tim_b++;
			if(tim_b >= 15) { break; }
			
			if ((Device == 'A') && (Floor == 1))
			{
				Send_ZigbeeData_To_Fifo(GarageA_To1, 8);	// ���峵��A �����1��
			} else if ((Device == 'A') && (Floor == 2))
			{
				Send_ZigbeeData_To_Fifo(GarageA_To2, 8);	// ���峵��A �����2��
			} else if ((Device == 'A') && (Floor == 3))
			{
				Send_ZigbeeData_To_Fifo(GarageA_To3, 8);	// ���峵��A �����3��
			} else if ((Device == 'A') && (Floor == 4))
			{
				Send_ZigbeeData_To_Fifo(GarageA_To4, 8);	// ���峵��A �����4��
			} else if ((Device == 'B') && (Floor == 1))
			{
				Send_ZigbeeData_To_Fifo(GarageB_To1, 8);	// ���峵��B �����1��
			} else if ((Device == 'B') && (Floor == 2))
			{
				Send_ZigbeeData_To_Fifo(GarageB_To2, 8);	// ���峵��B �����2��
			} else if ((Device == 'B') && (Floor == 3))
			{
				Send_ZigbeeData_To_Fifo(GarageB_To3, 8);	// ���峵��B �����3��
			} else if ((Device == 'B') && (Floor == 4))
			{
				Send_ZigbeeData_To_Fifo(GarageA_To4, 8);	// ���峵��B �����4��
			} else
			{
				Send_ZigbeeData_To_Fifo(GarageA_To1, 8);	// ���峵��A �����1��
			}
		}
		if(Zigbee_Rx_flag == 1)	 //zigbee������Ϣ
		{
			Zigbee_Rx_flag =0;
			if((Zigb_Rx_Buf[0] == 0x55) && (Zigb_Rx_Buf[1] == 0x0D))
			{
				if((Zigb_Rx_Buf[2] == 0x03) && (Zigb_Rx_Buf[3] == 0x01))
				{
					if(Zigb_Rx_Buf[4] == Floor)
					{
						break;
					}
				}
			}
		}
	}
}

/**********************************************************************
 * �� �� �� ��  ZigBeeͨѶ�Զ��巢�ͺ���
 * ��    �� ��  Head -> ��ͷ
				Pri  -> ��ָ��
				Sec1,Sec2,Sec3	-> ��ְ��
 * �� �� ֵ ��  ��
 * ��    �� ��  Test_Zigbee(0x03,0x01,0x01,0x00,0x00);
**********************************************************************/
void Test_Zigbee(uint8_t Head,uint8_t Pri,uint8_t Sec1,uint8_t Sec2,uint8_t Sec3)
{
	Zigbee[0] = 0x55;
	Zigbee[1] = Head;
	Zigbee[2] = Pri;
	Zigbee[3] = Sec1;
	Zigbee[4] = Sec2;
	Zigbee[5] = Sec3;
	Zigbee[6] = (Zigbee[2]+Zigbee[3]+Zigbee[4]+Zigbee[5])%256;
	Zigbee[7] = 0xBB;
	Send_ZigbeeData_To_Fifo(Zigbee, 8);
	delay_ms(100);
}

/*****************************************************************
 * �� �� �� ��	�Զ����ڹ���ǿ�Ⱥ���
 * ��    �� ��	gear -> Ŀ�굲λֵ
 * �� �� ֵ ��	gear_init -> ��ʼ��λֵ
 * ��     ����	gear_init = Light_Inf(3);	// ����·�Ƶ���3��
*****************************************************************/
uint8_t Light_Inf(uint8_t gear)
{
	uint8_t i;
	uint8_t gear_init = 0;	// ��ʼ��λֵ
	uint16_t array[2];		//������ѧϰ�Ĺ⵵λ����
	
	if((gear > 0) && (gear < 5))
	{
		delay_ms(100);
		array[0] = Get_Bh_Value();		//��ǿ�ȴ�����	
		for(i=0; i<4; i++)
		{
			gear_init++;
			Infrared_Send(Light_plus1,4);
			delay_ms(500);
			delay_ms(500);
			delay_ms(500);
			array[1] = Get_Bh_Value();		//��ǿ�ȴ�����
			if (array[0] < array[1]) 
			{
				array[0] = array[1];
				array[1] = 0;
			}
			else
			{
				gear_init = 5 - gear_init;
				break;
			}
		}
		if(gear==2)
		{
			Infrared_Send(Light_plus1,4);	//��Դ��λ��1
		}else if(gear==3)
		{
			Infrared_Send(Light_plus2,4);	//��Դ��λ��2
		}else if(gear==4)
		{
			Infrared_Send(Light_plus3,4);	//��Դ��λ��3
		}
	}
	return gear_init;
}

/*****************************************************************
 * �� �� �� ��	ETCϵͳ���
 * ��    �� ��	��
 * �� �� ֵ ��	��
 * ��     ����	ETC_Get_Zigbee();	// ETCϵͳ���
*****************************************************************/
void ETC_Get_Zigbee(void)
{
	tim_a = 0;
	tim_b = 0;
	Stop_Flag = 0;
	while(Stop_Flag != 0x06)
	{
		tim_a++;
		delay_ms(1);
		if(tim_a > 800)
		{
			tim_a = 0;
			tim_b++;
			if(tim_b < 6)
			{
				Car_Back(40, 100);	// ����
				delay_ms(100);
				Car_Go(40, 110);	//С��ǰ��
			}
			else
			{
			//	SYN_TTS("ETCϵͳʶ��ʧ�ܣ�ǿ��ͨ��");
				break;
			}
		}
		
		if(Zigbee_Rx_flag == 1)	 // zigbee����
		{
			Zigbee_Rx_flag = 0;
			delay_us(5);
			if(Zigb_Rx_Buf[1] == 0x0C)		// ETCϵͳ
			{
				if(Zigb_Rx_Buf[2] == 0x01)
				{
					if(Zigb_Rx_Buf[3] == 0x01)
					{
						Stop_Flag = Zigb_Rx_Buf[4];	  // ETCբ�ſ����ɹ�
					}
				}
			}
		}
	}
}

void AGV_Thread(uint8_t mode)		// AGV�Զ���ʻ
{
	AGV_next = 0;
	AGV_Time = 60000;
	AGV_Zigbee[2] = 0xA0;
	AGV_Zigbee[3] = mode;
	AGV_Zigbee[4] = 0x00;
	AGV_Zigbee[5] = 0x00;
	AGV_Zigbee[6] = (AGV_Zigbee[2]+AGV_Zigbee[3]+AGV_Zigbee[4]+AGV_Zigbee[5])%256;
	while(1)
	{
		AGV_Time++;
		delay_ms(10);
		if(AGV_Time > 5000)
		{
			AGV_Time = 0;
			AGV_next++;
			Send_ZigbeeData_To_Fifo(AGV_Zigbee, 8);
			delay_ms(100);
			Send_ZigbeeData_To_Fifo(AGV_Zigbee, 8);
			delay_ms(100);
			Send_InfoData_To_Fifo("AGV_Thread\n",12);	// �ϴ�������Ϣ
			delay_ms(100);
			if (AGV_next > 3)
			{
				break;
			}
		}
		if(Zigbee_Rx_flag == 1)	 //zigbee������Ϣ
		{
			Zigbee_Rx_flag =0;
			if((Zigb_Rx_Buf[0] == 0x55) && (Zigb_Rx_Buf[1] == 0x02))
			{
				if(Zigb_Rx_Buf[2] == 0x08)
				{
					time[5] = AGV_Time/1000%10+0x30;
					time[6] = AGV_Time/100%10+0x30;
					Send_InfoData_To_Fifo((char *)time, 10);	// �ϴ�������Ϣ
					break;
				}
			}
		}
	}
}

void AGV_GetThread(uint8_t mode)		// AGVȫ�Զ���ɱ�־��ȡ
{
	AGV_Time = 0;
	while(1)
	{
		AGV_Time++;
		delay_ms(1);
		if(AGV_Time > 60000)
		{
			AGV_Time = 0;
			break;
		}
		if(Zigbee_Rx_flag == 1)	 //zigbee������Ϣ
		{
			Zigbee_Rx_flag =0;
			if((Zigb_Rx_Buf[0] == 0x55) && (Zigb_Rx_Buf[1] == 0x02))
			{
				if(Zigb_Rx_Buf[2] == mode)
				{
					time[5] = AGV_Time/10000%10+0x30;
					time[6] = AGV_Time/1000%10+0x30;
					Send_InfoData_To_Fifo((char *)time, 10);	// �ϴ�������Ϣ
					break;
				}
			}
		}
	}
}

/*********************************************************************************
 * �� �� �� ��	������Android�ն˷�������������
 * ��    �� ��	Pri  -> ��ָ��
				Sec  -> ��ְ��
				�����¼1����¼2
 * �� �� ֵ ��	��
 * ��    �� ��	Send_Android(0xA1, 0x01);		// ��������ͷԤ��λ1


��¼1��������Android�ն˷��͵�������ָ��ṹ
--------------------------------------------------------
| ��ָ�� | ��ָ�� |   ˵��
|--------|--------|-------------------------------------
|  0xA0  |  0x00  | ����ȫ�Զ�
|--------|--------|-------------------------------------
|  0xA1  |  0x01  | ��������ͷԤ��Ϊ1
|        |  0x02  | ��������ͷԤ��Ϊ2
|        |  0x03  | ��������ͷԤ��Ϊ3
|        |  0x04  | ��������ͷԤ��Ϊ4
|        |  0x05  | ��������ͷԤ��Ϊ5
|--------|--------|-------------------------------------
|  0xA2  |  0x00  | ���ö�ά��ʶ��
|--------|--------|-------------------------------------
|  0xA3  |  0x00  | ���ý�ͨ��ʶ��
|--------|--------|-------------------------------------
|  0xA4  |  0x00  | ���ó���ʶ��
|--------|--------|-------------------------------------
|  0xA4  |  0x00  | ����ͼ��ʶ��
--------------------------------------------------------

��¼2��Android�ն�����������ʶ����ָ��ṹ
-------------------------------------------------------------------------------
| ��ָ�� | ��ָ��[1] | ��ָ��[2] | ��ָ��[3] |		˵��
|--------|-----------|-----------|-----------|---------------------------------
|  0xA2  |   0x00    |   0x00    |   0x00	   | ��ά��ʶ��ɹ�
|--------|-----------|-----------|-----------|---------------------------------
|  0xA3  |   0x01    |   0x00    |   0x00	   | ��ͨ��ʶ��ɹ�����ɫ��
|        |   0x02    |   0x00    |   0x00	   | ��ͨ��ʶ��ɹ�����ɫ��
|        |   0x03    |   0x00    |   0x00	   | ��ͨ��ʶ��ɹ�����ɫ��
|--------|-----------|-----------|-----------|---------------------------------
|  0xA4  |   0xXX    |   0xXX    |   0xXX	   | ����ʶ��ɹ� ����ǰ��λASICC
|--------|-----------|-----------|-----------|---------------------------------
|  0xA5  |   0xXX    |   0xXX    |   0xXX	 | ����ʶ��ɹ� ���ƺ���λASICC
|--------|-----------|-----------|-----------|---------------------------------
|  0xA6  |   0xXX    |   0xXX    |   0xXX	 | ͼ��ʶ�� 
|        |�������Σ� | ��Բ�Σ�  | �����Σ�  | 
-------------------------------------------------------------------------------
*********************************************************************************/
void Send_Android(uint8_t Pri,uint8_t Sec)
{
	Principal_Tab[0] = 0x55;
	Principal_Tab[1] = 0xAA;
	Principal_Tab[2] = Pri;
	Principal_Tab[3] = Sec;
	Send_WifiData_To_Fifo(Principal_Tab, 4);   // ͨ��Wifi�ϴ���������
	UartA72_TxClear();
	UartA72_TxAddStr(Principal_Tab, 4);        // ͨ�������ϴ���������
	UartA72_TxStart();//����
}


/**************************** ���п���ָ�� ****************************/
void Car_Go(uint8_t speed, uint16_t temp)   // ����ǰ�� �������ٶ�/����
{
    Roadway_mp_syn();       // ����ͬ��
    Stop_Flag = 0;          // ����״̬��־λ
    Go_Flag = 1;            // ǰ����־λ
    wheel_L_Flag = 0;       // ��ת��־λ
    wheel_R_Flag = 0;       // ��ת��־λ
    wheel_Nav_Flag = 0;     // ������ת��־λ
    Back_Flag = 0;          // ���˱�־λ
    Track_Flag = 0;         // ѭ����־λ
    temp_MP = temp;         // ����ֵ
    Car_Spend = speed;      // �ٶ�ֵ
    Control(Car_Spend, Car_Spend);  // �����������
    while(Stop_Flag != 0x03);       // �ȴ�ǰ�����
}

void Car_Back(uint8_t speed, uint16_t temp) // �������� �������ٶ�/����
{
    Roadway_mp_syn();       // ����ͬ��
    Stop_Flag = 0;          // ����״̬��־λ
    Go_Flag = 0;            // ǰ����־λ
    wheel_L_Flag = 0;       // ��ת��־λ
    wheel_R_Flag = 0;       // ��ת��־λ
    wheel_Nav_Flag = 0;     // ������ת��־λ
    Back_Flag = 1;          // ���˱�־λ
    Track_Flag = 0;         // ѭ����־λ
    temp_MP = temp;         // ����ֵ
    Car_Spend = speed;      // �ٶ�ֵ
    Control(-Car_Spend, -Car_Spend); // �����������
    while(Stop_Flag != 0x03);       // �ȴ��������
}

void Car_Left(uint8_t speed)       // ������ת �������ٶ�
{
	delay_ms(100);
    Stop_Flag = 0;          // ����״̬��־λ
    Go_Flag = 0;            // ǰ����־λ
    wheel_L_Flag = 1;       // ��ת��־λ
    wheel_R_Flag = 0;       // ��ת��־λ
    wheel_Nav_Flag = 0;     // ������ת��־λ
    Back_Flag = 0;          // ���˱�־λ
    Track_Flag = 0;         // ѭ����־λ
    Car_Spend = speed;      // �ٶ�ֵ
    Control(-Car_Spend, Car_Spend); // �����������
    while(Stop_Flag != 0x02);       // �ȴ���ת���
	//delay_ms(100);
}

void Car_Right(uint8_t speed)       // ������ת �������ٶ�
{
    delay_ms(100);
    Stop_Flag = 0;          // ����״̬��־λ
    Go_Flag = 0;            // ǰ����־λ
    wheel_L_Flag = 0;       // ��ת��־λ
    wheel_R_Flag = 1;       // ��ת��־λ
    wheel_Nav_Flag = 0;     // ������ת��־λ
    Back_Flag = 0;          // ���˱�־λ
    Track_Flag = 0;         // ѭ����־λ
    Car_Spend = speed;      // �ٶ�ֵ
    Control(Car_Spend, -Car_Spend); // �����������
    while(Stop_Flag != 0x02);       // �ȴ���ת���
    delay_ms(100);
}

void Car_Track(uint8_t speed)   // ����ѭ�� �������ٶ�
{
    Stop_Flag = 0;          // ����״̬��־λ
    Go_Flag = 0;            // ǰ����־λ
    wheel_L_Flag = 0;       // ��ת��־λ
    wheel_R_Flag = 0;       // ��ת��־λ
    wheel_Nav_Flag = 0;     // ������ת��־λ
    Back_Flag = 0;          // ���˱�־λ
    Track_Flag = 1;         // ѭ����־λ
    Car_Spend = speed;      // �ٶ�ֵ
    Control(Car_Spend, Car_Spend);  // ����������� *
    while(Stop_Flag != 0x01);       // �ȴ�ѭ�����
}
void Track_duoy_RFID(uint8_t speed)   // 
{
    Stop_Flag = 0;          // ����״̬��־λ
    Go_Flag = 0;            // ǰ����־λ
    wheel_L_Flag = 0;       // ��ת��־λ
    wheel_R_Flag = 0;       // ��ת��־λ
    wheel_Nav_Flag = 0;     // ������ת��־λ
    Back_Flag = 0;          // ���˱�־λ
    Track_Flag = 1;         // ѭ����־λ
    Car_Spend = speed;      // �ٶ�ֵ
    Control(Car_Spend, Car_Spend);  // ����������� *
    
}
void Track_duo_RFID(uint8_t speed,uint8_t card)//��������   ������ѭ���ٶȣ���Ҫ��ȡ�����ݿ�
{
  Track_duoy_RFID(speed);
  delay_ms(300);
  while(Stop_Flag != 0x01)
  {
     if(PcdRequest(PICC_REQALL,CT) == MI_OK)   //��Ѱ��RFID
     {
         Send_UpMotor(0 ,0);
         wheel_L_Flag =0;
         wheel_R_Flag = 0;
         Go_Flag = 0;
         Back_Flag = 0;
         Track_Flag = 0;
         Stop_Flag = 0;
         delay_ms(500); 
         for(char i=0;i<10;i++)
         {
            MP_SPK = 1;
            delay_ms(80);
            MP_SPK = 0;
            delay_ms(10);
         }
         delay_ms(500);
         RC522(card,RFID_Read);//����
         delay_ms(400);
         delay_ms(400);
         Track_duoy_RFID(speed);
			
  }   
 }
}

void Car_L45(int8_t speed, uint16_t times)		// ����ת ��������תʱ��
{
	delay_ms(100);
	Send_UpMotor(-speed ,speed);	
	delay_ms(times);
	Send_UpMotor(0 ,0);
	delay_ms(100);
}

void Car_R45(int8_t speed, uint16_t tims)		// ����ת ��������תʱ��
{
	delay_ms(100);
	Send_UpMotor(speed,-speed);		// �����������
	delay_ms(tims);					// ��ʱ���Ƕ�
	Send_UpMotor(0 ,0);				// ͣ��
	delay_ms(100);
}

void Car_Time_Track(uint8_t speed, uint16_t tims)		// С��ѭ�� �������ٶȣ�ʱ��
{
		Stop_Flag = 0;          // ����״̬��־λ
    Go_Flag = 0;            // ǰ����־λ
    wheel_L_Flag = 0;       // ��ת��־λ
    wheel_R_Flag = 0;       // ��ת��־λ
    wheel_Nav_Flag = 0;     // ������ת��־λ
    Back_Flag = 0;          // ���˱�־λ
    Track_Flag = 1;         // ѭ����־λ
    Car_Spend = speed;      // �ٶ�ֵ
		Control(Car_Spend, Car_Spend);  // ����������� *
	if(tims <= 790)
	{
		delay_ms(tims);
	}
	else
	{
		delay_ms(790);
		delay_ms(tims-790);
	}
	Roadway_Flag_clean();	// �����־λ״̬	
	Send_UpMotor(0,0);		// ͣ��
	delay_ms(100);
}

void CarThread_Go(uint8_t speed, uint16_t temp)   // ����ǰ�� �������ٶ�/����
{
    Roadway_mp_syn();       // ����ͬ��
    Stop_Flag = 0;          // ����״̬��־λ
    Go_Flag = 1;            // ǰ����־λ
    wheel_L_Flag = 0;       // ��ת��־λ
    wheel_R_Flag = 0;       // ��ת��־λ
    wheel_Nav_Flag = 0;     // ������ת��־λ
    Back_Flag = 0;          // ���˱�־λ
    Track_Flag = 0;         // ѭ����־λ
    temp_MP = temp;         // ����ֵ
    Car_Spend = speed;      // �ٶ�ֵ
    Control(Car_Spend, Car_Spend);  // �����������
}

void CarThread_Track(uint8_t speed)   // ����ѭ�� �������ٶ�
{
    Stop_Flag = 0;          // ����״̬��־λ
    Go_Flag = 0;            // ǰ����־λ
    wheel_L_Flag = 0;       // ��ת��־λ
    wheel_R_Flag = 0;       // ��ת��־λ
    wheel_Nav_Flag = 0;     // ������ת��־λ
    Back_Flag = 0;          // ���˱�־λ
    Track_Flag = 1;         // ѭ����־λ
    Car_Spend = speed;      // �ٶ�ֵ
    Control(Car_Spend, Car_Spend);  // ����������� *
}

