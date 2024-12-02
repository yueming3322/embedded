/* Includes ------------------------------------------------------------------*/
#include "stdio.h"
#include "string.h"
#include "bkrc_voice.h"
#include <stdio.h>
#include "stm32f4xx.h"
#include "delay.h"
#include "delay.h"
#include "canp_hostcom.h"


/* �������� ---------------------------------------------------------*/
uint8_t uart6_data = 0;			// USART6 �������ݻ���
uint8_t uart6_flag = 0;			// USART6 ��������ʱ��
uint8_t UART6_RxData[8];		// USART6 �������ݻ���
uint8_t voice_falg = 0;		// ����ģ�鷵��״̬
uint8_t YY_Init[5] = {0xFD, 0x00, 0x00, 0x01, 0x01};
uint8_t Zigbee[8];           // Zigbee�������ݻ���

uint8_t start_voice_dis[5]= {0xFA,0xFA,0xFA,0xFA,0xA1};
//uint8_t start_voice_dis[8]= {0xAF,0x06,UART6_RxData[2],0x00,0x00,0x01,0xBB};
uint8_t bkrc_voice_Flag = 0;           // SYN7318����ʶ������ID���
/*******************************************************
�����ܣ���ʼ������
�Ρ�������
����ֵ����
********************************************************/
static void USART6_Hardware_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /* ʹ�� GPIOC ����ʱ�� */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

    /* ʹ�� USART6 ����ʱ�� */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);

    /* ���� PC6/PC7 ���Ÿ���ӳ�� */
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_USART6);

    /* �� PC6/PC7 ��������Ϊ���ù���ģʽ�������� */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;		// �˿�ģʽ -> ���ù���ģʽ
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;		// �˿�������� -> �������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	// �˿�����ٶ� -> ���� 100MHz(30pF)
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;  		// �˿�����/���� -> ����
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    /* USART6 ��ʼ������ */
    USART_InitStructure.USART_BaudRate = 115200;					// ����������
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	// Ӳ�������� -> ��Ӳ��������
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	// �շ�ģʽ���� -> ����+����
    USART_InitStructure.USART_Parity = USART_Parity_No;				// ��żУ��λ���� -> ����żУ��λ
    USART_InitStructure.USART_StopBits = USART_StopBits_1;			// ֹͣλ���� -> 1λֹͣλ
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;		// ����λ���� -> 8λ���ݸ�ʽ
    USART_Init(USART6, &USART_InitStructure);

    /* ʹ�� USART6 �ж� */
    USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);		// �������ݼĴ�����Ϊ���ж�

    /* ���� USART6 �ж����ȼ� */
    NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;				// ѡ�� IRQ ͨ��
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;	// ��ռ���ȼ�����
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;			// ��Ӧ���ȼ�����
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;					// ���� USART6 IRQ ͨ��
    NVIC_Init(&NVIC_InitStructure);

    /* ʹ�� USART6 */
    USART_Cmd(USART6, ENABLE);
}

/*******************************************************
�����ܣ������жϺ���
�Ρ�������
����ֵ����
********************************************************/
void USART6_IRQHandler(void)
{
    /* �ж� USART6 �Ƿ񴥷�ָ���ж� -> �������ݼĴ�����Ϊ���ж� */
    if (USART_GetITStatus(USART6, USART_IT_RXNE) != RESET)
    {
        uart6_data = USART_ReceiveData(USART6);		// ��ȡ USART6 ���ݼĴ���

        if (uart6_flag == 0x00)
        {
            if (uart6_data == 0x55)				// �Զ�������֡ͷ
            {
                uart6_flag = 0x01;
                UART6_RxData[0] = uart6_data;	// ֡ͷ
                UART6_RxData[1] = 0x00;
                UART6_RxData[2] = 0x00;
                UART6_RxData[3] = 0x00;
            }
        }
        else if (uart6_flag == 0x01)
        {
            uart6_flag = 0x02;
            UART6_RxData[1] = uart6_data;		// ��������
        }
        else if(uart6_flag == 0x02)
        {
            uart6_flag = 0x03;
            UART6_RxData[2] = uart6_data;		// ״̬��־
        }
        else if(uart6_flag == 0x03)
        {
            uart6_flag = 0x00;
            UART6_RxData[3] = uart6_data;		// ����λ
            voice_falg = 0x01;					// �Զ�������֡�������
        }
        else
        {
            uart6_flag = 0x00;
            voice_falg = 0x00;
            UART6_RxData[0] = 0x00;
        }

    }
    //��������жϽ��ձ�־λ
    USART_ClearITPendingBit(USART6,USART_IT_RXNE);
}

/*******************************************************
�����ܣ�ͨ������1����һ���ֽڣ���0x12��0xff��
�Ρ�����hex -> �ֽ�
����ֵ����
********************************************************/
void USART6_Send_Byte(uint8_t byte)
{
    USART_SendData(USART6,byte);
    while(USART_GetFlagStatus(USART6,USART_FLAG_TXE) == RESET);
}

/*******************************************************
�����ܣ�ͨ������1����һ������
�Ρ�����*buf -> ָ��ָ��һ������
		 length -> ����ĳ���
����ֵ����
********************************************************/
void USART6_Send_Length(uint8_t *buf,uint8_t length)
{
    uint8_t len = 0;
    for(len = 0; len < length; len++)
    {
        USART6_Send_Byte(buf[len]);
    }
}

/**************************************************
��  �ܣ�����ʶ����
��  ����	0�������������漴����ָ������ʶ����ԣ�2-6����ָ�������������ʶ�����
����ֵ��	��������ID    ��������
           0x02 -> ��ǿ·վ
			  0x03 -> ����·վ
			  0x04 -> ����·վ
			  0x05 -> ��г·վ
			  0x06 -> ����·վ
			  0x07 -> ��ҵ·վ
			  0x08 -> ����·վ
		
**************************************************/
uint8_t BKRC_Voice_Extern(uint8_t yy_mode)		// ����ʶ��
{
    uint16_t timers = 0;               // ����ֵ2
    USART6_Send_Length(start_voice_dis,5);//���Ϳ�������ʶ��ָ��
    delay_ms(500);
    bkrc_voice_Flag = Voice_Drive();//���շ���״̬

    delay_ms(500);
    delay_ms(500);
    delay_ms(500);
	if(yy_mode==0)
	{
	    YY_Comm_Zigbee(0x20,0x01);			//�������������������
	}else
	{
		YY_Comm_Zigbee(0x10,yy_mode);			//�������������������
	}

    bkrc_voice_Flag=0;
    while (1)
    {

        delay_ms(1);
        timers++;
        bkrc_voice_Flag = Voice_Drive();

        if (bkrc_voice_Flag != 0x00||timers>6000)   //�жϳ�ʱ�˳�
        {
            timers=0;
            return bkrc_voice_Flag;
        }
    }
}


/**************************************************
��  �ܣ�����ʶ��ش������������
��  ����	��
����ֵ��	��������ID /С������ʶ��ģ��״̬
**************************************************/
uint8_t Voice_Drive(void)
{
    uint8_t status = 0;
		if ((voice_falg == 0x01) && (UART6_RxData[0] == 0xAF))			// �Զ�������֡�������
    //if ((voice_falg == 0x01) && (UART6_RxData[0] == 0x55))			// �Զ�������֡�������
    {
        if (UART6_RxData[1] == 0x01)
        {
            status = 0x00;
            switch (UART6_RxData[2])
            {
            case 0x01: {
                //                printf("* ��ʼ����� *");
                status |= 0x80;
                break;
            }
            case 0x02: {
                //                printf("* ����ʶ��ģʽ *");
                status |= 0x40;
                break;
            }
            case 0x03: {
                //                printf("* �˳�ʶ��ģʽ *");
                status &= 0xB0;
                break;
            }
            case 0x04: {
                //                printf("* ��������ģʽ *");
                status = 0x00;
                break;
            }
            default  :
                break;
            }
        }

        else if (UART6_RxData[1] == 0x06)
        {
            status &= 0xF0;
            switch (UART6_RxData[2])
            {
            case 0x01: {
                USART6_Send_Byte(0x01);
				Send_InfoData_To_Fifo("ID: 2\n", 9);
				printf("* �κ�����վ *");
                status |= 0x02;
                break;
            }
            case 0x02: {
                USART6_Send_Byte(0x02);
				Send_InfoData_To_Fifo("ID: 3\n", 9);
                printf("* ������վ *");
                status |= 0x03;
                break;
            }
            case 0x03: {
                USART6_Send_Byte(0x03);
				Send_InfoData_To_Fifo("ID: 4\n", 9);
                printf("* ��ɽվ *");
                status |= 0x04;
                break;
            }
            case 0x04: {
                USART6_Send_Byte(0x04);
				Send_InfoData_To_Fifo("ID: 5\n", 9);
								printf("* ͬ��վ *");
                status |= 0x05;
                break;
            }
            case 0x05: {
                USART6_Send_Byte(0x05);
				Send_InfoData_To_Fifo("ID: 6\n", 9);
                printf("* ����վ *");
                status |= 0x06;
                break;
            }
			case 0x06: {
                USART6_Send_Byte(0x06);
				Send_InfoData_To_Fifo("ID: 7\n", 9);
                printf("* ����վ *");
                status |= 0x07;
                break;
            }
			case 0x07: {
                USART6_Send_Byte(0x07);
				Send_InfoData_To_Fifo("ID: 8\n", 9);
                printf("* ������վ *");
                status |= 0x08;
                break;
            }
            default  :
                break;
            }
        }
        else if (UART6_RxData[1] == 0x03)
        {
            switch (UART6_RxData[2])
            {
            case 0x01:
                //                printf("* TTS: �������� *");
                break;
            case 0x02:
                //                printf("* TTS: ����ɽ�� *");
                break;
            case 0x03:
                //                printf("* TTS: ׷������ *");
                break;
            case 0x04:
                //                printf("* TTS: �﷫���� *");
                break;
            case 0x05:
                //                printf("* TTS: ��ͷ���� *");
                break;
            case 0x10:
                //                printf("* 00.mp3 *");
                break;
            case 0x11:
                //                printf("* 01.mp3 *");
                break;
            case 0x12:
                //                printf("* 02.mp3 *");
                break;
            default  :
                break;
            }
        }
				
//				  else if (UART6_RxData[1] == 0x06)
//        {
//            switch (UART6_RxData[2])
//            {
//            case 0x01:
//                                printf("* TTS: ���ܳɲ� *");
//                break;
//            case 0x02:
//                                printf("* TTS: �������� *");
//                break;
//            case 0x03:
//                                printf("* TTS: �������� *");
//                break;
//            case 0x04:
//                                printf("* TTS: �������� *");
//                break;
//            case 0x05:
//                                printf("* TTS: չ��ҵ�ټ� *");
//                break;
//            case 0x06:
//                                printf("* TTS: ���˲��¹� *");
//                break;
//            case 0x10:
//                                printf("* 00.mp3 *");
//                break;
//            case 0x11:
//                                printf("* 01.mp3 *");
//                break;
//            case 0x12:
//                                printf("* 02.mp3 *");
//                break;
//            default  :
//                break;
//            }
//        }
        voice_falg = 0x00;
    }
    return status;
}

/**************************************************
��  �ܣ���������������־�ﲥ��ָ���ı���Ϣ
��  ����	*p  --> ��Ҫ���͵�����
����ֵ��	��
**************************************************/
void YY_Play_Zigbee(char *p)
{
    uint16_t p_len = strlen(p);             // �ı�����

    YY_Init[1] = 0xff & ((p_len + 2) >> 8); // ���������ȸ߰�λ
    YY_Init[2] = 0xff & (p_len + 2);        // ���������ȵͰ�λ
    Send_ZigbeeData_To_Fifo(YY_Init, 5);
    Send_ZigbeeData_To_Fifo((uint8_t *)p, p_len);
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
|  0x10   |  0x01     | ��ǿ·վ
|         |  0x02     | ����·վ
|         |  0x03     | ����·վ
|         |  0x04     | ��г·վ
|         |  0x05     | ����·վ
|         |  0x06     | ��ҵ·վ
|         |  0x07     | ����·վ
|---------|-----------|------------------------------------------------
|  0x20   |  0x01     | ���ָ��
|---------|-----------|------------------------------------------------
***********************************************************************/
void YY_Comm_Zigbee(uint8_t Primary, uint8_t Secondary)
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
/*******************************************************
�����ܣ�����ʶ���ʼ������
�Ρ�������
����ֵ����
********************************************************/
void BKRC_Voice_Init(void)
{
    USART6_Hardware_Init();
}

//										endfile
