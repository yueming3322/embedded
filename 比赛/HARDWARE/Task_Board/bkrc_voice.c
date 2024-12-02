/* Includes ------------------------------------------------------------------*/
#include "stdio.h"
#include "string.h"
#include "bkrc_voice.h"
#include <stdio.h>
#include "stm32f4xx.h"
#include "delay.h"
#include "delay.h"
#include "canp_hostcom.h"


/* 变量定义 ---------------------------------------------------------*/
uint8_t uart6_data = 0;			// USART6 接收数据缓存
uint8_t uart6_flag = 0;			// USART6 接收数据时序
uint8_t UART6_RxData[8];		// USART6 接收数据缓存
uint8_t voice_falg = 0;		// 语音模块返回状态
uint8_t YY_Init[5] = {0xFD, 0x00, 0x00, 0x01, 0x01};
uint8_t Zigbee[8];           // Zigbee发送数据缓存

uint8_t start_voice_dis[5]= {0xFA,0xFA,0xFA,0xFA,0xA1};
//uint8_t start_voice_dis[8]= {0xAF,0x06,UART6_RxData[2],0x00,0x00,0x01,0xBB};
uint8_t bkrc_voice_Flag = 0;           // SYN7318语音识别命令ID编号
/*******************************************************
功　能：初始化串口
参　数：无
返回值：无
********************************************************/
static void USART6_Hardware_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /* 使能 GPIOC 外设时钟 */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

    /* 使能 USART6 外设时钟 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);

    /* 配置 PC6/PC7 引脚复用映射 */
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_USART6);

    /* 将 PC6/PC7 引脚配置为复用功能模式（上拉） */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;		// 端口模式 -> 复用功能模式
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;		// 端口输出类型 -> 推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	// 端口输出速度 -> 高速 100MHz(30pF)
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;  		// 端口上拉/下拉 -> 上拉
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    /* USART6 初始化配置 */
    USART_InitStructure.USART_BaudRate = 115200;					// 波特率设置
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	// 硬件流设置 -> 无硬件流控制
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	// 收发模式设置 -> 接收+发送
    USART_InitStructure.USART_Parity = USART_Parity_No;				// 奇偶校验位设置 -> 无奇偶校验位
    USART_InitStructure.USART_StopBits = USART_StopBits_1;			// 停止位设置 -> 1位停止位
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;		// 数据位长度 -> 8位数据格式
    USART_Init(USART6, &USART_InitStructure);

    /* 使能 USART6 中断 */
    USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);		// 接收数据寄存器不为空中断

    /* 设置 USART6 中断优先级 */
    NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;				// 选择 IRQ 通道
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;	// 抢占优先级设置
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;			// 响应优先级设置
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;					// 启用 USART6 IRQ 通道
    NVIC_Init(&NVIC_InitStructure);

    /* 使能 USART6 */
    USART_Cmd(USART6, ENABLE);
}

/*******************************************************
功　能：串口中断函数
参　数：无
返回值：无
********************************************************/
void USART6_IRQHandler(void)
{
    /* 判断 USART6 是否触发指定中断 -> 接收数据寄存器不为空中断 */
    if (USART_GetITStatus(USART6, USART_IT_RXNE) != RESET)
    {
        uart6_data = USART_ReceiveData(USART6);		// 读取 USART6 数据寄存器

        if (uart6_flag == 0x00)
        {
            if (uart6_data == 0x55)				// 自定义数据帧头
            {
                uart6_flag = 0x01;
                UART6_RxData[0] = uart6_data;	// 帧头
                UART6_RxData[1] = 0x00;
                UART6_RxData[2] = 0x00;
                UART6_RxData[3] = 0x00;
            }
        }
        else if (uart6_flag == 0x01)
        {
            uart6_flag = 0x02;
            UART6_RxData[1] = uart6_data;		// 数据类型
        }
        else if(uart6_flag == 0x02)
        {
            uart6_flag = 0x03;
            UART6_RxData[2] = uart6_data;		// 状态标志
        }
        else if(uart6_flag == 0x03)
        {
            uart6_flag = 0x00;
            UART6_RxData[3] = uart6_data;		// 数据位
            voice_falg = 0x01;					// 自定义数据帧接收完毕
        }
        else
        {
            uart6_flag = 0x00;
            voice_falg = 0x00;
            UART6_RxData[0] = 0x00;
        }

    }
    //清除串口中断接收标志位
    USART_ClearITPendingBit(USART6,USART_IT_RXNE);
}

/*******************************************************
功　能：通过串口1发送一个字节，如0x12、0xff等
参　数：hex -> 字节
返回值：无
********************************************************/
void USART6_Send_Byte(uint8_t byte)
{
    USART_SendData(USART6,byte);
    while(USART_GetFlagStatus(USART6,USART_FLAG_TXE) == RESET);
}

/*******************************************************
功　能：通过串口1发送一个数组
参　数：*buf -> 指针指向一个数组
		 length -> 数组的长度
返回值：无
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
功  能：语音识别函数
参  数：	0控制语音播报随即播报指令语音识别测试，2-6播报指定命令进行语音识别测试
返回值：	语音词条ID    词条内容
           0x02 -> 富强路站
			  0x03 -> 民主路站
			  0x04 -> 文明路站
			  0x05 -> 和谐路站
			  0x06 -> 爱国路站
			  0x07 -> 敬业路站
			  0x08 -> 友善路站
		
**************************************************/
uint8_t BKRC_Voice_Extern(uint8_t yy_mode)		// 语音识别
{
    uint16_t timers = 0;               // 计数值2
    USART6_Send_Length(start_voice_dis,5);//发送开启语音识别指令
    delay_ms(500);
    bkrc_voice_Flag = Voice_Drive();//接收返回状态

    delay_ms(500);
    delay_ms(500);
    delay_ms(500);
	if(yy_mode==0)
	{
	    YY_Comm_Zigbee(0x20,0x01);			//语音播报随机语音命令
	}else
	{
		YY_Comm_Zigbee(0x10,yy_mode);			//语音播报随机语音命令
	}

    bkrc_voice_Flag=0;
    while (1)
    {

        delay_ms(1);
        timers++;
        bkrc_voice_Flag = Voice_Drive();

        if (bkrc_voice_Flag != 0x00||timers>6000)   //判断超时退出
        {
            timers=0;
            return bkrc_voice_Flag;
        }
    }
}


/**************************************************
功  能：语音识别回传命令解析函数
参  数：	无
返回值：	语音词条ID /小创语音识别模块状态
**************************************************/
uint8_t Voice_Drive(void)
{
    uint8_t status = 0;
		if ((voice_falg == 0x01) && (UART6_RxData[0] == 0xAF))			// 自定义数据帧接收完毕
    //if ((voice_falg == 0x01) && (UART6_RxData[0] == 0x55))			// 自定义数据帧接收完毕
    {
        if (UART6_RxData[1] == 0x01)
        {
            status = 0x00;
            switch (UART6_RxData[2])
            {
            case 0x01: {
                //                printf("* 初始化完成 *");
                status |= 0x80;
                break;
            }
            case 0x02: {
                //                printf("* 进入识别模式 *");
                status |= 0x40;
                break;
            }
            case 0x03: {
                //                printf("* 退出识别模式 *");
                status &= 0xB0;
                break;
            }
            case 0x04: {
                //                printf("* 进入休眠模式 *");
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
				printf("* 嘉禾望岗站 *");
                status |= 0x02;
                break;
            }
            case 0x02: {
                USART6_Send_Byte(0x02);
				Send_InfoData_To_Fifo("ID: 3\n", 9);
                printf("* 花地湾站 *");
                status |= 0x03;
                break;
            }
            case 0x03: {
                USART6_Send_Byte(0x03);
				Send_InfoData_To_Fifo("ID: 4\n", 9);
                printf("* 五山站 *");
                status |= 0x04;
                break;
            }
            case 0x04: {
                USART6_Send_Byte(0x04);
				Send_InfoData_To_Fifo("ID: 5\n", 9);
								printf("* 同和站 *");
                status |= 0x05;
                break;
            }
            case 0x05: {
                USART6_Send_Byte(0x05);
				Send_InfoData_To_Fifo("ID: 6\n", 9);
                printf("* 琵琶站 *");
                status |= 0x06;
                break;
            }
			case 0x06: {
                USART6_Send_Byte(0x06);
				Send_InfoData_To_Fifo("ID: 7\n", 9);
                printf("* 大洲站 *");
                status |= 0x07;
                break;
            }
			case 0x07: {
                USART6_Send_Byte(0x07);
				Send_InfoData_To_Fifo("ID: 8\n", 9);
                printf("* 广州南站 *");
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
                //                printf("* TTS: 美好生活 *");
                break;
            case 0x02:
                //                printf("* TTS: 秀丽山河 *");
                break;
            case 0x03:
                //                printf("* TTS: 追逐梦想 *");
                break;
            case 0x04:
                //                printf("* TTS: 扬帆启航 *");
                break;
            case 0x05:
                //                printf("* TTS: 齐头并进 *");
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
//                                printf("* TTS: 技能成才 *");
//                break;
//            case 0x02:
//                                printf("* TTS: 匠心筑梦 *");
//                break;
//            case 0x03:
//                                printf("* TTS: 筑梦扬威 *");
//                break;
//            case 0x04:
//                                printf("* TTS: 技行天下 *");
//                break;
//            case 0x05:
//                                printf("* TTS: 展行业百技 *");
//                break;
//            case 0x06:
//                                printf("* TTS: 树人才新观 *");
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
功  能：控制语音播报标志物播报指定文本信息
参  数：	*p  --> 需要发送的数据
返回值：	无
**************************************************/
void YY_Play_Zigbee(char *p)
{
    uint16_t p_len = strlen(p);             // 文本长度

    YY_Init[1] = 0xff & ((p_len + 2) >> 8); // 数据区长度高八位
    YY_Init[2] = 0xff & (p_len + 2);        // 数据区长度低八位
    Send_ZigbeeData_To_Fifo(YY_Init, 5);
    Send_ZigbeeData_To_Fifo((uint8_t *)p, p_len);
    delay_ms(100);
}

/**********************************************************************
 * 函 数 名 ：  控制语音播报标志物播报语音控制命令
 * 参    数 ：  Primary   -> 主指令
                Secondary -> 副职令
                详见附录1
 * 返 回 值 ：  无
 * 简    例 ：  YY_Comm_Zigbee(0x20, 0x01);     // 语音播报随机语音命令

附录1：
-----------------------------------------------------------------------
| Primary | Secondary | 说明
|---------|-----------|------------------------------------------------
|  0x10   |  0x01     | 富强路站
|         |  0x02     | 民主路站
|         |  0x03     | 文明路站
|         |  0x04     | 和谐路站
|         |  0x05     | 爱国路站
|         |  0x06     | 敬业路站
|         |  0x07     | 友善路站
|---------|-----------|------------------------------------------------
|  0x20   |  0x01     | 随机指令
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
功　能：语音识别初始化函数
参　数：无
返回值：无
********************************************************/
void BKRC_Voice_Init(void)
{
    USART6_Hardware_Init();
}

//										endfile
