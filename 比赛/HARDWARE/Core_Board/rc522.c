#include "sys.h"
#include "rc522.h"
#include "delay.h"
#include "string.h" 
#include "cba.h"
#include "Timer.h"
#include "canp_hostcom.h"

#define MAXRLEN 18 

static uint8_t Rc522_LinkFlag;

uint8_t CT[2];		// ������
uint8_t SN[4];		// ����
uint8_t READ_RFID[16];		// ���RFID 
uint8_t WRITE_RFID[16]={"0123456789ABCDEF"};
uint8_t KEY_A[6]={0xff,0xff,0xff,0xff,0xff,0xff};   // A��Կ
uint8_t KEY_B[6]={0xff,0xff,0xff,0xff,0xff,0xff};	// B��Կ
uint8_t ADDR_Str[14]={"RFID_ADDR:01\n"};


/********************************************************
 * ��������	RC522	RFID��д���ƺ���
 * ��  ����	card_addr	���ַ����Χ��0~63��
			mode	RFID_Read -> RFID������
					RFID_Write -> RFIDд����
					RFID_Write_Read -> RFID��д�����������
 * ����ֵ��	��
********************************************************/
void RC522(uint8_t card_addr,uint8_t mode)
{
	uint8_t card_key = (card_addr/4)*4+3;
	
	LED1 = 0;
	LED2 = 0;
	LED3 = 0;
	LED4 = 0;
   if(1)
//	if(PcdRequest(PICC_REQALL,CT) == MI_OK)		// Ѱ���ɹ�
	{
		LED1=1;
		if(PcdAnticoll(SN) == MI_OK)			// ����ײ�ɹ�
		{
			LED2=1;
			if(PcdSelect(SN) == MI_OK)			// ѡ���ɹ�
			{
				LED3=1;
				if(PcdAuthState(KEYA,card_key,KEY_A,SN) == MI_OK)	// ��֤��Կ��A��Կ��
				{
					LED4=1;
					ADDR_Str[10] = card_addr/10%10 + 0x30;
					ADDR_Str[11] = card_addr%10 + 0x30;
					Send_InfoData_To_Fifo((char *)ADDR_Str,14);		// ��ӡ��д�������ַ
					if(card_addr == card_key)		// ��д���ַΪ��Կ��
					{
						mode = 0;			// ��Կ���ֹ��д����
						Send_InfoData_To_Fifo("ADDR ERROR!\n",13);
					}
					
					if((mode == RFID_Write) || (mode == RFID_Write_Read))	// ����д������
					{
						if((PcdWrite(card_addr,WRITE_RFID) == MI_OK))		// д������
						{
							Send_InfoData_To_Fifo("WRITE_RFID - OK\n",17);
						}
					}
					if((mode == RFID_Read) || (mode == RFID_Write_Read))	// �����������
					{
						if(PcdRead(card_addr,READ_RFID) == MI_OK)			// ��ȡ����
						{
							Send_InfoData_To_Fifo("READ_RFID - OK\n",16);
							Send_InfoData_To_Fifo("DATA(ASCII):\n",14);
							Send_InfoData_To_Fifo((char *)READ_RFID,16);
							Send_InfoData_To_Fifo("\n",2);
						}
					}
					Send_InfoData_To_Fifo("\n",2);
				}
			}
		}
	}
}



//��ʼ��IO ����1 
//bound:������
void RC522_Uart_init(u32 baudrate)
{
	GPIO_InitTypeDef  GPIO_TypeDefStructure;
	USART_InitTypeDef USART_TypeDefStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1);
	
	//PA9-Tx
	GPIO_TypeDefStructure.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_10;
	GPIO_TypeDefStructure.GPIO_Mode = GPIO_Mode_AF;		//���ù���
	GPIO_TypeDefStructure.GPIO_OType = GPIO_OType_PP; //�������
	GPIO_TypeDefStructure.GPIO_PuPd = GPIO_PuPd_UP;   //����
	GPIO_TypeDefStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOA,&GPIO_TypeDefStructure);
	
	USART_TypeDefStructure.USART_BaudRate = baudrate;					       //������
	USART_TypeDefStructure.USART_HardwareFlowControl = 				       //��Ӳ��������
												 USART_HardwareFlowControl_None;  
	USART_TypeDefStructure.USART_Mode = USART_Mode_Tx|USART_Mode_Rx; //�����뷢��ģʽ
	USART_TypeDefStructure.USART_Parity = USART_Parity_No; 		       //��У��λ
	USART_TypeDefStructure.USART_StopBits = USART_StopBits_1;        //ֹͣλ1
	USART_TypeDefStructure.USART_WordLength = USART_WordLength_8b;   //����λ8λ
	USART_Init(USART1,&USART_TypeDefStructure);

	USART_Cmd(USART1, ENABLE);                    //ʹ�ܴ��� 

	Rc522_LinkFlag = 0;
}

void WriteRawRC(unsigned char addr, unsigned char datas)
{
	if(WriteRawRC_HDL(addr,datas) != STATUS_SUCCESS)
	{
		;
	}
}

char InitRc522(void)
{
	if(PcdReset() != MI_OK)
		return MI_ERR;
	PcdAntennaOff();
	delay_ms(2);  
	PcdAntennaOn();
	if(M500PcdConfigISOType( 'A' ) != MI_OK)
		return MI_ERR;
	Rc522_LinkFlag = 1;
	return MI_OK;
}

void Readcard_daivce_Init(void)
{
	RC522_Uart_init(9600);	// ���ڳ�ʼ��Ϊ9600
	delay_ms(500);
	InitRc522();			//��������ʼ��
}

void Reset_RC522(void)
{
	PcdReset();
	PcdAntennaOff();
	delay_ms(2);
	PcdAntennaOn();
}

/////////////////////////////////////////////////////////////////////
//��    �ܣ�Ѱ��
//����˵��: req_code[IN]:Ѱ����ʽ
//                0x52 = Ѱ��Ӧ�������з���14443A��׼�Ŀ�
//                0x26 = Ѱδ��������״̬�Ŀ�
//          	  pTagType[OUT]����Ƭ���ʹ���
//                0x4400 = Mifare_UltraLight
//                0x0400 = Mifare_One(S50)
//                0x0200 = Mifare_One(S70)
//                0x0800 = Mifare_Pro(X)
//                0x4403 = Mifare_DESFire
//��    ��: �ɹ�����MI_OK
/////////////////////////////////////////////////////////////////////
char PcdRequest(unsigned char req_code,unsigned char *pTagType)
{
	char status;  
	unsigned int  unLen;
	unsigned char ucComMF522Buf[MAXRLEN];
	
	ClearBitMask(Status2Reg,0x08);
	WriteRawRC(BitFramingReg,0x07);
	SetBitMask(TxControlReg,0x03);
	ucComMF522Buf[0] = req_code;

	status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,1,ucComMF522Buf,&unLen);
	if((status == MI_OK) && (unLen == 0x10))
	{
		*pTagType = ucComMF522Buf[0];
		*(pTagType+1) = ucComMF522Buf[1];
	}
	else
	{
		status = MI_ERR;
	}
	return status;
}

/////////////////////////////////////////////////////////////////////
//��    �ܣ�����ײ
//����˵��: pSnr[OUT]:��Ƭ���кţ�4�ֽ�
//��    ��: �ɹ�����MI_OK
/////////////////////////////////////////////////////////////////////  
char PcdAnticoll(unsigned char *pSnr)
{
    char status;
    unsigned char i,snr_check=0;
    unsigned int  unLen;
    unsigned char ucComMF522Buf[MAXRLEN]; 
    
    ClearBitMask(Status2Reg,0x08);
    WriteRawRC(BitFramingReg,0x00);
    ClearBitMask(CollReg,0x80);
    ucComMF522Buf[0] = PICC_ANTICOLL1;
    ucComMF522Buf[1] = 0x20;
    status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,2,ucComMF522Buf,&unLen);

    if(status == MI_OK)
    {
    	 for (i=0; i<4; i++)
         {
             *(pSnr+i) = ucComMF522Buf[i];
             snr_check ^= ucComMF522Buf[i];
         }
         if (snr_check != ucComMF522Buf[i])
         {
			 status = MI_ERR;
		 }
    }
    SetBitMask(CollReg,0x80);
    return status;
}

/////////////////////////////////////////////////////////////////////
//��    �ܣ�ѡ����Ƭ
//����˵��: pSnr[IN]:��Ƭ���кţ�4�ֽ�
//��    ��: �ɹ�����MI_OK
/////////////////////////////////////////////////////////////////////
char PcdSelect(unsigned char *pSnr)
{
    char status;
    unsigned char i;
    unsigned int  unLen;
    unsigned char ucComMF522Buf[MAXRLEN]; 
    
    ucComMF522Buf[0] = PICC_ANTICOLL1;
    ucComMF522Buf[1] = 0x70;
    ucComMF522Buf[6] = 0;
    for (i=0; i<4; i++)
    {
    	ucComMF522Buf[i+2] = *(pSnr+i);
    	ucComMF522Buf[6] ^= *(pSnr+i);
    }
    CalulateCRC(ucComMF522Buf,7,&ucComMF522Buf[7]);
    ClearBitMask(Status2Reg,0x08);
    status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,9,ucComMF522Buf,&unLen);
    
    if ((status == MI_OK) && (unLen == 0x18))
    {
		status = MI_OK;
	}
    else
    {
		status = MI_ERR;
	}
    return status;
}

/////////////////////////////////////////////////////////////////////
//��    �ܣ���֤��Ƭ����
//����˵��: auth_mode[IN]: ������֤ģʽ
//                 0x60 = ��֤A��Կ
//                 0x61 = ��֤B��Կ 
//          addr[IN]�����ַ
//          pKey[IN]������
//          pSnr[IN]����Ƭ���кţ�4�ֽ�
//��    ��: �ɹ�����MI_OK
/////////////////////////////////////////////////////////////////////               
char PcdAuthState(unsigned char auth_mode,unsigned char addr,unsigned char *pKey,unsigned char *pSnr)
{
    char status;
    unsigned int  unLen;
    unsigned char i,ucComMF522Buf[MAXRLEN]; 

    ucComMF522Buf[0] = auth_mode;
    ucComMF522Buf[1] = addr;
    for (i=0; i<6; i++)
    {
		ucComMF522Buf[i+2] = *(pKey+i);
	}
    for (i=0; i<6; i++)
    {
		ucComMF522Buf[i+8] = *(pSnr+i);
	}
    status = PcdComMF522(PCD_AUTHENT,ucComMF522Buf,12,ucComMF522Buf,&unLen);
    if((status != MI_OK) || (!(ReadRawRC(Status2Reg) & 0x08)))
    {
		status = MI_ERR;
	}
    return status;
}

/////////////////////////////////////////////////////////////////////
//��    �ܣ���ȡM1��һ������
//����˵��: addr[IN]�����ַ
//          pData[OUT]�����������ݣ�16�ֽ�
//��    ��: �ɹ�����MI_OK
///////////////////////////////////////////////////////////////////// 
char PcdRead(unsigned char addr,unsigned char *pData)
{
    char status;
    unsigned int  unLen;
    unsigned char i,ucComMF522Buf[MAXRLEN]; 

    ucComMF522Buf[0] = PICC_READ;
    ucComMF522Buf[1] = addr;
    CalulateCRC(ucComMF522Buf,2,&ucComMF522Buf[2]);
    status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,4,ucComMF522Buf,&unLen);
    if((status == MI_OK) && (unLen == 0x90))
    {
        for (i=0; i<16; i++)
        {
			*(pData+i) = ucComMF522Buf[i];
		}
    }
    else
    {
		status = MI_ERR;
	}
    return status;
}

/////////////////////////////////////////////////////////////////////
//��    �ܣ�д���ݵ�M1��һ��
//����˵��: addr[IN]�����ַ
//          pData[IN]��д������ݣ�16�ֽ�
//��    ��: �ɹ�����MI_OK
/////////////////////////////////////////////////////////////////////                  
char PcdWrite(unsigned char addr,unsigned char *pData)
{
    char status;
    unsigned int  unLen;
    unsigned char i,ucComMF522Buf[MAXRLEN]; 
    
    ucComMF522Buf[0] = PICC_WRITE;
    ucComMF522Buf[1] = addr;
    CalulateCRC(ucComMF522Buf,2,&ucComMF522Buf[2]);
    status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,4,ucComMF522Buf,&unLen);

    if((status != MI_OK) || (unLen != 4) || ((ucComMF522Buf[0] & 0x0F) != 0x0A))
    {
		status = MI_ERR;
	}
        
    if(status == MI_OK)
    {
        for (i=0; i<16; i++)
        {
			ucComMF522Buf[i] = *(pData+i);
		}
        CalulateCRC(ucComMF522Buf,16,&ucComMF522Buf[16]);
        status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,18,ucComMF522Buf,&unLen);
        if ((status != MI_OK) || (unLen != 4) || ((ucComMF522Buf[0] & 0x0F) != 0x0A))
        {
			status = MI_ERR;
		}
    }
    return status;
}

/////////////////////////////////////////////////////////////////////
//��    �ܣ����Ƭ��������״̬
//��    ��: �ɹ�����MI_OK
/////////////////////////////////////////////////////////////////////
char PcdHalt(void)
{
    char status;
    unsigned int  unLen;
    unsigned char ucComMF522Buf[MAXRLEN]; 

    ucComMF522Buf[0] = PICC_HALT;
    ucComMF522Buf[1] = 0;
    CalulateCRC(ucComMF522Buf,2,&ucComMF522Buf[2]);
    status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,4,ucComMF522Buf,&unLen);
    return status;
}

/////////////////////////////////////////////////////////////////////
//��MF522����CRC16����
/////////////////////////////////////////////////////////////////////
char CalulateCRC(unsigned char *pIndata,unsigned char len,unsigned char *pOutData)
{
    unsigned char i,n;
    ClearBitMask(DivIrqReg,0x04);
    WriteRawRC(CommandReg,PCD_IDLE);
    SetBitMask(FIFOLevelReg,0x80);
    for (i=0; i<len; i++)
    {
		WriteRawRC(FIFODataReg, *(pIndata+i));
	}
    WriteRawRC(CommandReg, PCD_CALCCRC);
    i = 0xFF;
    do 
    {
        n = ReadRawRC(DivIrqReg);
        i--;
    }
    while((i!=0) && !(n&0x04));
    pOutData[0] = ReadRawRC(CRCResultRegL);
    pOutData[1] = ReadRawRC(CRCResultRegM);
	return MI_OK;
}

/////////////////////////////////////////////////////////////////////
//��    �ܣ���λRC522
//��    ��: �ɹ�����MI_OK
/////////////////////////////////////////////////////////////////////
char PcdReset(void)
{
	if(Rc522_OutSRst()!=MI_OK)
		return MI_ERR;
    WriteRawRC(CommandReg,PCD_RESETPHASE);
    WriteRawRC(ModeReg,0x3D);            //��Mifare��ͨѶ��CRC��ʼֵ0x6363
    WriteRawRC(TReloadRegL,30);           
    WriteRawRC(TReloadRegH,0);
    WriteRawRC(TModeReg,0x8D);
    WriteRawRC(TPrescalerReg,0x3E);
    WriteRawRC(TxAutoReg,0x40); 
	  
    return MI_OK;
}
//////////////////////////////////////////////////////////////////////
//����RC522�Ĺ�����ʽ 
//////////////////////////////////////////////////////////////////////
char M500PcdConfigISOType(unsigned char type)
{
	if(type == 'A')                     //ISO14443_A
	{
		ClearBitMask(Status2Reg,0x08);
		WriteRawRC(ModeReg,0x3D);//3F
		WriteRawRC(RxSelReg,0x86);//84
		WriteRawRC(RFCfgReg,0x7F);   //4F
		WriteRawRC(TReloadRegL,30);//tmoLength);// TReloadVal = 'h6a =tmoLength(dec) 
		WriteRawRC(TReloadRegH,0);
		WriteRawRC(TModeReg,0x8D);
		WriteRawRC(TPrescalerReg,0x3E);
		delay_ms(10);
		PcdAntennaOn();
	}
	else 
	{
		return ((char)-1);
	}
	return MI_OK;
}
/*************************************************
Function:       Send_data
Description:
     write a byte to serial port
Parameter:
     ch            the byte to write
Return:
     None
**************************************************/
void Send_data(unsigned char ch)
{	
	USART1->SR;
	while((USART1->SR & USART_FLAG_TXE) == SET)
	{ ; }
    USART_SendData(USART1,ch); 
	while(USART_GetFlagStatus(USART1,USART_FLAG_TC) != SET)
	{ ; }
}

/*************************************************
Function:       Rece_data
Description:
     get a byte from serial port
Parameter:
     ch            the byte to save the received byte
     WaitTime      the counter for polling, avoid endless loop.
Return:
     STATUS_SUCCESS         get a byte data successfully
     STATUS_IO_TIMEOUT      time out
**************************************************/
short Rece_data(unsigned char *ch, unsigned int WaitTime)
{
	uint32_t tt;
	tt = gt_get() + WaitTime/2000;
	while(gt_get_sub(tt))
	{
		if(USART1->SR & USART_FLAG_RXNE)
		{
			*ch = (uint8_t)USART_ReceiveData(USART1);
			return STATUS_SUCCESS;			
		}
	}
	Rc522_LinkFlag = 0;
	return STATUS_IO_TIMEOUT;
	
}
/////////////////////////////////////////////////////////////////////
//��    �ܣ���RC632�Ĵ���
//����˵����Address[IN]:�Ĵ�����ַ
//��    �أ�������ֵ
/////////////////////////////////////////////////////////////////////
unsigned char ReadRawRC(unsigned char Address)
{
    unsigned char RegVal;
    short status;
    Address = (Address & 0x3f) | 0x80;   //code the first byte
    Send_data(Address);
    status = Rece_data(&RegVal, 10000);
    if(status != STATUS_SUCCESS)
        return 0xff;
    return RegVal;
}

/////////////////////////////////////////////////////////////////////
//��    �ܣ�дRC632�Ĵ���
//����˵����Address[IN]:�Ĵ�����ַ
//          value[IN]:д���ֵ
/////////////////////////////////////////////////////////////////////
//short WriteRawRC(unsigned char Address, unsigned char value)
//{  
//    unsigned char EchoByte;
//    short status;
//    Address &= 0x3f;   //code the first byte
//    Send_data(Address);
//    Send_data(value);
//		
//    status = Rece_data(&EchoByte, 10000);
//	return status;
//}

//short WriteRawRC_HDL2(unsigned char Address, unsigned char value)
//{  
//    unsigned char EchoByte;
//    short status;
//    Address &= 0x3f;   //code the first byte
//    Send_data(Address);
//	Send_data(value);
//    status = Rece_data(&EchoByte, 10000);
//	if(status == STATUS_SUCCESS)
//	{
//		if(Address == EchoByte)
//		{
//			
//		}
////			Rs522_cmd_cnt++;
//		else status = STARUS_ADDR_RERR;
//	}
//	return status;
//}

short WriteRawRC_HDL(unsigned char Address, unsigned char value)
{
    unsigned char EchoByte;
    short status;
	uint8_t e = 3;
	
    Address &= 0x3f;   //code the first byte
	for(e = 0; e<3; e++)
	{
		Send_data(Address);
		status = Rece_data(&EchoByte, 10000);
		if(status == STATUS_SUCCESS)
		{
			if(Address == EchoByte)
			{
				Send_data(value);
				break;
			}
			else
			{
				status = STARUS_ADDR_RERR;
			}
		}
	}
	return status;
}

/////////////////////////////////////////////////////////////////////
//��    �ܣ���RC522�Ĵ���λ
//����˵����reg[IN]:�Ĵ�����ַ
//          mask[IN]:��λֵ
/////////////////////////////////////////////////////////////////////
char SetBitMask(unsigned char reg,unsigned char mask)  
{
    char tmp = 0x0;
    tmp = ReadRawRC(reg);
    WriteRawRC(reg,tmp | mask);  // set bit mask
	return MI_OK;
}

/////////////////////////////////////////////////////////////////////
//��    �ܣ���RC522�Ĵ���λ
//����˵����reg[IN]:�Ĵ�����ַ
//          mask[IN]:��λֵ
/////////////////////////////////////////////////////////////////////
char ClearBitMask(unsigned char reg,unsigned char mask)  
{
    char tmp = 0x0;
    tmp = ReadRawRC(reg);
    WriteRawRC(reg, tmp & ~mask);  // clear bit mask
	return MI_OK;
}

/////////////////////////////////////////////////////////////////////
//��    �ܣ�ͨ��RC522��ISO14443��ͨѶ
//����˵����Command[IN]:RC522������
//          pInData[IN]:ͨ��RC522���͵���Ƭ������
//          InLenByte[IN]:�������ݵ��ֽڳ���
//          pOutData[OUT]:���յ��Ŀ�Ƭ��������
//          *pOutLenBit[OUT]:�������ݵ�λ����
/////////////////////////////////////////////////////////////////////
char PcdComMF522(unsigned char Command, 
                 unsigned char *pInData, 
                 unsigned char InLenByte,
                 unsigned char *pOutData, 
                 unsigned int  *pOutLenBit)
{
    char status = MI_ERR;
    unsigned char irqEn   = 0x00;
    unsigned char waitFor = 0x00;
    unsigned char lastBits;
    unsigned char n;
    unsigned int i;
	
    switch(Command)
    {
       case PCD_AUTHENT:
          irqEn   = 0x12;
          waitFor = 0x10;
          break;
	   
       case PCD_TRANSCEIVE:
          irqEn   = 0x77;
          waitFor = 0x30;
          break;
	   
       default:
         break;
    }
    WriteRawRC(ComIEnReg,irqEn|0x80);
    ClearBitMask(ComIrqReg,0x80);
    WriteRawRC(CommandReg,PCD_IDLE);
    SetBitMask(FIFOLevelReg,0x80);
    for (i=0; i<InLenByte; i++)
    {
		WriteRawRC(FIFODataReg, pInData[i]);
	}
    WriteRawRC(CommandReg, Command);
    if (Command == PCD_TRANSCEIVE)
    {
		SetBitMask(BitFramingReg,0x80);
	}
    i = 600;//����ʱ��Ƶ�ʵ���������M1�����ȴ�ʱ��25ms
    do 
    {
		n = ReadRawRC(ComIrqReg);
		i--;
    }
    while ((i!=0) && !(n&0x01) && !(n&waitFor));
    ClearBitMask(BitFramingReg,0x80);
    if(i != 0)
    {
		if(!(ReadRawRC(ErrorReg)&0x1B))
		{
			status = MI_OK;
			if (n & irqEn & 0x01)
			{
				status = MI_NOTAGERR;
			}
			if (Command == PCD_TRANSCEIVE)
			{
				n = ReadRawRC(FIFOLevelReg);
				lastBits = ReadRawRC(ControlReg) & 0x07;
				if(lastBits)
				{
					*pOutLenBit = (n-1)*8 + lastBits;
				}
				else
				{
					*pOutLenBit = n*8;
				}
				if (n == 0)
				{
					n = 1;
				}
				if(n > MAXRLEN)
				{
					n = MAXRLEN;
				}
				for (i=0; i<n; i++)
				{
					pOutData[i] = ReadRawRC(FIFODataReg);
				}
			}
		}
		else
		{
			status = MI_ERR;
		}
   }
   SetBitMask(ControlReg,0x80);           // stop timer now
   WriteRawRC(CommandReg,PCD_IDLE); 
   if(status)
   {
   }
   return status;
}

/////////////////////////////////////////////////////////////////////
//��������  
//ÿ��������ر����շ���֮��Ӧ������1ms�ļ��
/////////////////////////////////////////////////////////////////////
void PcdAntennaOn()
{
    unsigned char i;
    i = ReadRawRC(TxControlReg);
    if (!(i & 0x03))
    {
		SetBitMask(TxControlReg, 0x03);
    }
}


/////////////////////////////////////////////////////////////////////
//�ر�����
/////////////////////////////////////////////////////////////////////
void PcdAntennaOff()
{
    ClearBitMask(TxControlReg,0x03);
}

//�ȴ����뿪
void WaitCardOff(void)
{
	unsigned char status, TagType[2];

	while(1)
	{
		status = PcdRequest(REQ_ALL, TagType);
		if(status)
		{
			status = PcdRequest(REQ_ALL, TagType);
			if(status)
			{
				status = PcdRequest(REQ_ALL, TagType);
				if(status)
				{
					return;
				}
			}
		}
		delay_ms(10);
	}
}

uint8_t Rc522_GetLinkFlag(void)
{
	return Rc522_LinkFlag;
}

void Rc522_LinkTest(void)
{
	ReadRawRC(TxControlReg);
}

int8_t Rc522_OutSRst(void)
{
	char Rt = MI_ERR;
	uint8_t Rd=0;
	uint16_t i=0;
	
	for(i=0; i<100; i++)
	{
		Send_data(0x55);		
		if(Rece_data(&Rd, 10000) == STATUS_SUCCESS)
		{
			Send_data(0x55);
			delay_ms(100);
			Rt = MI_OK;
			break;
		}
	}
	return Rt;
}



