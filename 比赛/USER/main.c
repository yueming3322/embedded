#include <stdio.h>
#include "stm32f4xx.h"//��define
#include "delay.h"//��ʱ����
#include "infrared.h"//���ⷢ��
#include "cba.h"//pin
#include "ultrasonic.h"//������
#include "canp_hostcom.h"//can��������wifi��zigbee
#include "hard_can.h"//���can
#include "bh1750.h"//���մ�����
#include "power_check.h"//����
#include "can_user.h"//ͨ�Ž������ݻش�
#include "data_base.h"//Э������
#include "roadway_check.h"//ѭ����⣡
#include "tba.h"//������������ת��ơ���������
#include "data_base.h"
#include "swopt_drv.h"
#include "uart_a72.h"//����
#include "Can_check.h"//can���ݼ��
#include "delay.h"
#include "can_user.h"
#include "Timer.h"
#include "Rc522.h"//RFID
#include "bkrc_voice.h"//��������
#include "drive.h"//����
#include "stdlib.h"//



static uint32_t Power_check_times;          // �����������
static uint32_t LED_twinkle_times;          // LED��˸����
static uint32_t WIFI_Upload_data_times;     // ͨ��Wifi�ϴ���������
static uint32_t RFID_Init_Check_times;      // RFID��ʼ�����ʱ������
static uint32_t task_times;                 // ����ʱ����ʱ������

static void KEY_Check(void);                // ������⺯��
static void Hardware_Init(void);            // Ӳ����ʼ������
//static uint8_t bkrc_voice_Flag = 0;         // ����ʶ������ID���
uint8_t RFID_Flag = 0;          	           // RFID����־λ
uint16_t distance = 0;                      // ��¼����������
uint8_t gear_init = 0;                      // ����·�Ƴ�ʼ��λ
uint8_t coordinate;                         // ��������
uint8_t number = 0;                         // ����ֵ
uint8_t make = 0;                           // ȫ�Զ���ʻ��־λ
uint16_t random;                            // �����
uint8_t tf = 1;

uint8_t  Go_Speed  = 70;                    // ȫ���н��ٶ�ֵ
uint8_t  wheel_Speed = 100;                  // ȫ��ת���ٶ�ֵ
uint8_t  RFID_Go_Speed  = 35;               // RFID·�ε��ٶȸ����³����ɳ����е���
uint16_t Go_Temp = 350;                     // ȫ��ǰ������ֵ
uint16_t wheel_Time = 330;                  // ȫ��ת45��ʱ��


uint8_t Stere1[6]={0xFF,0x14,0x01,0x00,0x00,0x00};		// ������ʾ ��ʾ ǰ��ѧϰ����������
uint8_t Stere2[6]={0xFF,0x14,0x02,0x00,0x00,0x00};		// ������ʾ ��ʾ ǰ��ѧϰ����������
uint8_t Stere3[6]={0xFF,0x14,0x03,0x00,0x00,0x00};		// ������ʾ ��ʾ ǰ��ѧϰ����������
uint8_t Stere4[6]={0xFF,0x14,0x04,0x00,0x00,0x00};		// ������ʾ ��ʾ ǰ��ѧϰ����������

uint8_t data1[3]={0x55,0xA1,0x01};		//���߰�׿ʶ���ά��
uint8_t data2[3]={0x55,0xA1,0x02};		//���߰�׿ʶ���ά��
uint8_t data3[3]={0x55,0xA1,0x03};		//���߰�׿ʶ���ά��
uint8_t data4[3]={0x55,0xA1,0x04};		//���߰�׿ʶ���ά��
uint8_t data5[3]={0x55,0xA1,0x10};		//���߰�׿ʶ���ά��
uint8_t data6[3]={0x55,0xA1,0x11};		//���߰�׿ʶ���ά��


//uint8_t data2[8]={0x55,0x01,0x10,0x00,0x00,0x00,0xA3,0xBB}; 		// LED��ʾ ��ʱ����
//uint8_t data3[3]={0x55,0xA1,0x10};		//���߰�׿ʶ���ά��




/* ȫ�Զ����к��� */
void Car_Thread(void)
{
switch(make)
	{
      case 0x01:
      {
						Garage_Cont_Zigbee('A', 1);		                  //���峵��A��1��
					delay_ms(200);
						Send_ZigbeeData_To_Fifo(SMG_TimClear, 8);		   //��ʱ����
				delay_ms(200);
						Send_ZigbeeData_To_Fifo(SEG_TimOpen, 8);		   //�򿪼�ʱ
						delay_ms(200);
				//D7-D6
					Car_Track(Go_Speed);//trace��ѭ������
          Car_Go(Go_Speed,350);
					delay_ms(200);
					
					Car_Right(wheel_Speed);
					delay_ms(200);
				
				if(tf)
				{
					//etc��ʶ��
					ETC_Get_Zigbee();//etc��ʶ��
					delay_ms(200);
					task_times = gt_get() + 10000; //���ó�ʱ����ʱ��Ϊ10��  
					tf = 0;
				}

				//	task_times = gt_get() + 5000; //���ó�ʱ����ʱ��Ϊ10��  

				      //-------------��ʱ������-------------------
				 if(gt_get_sub(task_times) == 0)          // ����ָʾ��
        {
           // task_times = gt_get() + 5000;          // LED4״̬ȡ��
				}
				//D6-F6
					Car_Track(Go_Speed);//trace��ѭ������
          Car_Go(Go_Speed,350);
					delay_ms(200);
				//������������
				Car_Right(wheel_Speed);
				delay_ms(200);
											//����ʶ��
											UART6_RxData[1] = 0x06;
											YY_Comm_Zigbee(0x20, 0x01);     // �������������������
											delay_ms(500);
											BKRC_Voice_Extern(0);
											delay_ms(500);
				//�ϴ������ն�
				
								//��ѯ�¶ȣ���ΪF
								int g=(0x43+0x00+0x00+0x00)%256;
								uint8_t wendu[8]={0x05,0x06,0x43,0x00,0x00,0x00,g,0xBB};
								Send_ZigbeeData_To_Fifo(wendu,8);
								delay_ms(200);
								delay_ms(500);
								Can_ZigBeeRx_Check();
				Car_Left(wheel_Speed);
				delay_ms(200);
				Car_Left(wheel_Speed);
				delay_ms(200);
				
				Car_Track(Go_Speed);//trace��ѭ������
        Car_Go(Go_Speed,350);//f4
				delay_ms(200);
				Car_Right(wheel_Speed);
				delay_ms(200);
				
							//���������
							Ultrasonic_Ranging();//���������
							delay_ms(200); 
						
							LED_Date_Zigbee(0x00,0x02,0x30,0x02);
							LED_Dis_Zigbee(180);//led��ʾ�����Ϣ
							delay_ms(500); 
							
								//����TFT�๦����ʾ����ҳ
									TFT_Test_Zigbee('B',0x01,0x02,0x00,0x00);//ͼƬ��ʾ���·�ҳģʽ
									delay_ms(200);
									Send_Android(0xA4,0x00);
									delay_ms(500);
									Can_ZigBeeRx_Check();
									delay_ms(200);
									
									Car_Left(wheel_Speed);
									delay_ms(200);
									
									//ʶ���ά��
									//�򿪰�׿ϵͳ���ö�ά��ʶ�𣬷���ʶ����
									uint8_t data1[3]={0x55,0xA1,0x01};		//���߰�׿ʶ���ά��
										//Send_WifiData_To_Fifo(data1,3);
									Send_Android(0xA2,0x00);
									delay_ms(200);
									Can_WifiRx_Check();//����WiFi���ݽ�������
									
//				Car_Left(wheel_Speed);
//				delay_ms(200);
				
				//Car_Track(Go_Speed);//trace��ѭ������
        Car_Go(Go_Speed,350);
				delay_ms(200);
				
				//F2
//				RFID_Flag = 1;
//					Track_duo_RFID(Go_Speed,6);     //ʹ��ѭ����������
				Car_Right(100);
				delay_ms(200);
								
				
				
		
				//�๦��A
					//����TFT�๦����ʾ����ҳ
				TFT_Test_Zigbee('A',0x01,0x02,0x00,0x00);//ͼƬ��ʾ���·�ҳģʽ
				delay_ms(200);
				Send_Android(0xA4,0x00);
				delay_ms(200);
				Can_WifiRx_Check();
				delay_ms(200);
			
				
				Car_Left(100);
				delay_ms(200);
				Car_Left(100);
				delay_ms(200);
				
//				//��ת90+45
//					Car_Right(wheel_Speed);
//					delay_ms(200);
//					Car_R45(wheel_Speed,400);//45�ȣ������
//					delay_ms(300);
//				
//				//����TFT�๦����ʾ����ҳ
//				TFT_Test_Zigbee('A',0x01,0x02,0x00,0x00);//ͼƬ��ʾ���·�ҳģʽ
//				delay_ms(200);
//				Send_Android(0xA4,0x00);
//				delay_ms(200);
//				Can_WifiRx_Check();
//				delay_ms(200);
//					uint8_t g;
//					unsigned char fanye[8]={0x55,0x0B,0x01,0x02,0x00,0x00,g,0xBB};
//					g=(fanye[2]+fanye[3]+fanye[4]+fanye[5])%256;
//					Send_ZigbeeData_To_Fifo(fanye,8);//ͼƬ��ʾ���·�ҳģʽ
					
				
					make = 0x02;
				  break;
			}
			case 0x02:
			{
					
				//����·��
					while(2)
				{
						Car_Go(50,100);
						delay_ms(300);
						
						Car_Back(50,100);
						
						Car_R45(40,50);//���ͨ���ϰ�·�Σ�ͨ������R45������ת�٣��������жϺ���Ƶ��������
						delay_ms(300);
					break;
				}
					Car_Go(Go_Speed+20,1000);//
					delay_ms(200);
					Car_Left(100);
					delay_ms(200);
				//�򿪵�բ
           Send_ZigbeeData_To_Fifo(Gate_Open, 8);		   // �򿪵�բ
           delay_ms(500);
           Send_ZigbeeData_To_Fifo(Gate_Open, 8);		   // �򿪵�բ
           delay_ms(500);
           delay_ms(500);
					Gate_Show_Zigbee("B8542D");//��բ��ʾ����
					Send_ZigbeeData_To_Fifo(AGV_Thread1,8);//���ӳ�ͨ��
				//Car_Track(Go_Speed);//trace��ѭ������
        Car_Go(Go_Speed,350);
				delay_ms(200);
				//Car_Track(Go_Speed);//trace��ѭ������
        Car_Go(Go_Speed,350);
				delay_ms(200);
				
				Car_Right(wheel_Speed);
					delay_ms(200);
					
				Car_Track(Go_Speed);//trace��ѭ������
        Car_Go(Go_Speed,350);
				delay_ms(200);
				
				
					 
					
//					Ultrasonic_Ranging();//���������
//					delay_ms(200); 
//					Rotate_Dis_Inf(230);//������ʾ��־����ʾ�����Ϣ
//					delay_ms(300); 
//					distance = dis;
//					LED_Dis_Zigbee(230);//led��ʾ�����Ϣ
//					delay_ms(300); 
//					TFT_Dis_Zigbee('A',230);//tft��ʾ������Ϣ
//					delay_ms(300); 
//				
//					Rotate_show_Inf("C1GB",'F','6');//������ʾ������Ϣ
//					delay_ms(300); 
//					
//					   //�򿪵�բ
//           Send_ZigbeeData_To_Fifo(Gate_Open, 8);		   // �򿪵�բ
//           delay_ms(500);
//           Send_ZigbeeData_To_Fifo(Gate_Open, 8);		   // �򿪵�բ
//           delay_ms(500);
//           delay_ms(500);
//					Gate_Show_Zigbee("D2870D");//��բ��ʾ����
//					delay_ms(500);
//					
//				
//					 RFID_Flag = 1;
//					Track_duo_RFID(Go_Speed,6);     //ʹ��ѭ����������
//					

//				TIM6_DAC_IRQHandler();
//				delay_ms(200); 
//				EXTI4_IRQHandler();
//				delay_ms(200); 

//				//rcc522 RFID��д���
//					RC522(11,RFID_Write_Read);
//				
//				//�򿪰�׿ϵͳ���ö�ά��ʶ�𣬷���ʶ����
//				uint8_t data1[3]={0x55,0xA1,0x01};		//���߰�׿ʶ���ά��
//          //Send_WifiData_To_Fifo(data1,3);
//					Send_Android(0xA2,0x00);
//					delay_ms(200);
					
				
				
				   	make = 0x03;
        
         
				 break;
			 }
				 
			case 0x03:
			{
//				Car_Right(wheel_Speed);
//					delay_ms(200);
//					//��ͨ��ʶ��
//					Car_Track(Go_Speed);//trace��ѭ������
//				Car_Go(Go_Speed,350);
//				delay_ms(200);
//				//led��ʾ
//				Car_Track(Go_Speed);//trace��ѭ������
//				Car_Go(Go_Speed,350);
//				delay_ms(200);
//					//·��ʶ��
//					Car_Left(wheel_Speed);
//					delay_ms(200);
//						//����·�Ƶ�����2��
//					 gear_init = Light_Inf(2);
//					 delay_ms(500);
				
//					Car_Track(Go_Speed);
//          Car_Go(Go_Speed,Go_Temp);
//					delay_ms(200);
//					
//					Car_Right(wheel_Speed);
//					delay_ms(200);
//				
////				Get_Host_UpTrack(TRACK_Q7);//��ȡѭ������
////				Get_Host_UpTrack(TRACK_H8);
//				//2
//				Ultrasonic_Ranging();//���������
//					delay_ms(200); 
//				distance = dis;
//					Rotate_Dis_Inf(180);//������ʾ��־����ʾ�����Ϣ
//					delay_ms(300); 
//					LED_Dis_Zigbee(180);//led��ʾ�����Ϣ
//					delay_ms(300); 
//					TFT_Dis_Zigbee('A',180);//tft��ʾ������Ϣ
//					delay_ms(300); 
//				
//					Rotate_show_Inf("R8&C",'F','6');//������ʾ������Ϣ
//					delay_ms(300); 
//					
//					   //�򿪵�բ
//           Send_ZigbeeData_To_Fifo(Gate_Open, 8);		   // �򿪵�բ
//           delay_ms(500);
//           Send_ZigbeeData_To_Fifo(Gate_Open, 8);		   // �򿪵�բ
//           delay_ms(500);
//           delay_ms(500);
//					Gate_Show_Zigbee("B8542D");//��բ��ʾ����
//					
//					//����·�Ƶ�����3��
//           gear_init = Light_Inf(3);
//           delay_ms(500);
//				
//				//����·������
//				while(2)
//				{
//				Car_Go(50,100);
//						delay_ms(300);
//						
//						Car_Back(50,100);
//						
//						Car_R45(40,50);//���ͨ���ϰ�·�Σ�ͨ������R45������ת�٣��������жϺ���Ƶ��������
//						delay_ms(300);
//					break;
//				}
//					Car_Go(Go_Speed+20,1000);//
//					delay_ms(200);
							
//				while(Track_Value==0xf6)
//				{}
//					while((TRACK_Q7<TRACK_H8)&&(TRACK_Q7>TRACK_H8))//������ߵĺ����С�ڻ��ߴ����ұߵ� ���������ƶ�����
//					{
//						Car_Go(50,100);
//						delay_ms(300);
//						
//						Car_Back(50,100);
//						
//						Car_R45(40,50);//���ͨ���ϰ�·�Σ�ͨ������R45������ת�٣��������жϺ���Ƶ��������
//						delay_ms(300);
//						if(TRACK_Q7==(TRACK_H8-1))
//						{break;}
////						Car_R45(80,80);
////						delay_ms(300);
//						
//						Car_L45(40,50);//���ͨ���ϰ�·�Σ�ͨ������R45������ת�٣��������жϺ���Ƶ��������
//						delay_ms(300);
//						if(TRACK_Q7==(TRACK_H8-1))
//						{break;}
//					}
//			}
				
					
					
//					Car_Track(Go_Speed);
//          Car_Go(Go_Speed,Go_Temp);
//					delay_ms(400);
//					
//					Car_Track(Go_Speed);
//          Car_Go(Go_Speed,Go_Temp);
//					delay_ms(400);
//					
//					Send_Android(0XA2,0x00);//�򿪰�׿ϵͳ,��ά��ʶ��ϵͳ
//					delay_ms(400);

//					//��ת
//					Car_Left(wheel_Speed);
//					delay_ms(200);
//					
//					Car_Track(Go_Speed);
//          Car_Go(Go_Speed,Go_Temp);
//					delay_ms(200);
//					
//					
//					
//					Car_Left(wheel_Speed);
//					delay_ms(200);
//					
//					
//					//ֱ��
//					Car_Track(Go_Speed);
//          Car_Go(Go_Speed,Go_Temp);
//					delay_ms(200);
//					
//					Car_Right(80);
//					delay_ms(400);
//					
//					//����·��ʶ��ϵͳ
//					uint16_t h=dis;
//					gear_init = ((h/60)^(h/60))%4+1;	
//					delay_ms(400);
//					
//					//���ӳ�ͨ��
//					Send_ZigbeeData_To_Fifo(AGV_Thread1,8);
//					
//					Car_Left(wheel_Speed);
//					delay_ms(200);
//					
//					Car_Track(Go_Speed);
//          Car_Go(Go_Speed,Go_Temp);
//					delay_ms(400);
//					
//					Car_Left(wheel_Speed);
//					delay_ms(200);
//					
//					//����ʶ�𣬻���Ҫһ���㷨����,������д
//					Gate_Show_Zigbee("A123B4");
//					delay_ms(200);
//					
//					Gate_Show_Zigbee("A123B4");
//					delay_ms(200);

//					Car_Track(Go_Speed);
//          Car_Go(Go_Speed,Go_Temp);
//					delay_ms(400);
//					//�������
//					Car_Track(Go_Speed);//ֱ��
//          Car_Go(Go_Speed,Go_Temp);
//					delay_ms(400);
//					Car_Right(90);//��ת90��
//					delay_ms(200);
//				  Car_Right(90);//����ת90��
//					delay_ms(200);
//					Car_Track(40);//ѭ��
//					delay_ms(200);
//					Car_Back(60,1500);//�������
//					delay_ms(200);
//					
					make = 0x09;
					break;

			}
      case 0x09:
      {
    
				
           delay_ms(200);
   
           Garage_Cont_Zigbee('A', 3);		                  //���峵��A����3��
           delay_ms(500);
					Send_ZigbeeData_To_Fifo(SMG_TimClose,8);   //LED��ʼ�ر�
					delay_ms(500);
					Send_ZigbeeData_To_Fifo(SMG_TimClose,8);
					delay_ms(500);
           Send_ZigbeeData_To_Fifo(Charge_Open, 8);		   //�������߳��
           delay_ms(500);
           Send_ZigbeeData_To_Fifo(SMG_TimClose, 8);		   //�رռ�ʱ
           delay_ms(500);
           
           //task_times = gt_get() + 90000; //���ó�ʱ����ʱ��Ϊ90��  
           make=0x10;
         break;
      }
      
      case 0x10:
      { 
				//�ӳ� 
				//ͨ��Э��
				
         //-------------��ʱ������-------------------
         if(gt_get_sub(task_times) == 0)      //��ʱ����
         {
           make = 0x11;
         } 
         else if(Zigbee_Rx_flag == 1)	     
         {
            Zigbee_Rx_flag =0;
            if((Zigb_Rx_Buf[0] == 0x55) && (Zigb_Rx_Buf[1] == 0x02))     // 55 02 0D
            {
               if(Zigb_Rx_Buf[2] == 0x00)
               {
                  make =0;
               }
            }
         }           
         break;
      }
      case 0x11:
      {
			uint8_t zigbee[8]={0x55,0x02,0x02,0x00,0x00,0x00,0x02,0xBB}; //���������ӳ�
				
			//uint8_t ZigBee_back[16] = { 0x55, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
			make=0;
			break;
//				//ExtSRAMInterface.ExMem_Write_Bytes(a,s,l);
//					Car_Track(Go_Speed);
//          Car_Go(Go_Speed,Go_Temp);
//					delay_ms(200);
//					
//					Car_Right(wheel_Speed);
//					delay_ms(200);
//				
//					Car_Track(Go_Speed);
//          Car_Go(Go_Speed,Go_Temp);
//					delay_ms(200);
//				
//					Car_Left(wheel_Speed);
//					delay_ms(200);
//				
//					Car_Track(Go_Speed);
//          Car_Go(Go_Speed,Go_Temp);
//					delay_ms(200);
//				
//					Car_Right(wheel_Speed);
//					delay_ms(200);
//				
//					Car_Track(Go_Speed);
//          Car_Go(Go_Speed,Go_Temp);
//					delay_ms(200);
//					
//					Car_Left(wheel_Speed);
//					delay_ms(200);
//					
//					Car_Track(Go_Speed);
//          Car_Go(Go_Speed,Go_Temp);
//					delay_ms(200);
//					
//					Car_Right(wheel_Speed);
//					delay_ms(200);
//				
//				Car_Track(Go_Speed);
//          Car_Go(Go_Speed,Go_Temp);
//					delay_ms(200);
//		
         
      }
      default : 
       break;		
	}
}




/* ������⺯�� */
void KEY_Check(void)
{
    if(S1 == 0)
    {
        delay_ms(10);
        if(S1 == 0)
        {
            LED1 = !LED1;
            while(!S1);
					
           // Send_WifiData_To_Fifo(data1, 3);		   //
           make=0x01;
					
        }
    }
    if(S2 == 0)
    {
        delay_ms(10);
        if(S2 == 0)
        {
            LED2 = !LED2;
            while(!S2);
					Send_Android(0xA1,0x00);//�밲׿ͨ��
					Send_WifiData_To_Fifo(Principal_Tab,12);//�밲׿ͨ��
//			   	Car_Right(240);
//					delay_ms(400);
//					Send_InfoData_To_Fifo();//���ͺ���˵�����ϴ�������Ϣ����ʾ��debugger��
						
					Can_WifiRx_Check();//����WiFi���ݽ�������
					Can_ZigBeeRx_Check();//����zibe���ݽ�������
        }
    }
    if(S3 == 0)
    {
        delay_ms(10);
        if(S3 == 0)
        {
            LED3 = !LED3;
            while(!S3);
            //�๦��A
					//����TFT�๦����ʾ����ҳ
				TFT_Test_Zigbee('A',0x01,0x02,0x00,0x00);//ͼƬ��ʾ���·�ҳģʽ
				delay_ms(200);
				Send_Android(0xA4,0x00);
				delay_ms(200);
				Can_WifiRx_Check();
				delay_ms(200);
			
				
				Car_Left(100);
				delay_ms(200);
				Car_Left(100);
				delay_ms(200);
				
        }
    }
    if(S4 == 0)
    {
        delay_ms(10);
        if(S4 == 0)
		   {
            while(!S4);
           	Garage_Cont_Zigbee('A', 1);		                  //���峵��A��1��
				
						Send_ZigbeeData_To_Fifo(SMG_TimClear, 8);		   //��ʱ����
			
						Send_ZigbeeData_To_Fifo(SEG_TimOpen, 8);		   //�򿪼�ʱ
				 
        }
    }
}

int main(void)
{
    uint16_t Light_Value = 0;               // ��ǿ��ֵ
    uint16_t CodedDisk_Value = 0;           // ����ֵ
    Hardware_Init();                        // Ӳ����ʼ��
    LED_twinkle_times = gt_get() + 50;
    Power_check_times = gt_get() + 200;
    WIFI_Upload_data_times = gt_get() + 200;
    RFID_Init_Check_times = gt_get() + 200;
    Principal_Tab[0] = 0x55;                // ���������ϴ�ָ���ͷ
    Principal_Tab[1] = 0xAA;
    Follower_Tab[0] = 0x55;                 // �������䳵�����ϴ�ָ���ͷ
    Follower_Tab[1] = 0x02;
    Send_UpMotor(0, 0);
    while(1)
    {
        KEY_Check();                                    // �������
        Can_WifiRx_Check();                             // Wifi�������ݴ���
        Can_ZigBeeRx_Check();                           // Zigbee�������ݴ���
		  Car_Thread();									        // ȫ�Զ����г���
        if(gt_get_sub(LED_twinkle_times) == 0)          // ����ָʾ��
        {
            LED_twinkle_times = gt_get() + 50;          // LED4״̬ȡ��
            LED4 = !LED4;
        }

        if(gt_get_sub(Power_check_times) == 0)          // ��ص������
        {
            Power_check_times = gt_get() + 200;
            Power_Check();
        }

#if 1
        if(gt_get_sub(RFID_Init_Check_times) == 0)      // RFID��ʼ�����
        {
            RFID_Init_Check_times =  gt_get() + 200;
            if(Rc522_GetLinkFlag() == 0)
            {
                Readcard_daivce_Init();
                MP_SPK = !MP_SPK;
            }
            else
            {
				MP_SPK = 0;
				LED1 = !LED1;
				Rc522_LinkTest();
            }
        }
#endif
        if(gt_get_sub(WIFI_Upload_data_times) == 0)         // �����ϴ�
        {
            WIFI_Upload_data_times =  gt_get() + 500;
            if(Host_AGV_Return_Flag == RESET)               // ���������ϴ�
            {
                Principal_Tab[2] = Stop_Flag;               // ����״̬
                Principal_Tab[3] = Get_tba_phsis_value();   // ����״̬

                Ultrasonic_Ranging();                       // ���������ݲɼ�
                Principal_Tab[4] = dis % 256;               // ���������ݵͰ�λ
                Principal_Tab[5] = dis / 256;               // ���������ݸ߰�λ

                Light_Value = Get_Bh_Value();               // ��ǿ�ȴ��������ݲɼ�
                Principal_Tab[6] = Light_Value % 256;       // ��ǿ�����ݵͰ�λ
                Principal_Tab[7] = Light_Value / 256;       // ��ǿ�����ݸ߰�λ

                CodedDisk_Value = CanHost_Mp;               // ����ֵ
                Principal_Tab[8] = CodedDisk_Value % 256;
                Principal_Tab[9] = CodedDisk_Value / 256;

                Principal_Tab[10] = coordinate;             //���������Ԯ�����
//                Send_WifiData_To_Fifo(Principal_Tab, 13);   // ͨ��Wifi�ϴ���������
                UartA72_TxClear();
                UartA72_TxAddStr(Principal_Tab, 13);        // ͨ�������ϴ���������
                UartA72_TxStart();
				coordinate = 0 ;
            }
            else if((Host_AGV_Return_Flag == SET) && (AGV_data_Falg == SET))
            {

                UartA72_TxClear();
                UartA72_TxAddStr(Follower_Tab, 50);         // ͨ�������ϴ��ӳ�����
                UartA72_TxStart();
                Send_WifiData_To_Fifo(Follower_Tab, 50);    // ͨ��Wifi�ϴ��ӳ�����
                AGV_data_Falg = 0;
            }
        }
    }
}

/* Ӳ����ʼ������ */
void Hardware_Init(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);     // �жϷ���
    delay_init(168);                                    // ��ʱ��ʼ��
    Tba_Init();                                         // ������ʼ��
    Infrared_Init();                                    // �����ʼ��
    Cba_Init();                                         // ���İ��ʼ��
    Ultrasonic_Init();                                  // ��������ʼ��
    Hard_Can_Init();                                    // CAN���߳�ʼ��
    BH1750_Configure();                                 // BH1750��ʼ������
    BKRC_Voice_Init();									// С������ģ���ʼ��
    Electricity_Init();                                 // ��������ʼ��
    UartA72_Init();                                     // A72Ӳ������ͨѶ��ʼ��
    Can_check_Init(7, 83);                              // CAN���߶�ʱ����ʼ��
    roadway_check_TimInit(999, 167);                   	// ·�����
    Timer_Init(999, 167);                               // ��������ͨѶʱ��֡
    Readcard_daivce_Init();                         	// RFID��ʼ��
}

