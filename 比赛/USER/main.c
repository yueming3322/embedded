#include <stdio.h>
#include "stm32f4xx.h"//总define
#include "delay.h"//延时函数
#include "infrared.h"//红外发送
#include "cba.h"//pin
#include "ultrasonic.h"//超声波
#include "canp_hostcom.h"//can发送数据wifi，zigbee
#include "hard_can.h"//检测can
#include "bh1750.h"//光照传感器
#include "power_check.h"//电量
#include "can_user.h"//通信交互数据回传
#include "data_base.h"//协议数组
#include "roadway_check.h"//循迹检测！
#include "tba.h"//任务板蜂鸣器、转向灯、光敏电阻
#include "data_base.h"
#include "swopt_drv.h"
#include "uart_a72.h"//串口
#include "Can_check.h"//can数据监测
#include "delay.h"
#include "can_user.h"
#include "Timer.h"
#include "Rc522.h"//RFID
#include "bkrc_voice.h"//语音交互
#include "drive.h"//驱动
#include "stdlib.h"//



static uint32_t Power_check_times;          // 电量检测周期
static uint32_t LED_twinkle_times;          // LED闪烁周期
static uint32_t WIFI_Upload_data_times;     // 通过Wifi上传数据周期
static uint32_t RFID_Init_Check_times;      // RFID初始化检测时间周期
static uint32_t task_times;                 // 任务时间检测时间周期

static void KEY_Check(void);                // 按键检测函数
static void Hardware_Init(void);            // 硬件初始化函数
//static uint8_t bkrc_voice_Flag = 0;         // 语音识别命令ID编号
uint8_t RFID_Flag = 0;          	           // RFID检测标志位
uint16_t distance = 0;                      // 记录超声波数据
uint8_t gear_init = 0;                      // 智能路灯初始档位
uint8_t coordinate;                         // 随机坐标点
uint8_t number = 0;                         // 计数值
uint8_t make = 0;                           // 全自动驾驶标志位
uint16_t random;                            // 随机数
uint8_t tf = 1;

uint8_t  Go_Speed  = 70;                    // 全局行进速度值
uint8_t  wheel_Speed = 100;                  // 全局转弯速度值
uint8_t  RFID_Go_Speed  = 35;               // RFID路段的速度根据新车、旧车自行调节
uint16_t Go_Temp = 350;                     // 全局前进码盘值
uint16_t wheel_Time = 330;                  // 全局转45°时间


uint8_t Stere1[6]={0xFF,0x14,0x01,0x00,0x00,0x00};		// 立体显示 显示 前方学习，减速慢行
uint8_t Stere2[6]={0xFF,0x14,0x02,0x00,0x00,0x00};		// 立体显示 显示 前方学习，减速慢行
uint8_t Stere3[6]={0xFF,0x14,0x03,0x00,0x00,0x00};		// 立体显示 显示 前方学习，减速慢行
uint8_t Stere4[6]={0xFF,0x14,0x04,0x00,0x00,0x00};		// 立体显示 显示 前方学习，减速慢行

uint8_t data1[3]={0x55,0xA1,0x01};		//告诉安卓识别二维码
uint8_t data2[3]={0x55,0xA1,0x02};		//告诉安卓识别二维码
uint8_t data3[3]={0x55,0xA1,0x03};		//告诉安卓识别二维码
uint8_t data4[3]={0x55,0xA1,0x04};		//告诉安卓识别二维码
uint8_t data5[3]={0x55,0xA1,0x10};		//告诉安卓识别二维码
uint8_t data6[3]={0x55,0xA1,0x11};		//告诉安卓识别二维码


//uint8_t data2[8]={0x55,0x01,0x10,0x00,0x00,0x00,0xA3,0xBB}; 		// LED显示 计时清零
//uint8_t data3[3]={0x55,0xA1,0x10};		//告诉安卓识别二维码




/* 全自动运行函数 */
void Car_Thread(void)
{
switch(make)
	{
      case 0x01:
      {
						Garage_Cont_Zigbee('A', 1);		                  //立体车库A至1层
					delay_ms(200);
						Send_ZigbeeData_To_Fifo(SMG_TimClear, 8);		   //计时清零
				delay_ms(200);
						Send_ZigbeeData_To_Fifo(SEG_TimOpen, 8);		   //打开计时
						delay_ms(200);
				//D7-D6
					Car_Track(Go_Speed);//trace是循迹函数
          Car_Go(Go_Speed,350);
					delay_ms(200);
					
					Car_Right(wheel_Speed);
					delay_ms(200);
				
				if(tf)
				{
					//etc的识别
					ETC_Get_Zigbee();//etc的识别
					delay_ms(200);
					task_times = gt_get() + 10000; //设置超时处理时间为10秒  
					tf = 0;
				}

				//	task_times = gt_get() + 5000; //设置超时处理时间为10秒  

				      //-------------超时处理函数-------------------
				 if(gt_get_sub(task_times) == 0)          // 运行指示灯
        {
           // task_times = gt_get() + 5000;          // LED4状态取反
				}
				//D6-F6
					Car_Track(Go_Speed);//trace是循迹函数
          Car_Go(Go_Speed,350);
					delay_ms(200);
				//语音播报任务
				Car_Right(wheel_Speed);
				delay_ms(200);
											//语音识别
											UART6_RxData[1] = 0x06;
											YY_Comm_Zigbee(0x20, 0x01);     // 语音播报随机语音命令
											delay_ms(500);
											BKRC_Voice_Extern(0);
											delay_ms(500);
				//上传评分终端
				
								//查询温度，记为F
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
				
				Car_Track(Go_Speed);//trace是循迹函数
        Car_Go(Go_Speed,350);//f4
				delay_ms(200);
				Car_Right(wheel_Speed);
				delay_ms(200);
				
							//超声波测距
							Ultrasonic_Ranging();//超声波测距
							delay_ms(200); 
						
							LED_Date_Zigbee(0x00,0x02,0x30,0x02);
							LED_Dis_Zigbee(180);//led显示测距信息
							delay_ms(500); 
							
								//发送TFT多功能显示器翻页
									TFT_Test_Zigbee('B',0x01,0x02,0x00,0x00);//图片显示向下翻页模式
									delay_ms(200);
									Send_Android(0xA4,0x00);
									delay_ms(500);
									Can_ZigBeeRx_Check();
									delay_ms(200);
									
									Car_Left(wheel_Speed);
									delay_ms(200);
									
									//识别二维码
									//打开安卓系统启用二维码识别，发送识别结果
									uint8_t data1[3]={0x55,0xA1,0x01};		//告诉安卓识别二维码
										//Send_WifiData_To_Fifo(data1,3);
									Send_Android(0xA2,0x00);
									delay_ms(200);
									Can_WifiRx_Check();//接受WiFi数据交互处理
									
//				Car_Left(wheel_Speed);
//				delay_ms(200);
				
				//Car_Track(Go_Speed);//trace是循迹函数
        Car_Go(Go_Speed,350);
				delay_ms(200);
				
				//F2
//				RFID_Flag = 1;
//					Track_duo_RFID(Go_Speed,6);     //使用循迹读卡函数
				Car_Right(100);
				delay_ms(200);
								
				
				
		
				//多功能A
					//发送TFT多功能显示器翻页
				TFT_Test_Zigbee('A',0x01,0x02,0x00,0x00);//图片显示向下翻页模式
				delay_ms(200);
				Send_Android(0xA4,0x00);
				delay_ms(200);
				Can_WifiRx_Check();
				delay_ms(200);
			
				
				Car_Left(100);
				delay_ms(200);
				Car_Left(100);
				delay_ms(200);
				
//				//右转90+45
//					Car_Right(wheel_Speed);
//					delay_ms(200);
//					Car_R45(wheel_Speed,400);//45度，需测试
//					delay_ms(300);
//				
//				//发送TFT多功能显示器翻页
//				TFT_Test_Zigbee('A',0x01,0x02,0x00,0x00);//图片显示向下翻页模式
//				delay_ms(200);
//				Send_Android(0xA4,0x00);
//				delay_ms(200);
//				Can_WifiRx_Check();
//				delay_ms(200);
//					uint8_t g;
//					unsigned char fanye[8]={0x55,0x0B,0x01,0x02,0x00,0x00,g,0xBB};
//					g=(fanye[2]+fanye[3]+fanye[4]+fanye[5])%256;
//					Send_ZigbeeData_To_Fifo(fanye,8);//图片显示向下翻页模式
					
				
					make = 0x02;
				  break;
			}
			case 0x02:
			{
					
				//特殊路段
					while(2)
				{
						Car_Go(50,100);
						delay_ms(300);
						
						Car_Back(50,100);
						
						Car_R45(40,50);//如何通过障碍路段，通过调节R45函数（转速），或者判断红外灯的亮灭情况
						delay_ms(300);
					break;
				}
					Car_Go(Go_Speed+20,1000);//
					delay_ms(200);
					Car_Left(100);
					delay_ms(200);
				//打开道闸
           Send_ZigbeeData_To_Fifo(Gate_Open, 8);		   // 打开道闸
           delay_ms(500);
           Send_ZigbeeData_To_Fifo(Gate_Open, 8);		   // 打开道闸
           delay_ms(500);
           delay_ms(500);
					Gate_Show_Zigbee("B8542D");//道闸显示车牌
					Send_ZigbeeData_To_Fifo(AGV_Thread1,8);//主从车通信
				//Car_Track(Go_Speed);//trace是循迹函数
        Car_Go(Go_Speed,350);
				delay_ms(200);
				//Car_Track(Go_Speed);//trace是循迹函数
        Car_Go(Go_Speed,350);
				delay_ms(200);
				
				Car_Right(wheel_Speed);
					delay_ms(200);
					
				Car_Track(Go_Speed);//trace是循迹函数
        Car_Go(Go_Speed,350);
				delay_ms(200);
				
				
					 
					
//					Ultrasonic_Ranging();//超声波测距
//					delay_ms(200); 
//					Rotate_Dis_Inf(230);//立体显示标志物显示测距信息
//					delay_ms(300); 
//					distance = dis;
//					LED_Dis_Zigbee(230);//led显示测距信息
//					delay_ms(300); 
//					TFT_Dis_Zigbee('A',230);//tft显示距离信息
//					delay_ms(300); 
//				
//					Rotate_show_Inf("C1GB",'F','6');//立体显示车牌信息
//					delay_ms(300); 
//					
//					   //打开道闸
//           Send_ZigbeeData_To_Fifo(Gate_Open, 8);		   // 打开道闸
//           delay_ms(500);
//           Send_ZigbeeData_To_Fifo(Gate_Open, 8);		   // 打开道闸
//           delay_ms(500);
//           delay_ms(500);
//					Gate_Show_Zigbee("D2870D");//道闸显示车牌
//					delay_ms(500);
//					
//				
//					 RFID_Flag = 1;
//					Track_duo_RFID(Go_Speed,6);     //使用循迹读卡函数
//					

//				TIM6_DAC_IRQHandler();
//				delay_ms(200); 
//				EXTI4_IRQHandler();
//				delay_ms(200); 

//				//rcc522 RFID先写后读
//					RC522(11,RFID_Write_Read);
//				
//				//打开安卓系统启用二维码识别，发送识别结果
//				uint8_t data1[3]={0x55,0xA1,0x01};		//告诉安卓识别二维码
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
//					//交通灯识别
//					Car_Track(Go_Speed);//trace是循迹函数
//				Car_Go(Go_Speed,350);
//				delay_ms(200);
//				//led显示
//				Car_Track(Go_Speed);//trace是循迹函数
//				Car_Go(Go_Speed,350);
//				delay_ms(200);
//					//路灯识别
//					Car_Left(wheel_Speed);
//					delay_ms(200);
//						//智能路灯调节至2挡
//					 gear_init = Light_Inf(2);
//					 delay_ms(500);
				
//					Car_Track(Go_Speed);
//          Car_Go(Go_Speed,Go_Temp);
//					delay_ms(200);
//					
//					Car_Right(wheel_Speed);
//					delay_ms(200);
//				
////				Get_Host_UpTrack(TRACK_Q7);//获取循迹数据
////				Get_Host_UpTrack(TRACK_H8);
//				//2
//				Ultrasonic_Ranging();//超声波测距
//					delay_ms(200); 
//				distance = dis;
//					Rotate_Dis_Inf(180);//立体显示标志物显示测距信息
//					delay_ms(300); 
//					LED_Dis_Zigbee(180);//led显示测距信息
//					delay_ms(300); 
//					TFT_Dis_Zigbee('A',180);//tft显示距离信息
//					delay_ms(300); 
//				
//					Rotate_show_Inf("R8&C",'F','6');//立体显示车牌信息
//					delay_ms(300); 
//					
//					   //打开道闸
//           Send_ZigbeeData_To_Fifo(Gate_Open, 8);		   // 打开道闸
//           delay_ms(500);
//           Send_ZigbeeData_To_Fifo(Gate_Open, 8);		   // 打开道闸
//           delay_ms(500);
//           delay_ms(500);
//					Gate_Show_Zigbee("B8542D");//道闸显示车牌
//					
//					//智能路灯调节至3挡
//           gear_init = Light_Inf(3);
//           delay_ms(500);
//				
//				//特殊路段任务
//				while(2)
//				{
//				Car_Go(50,100);
//						delay_ms(300);
//						
//						Car_Back(50,100);
//						
//						Car_R45(40,50);//如何通过障碍路段，通过调节R45函数（转速），或者判断红外灯的亮灭情况
//						delay_ms(300);
//					break;
//				}
//					Car_Go(Go_Speed+20,1000);//
//					delay_ms(200);
							
//				while(Track_Value==0xf6)
//				{}
//					while((TRACK_Q7<TRACK_H8)&&(TRACK_Q7>TRACK_H8))//假设左边的红外灯小于或者大于右边的 进行左右移动调整
//					{
//						Car_Go(50,100);
//						delay_ms(300);
//						
//						Car_Back(50,100);
//						
//						Car_R45(40,50);//如何通过障碍路段，通过调节R45函数（转速），或者判断红外灯的亮灭情况
//						delay_ms(300);
//						if(TRACK_Q7==(TRACK_H8-1))
//						{break;}
////						Car_R45(80,80);
////						delay_ms(300);
//						
//						Car_L45(40,50);//如何通过障碍路段，通过调节R45函数（转速），或者判断红外灯的亮灭情况
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
//					Send_Android(0XA2,0x00);//打开安卓系统,二维码识别系统
//					delay_ms(400);

//					//左转
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
//					//直走
//					Car_Track(Go_Speed);
//          Car_Go(Go_Speed,Go_Temp);
//					delay_ms(200);
//					
//					Car_Right(80);
//					delay_ms(400);
//					
//					//智能路灯识别系统
//					uint16_t h=dis;
//					gear_init = ((h/60)^(h/60))%4+1;	
//					delay_ms(400);
//					
//					//主从车通信
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
//					//车牌识别，还需要一个算法函数,用数组写
//					Gate_Show_Zigbee("A123B4");
//					delay_ms(200);
//					
//					Gate_Show_Zigbee("A123B4");
//					delay_ms(200);

//					Car_Track(Go_Speed);
//          Car_Go(Go_Speed,Go_Temp);
//					delay_ms(400);
//					//倒车入库
//					Car_Track(Go_Speed);//直走
//          Car_Go(Go_Speed,Go_Temp);
//					delay_ms(400);
//					Car_Right(90);//旋转90度
//					delay_ms(200);
//				  Car_Right(90);//再旋转90度
//					delay_ms(200);
//					Car_Track(40);//循迹
//					delay_ms(200);
//					Car_Back(60,1500);//后退入库
//					delay_ms(200);
//					
					make = 0x09;
					break;

			}
      case 0x09:
      {
    
				
           delay_ms(200);
   
           Garage_Cont_Zigbee('A', 3);		                  //立体车库A升至3层
           delay_ms(500);
					Send_ZigbeeData_To_Fifo(SMG_TimClose,8);   //LED开始关闭
					delay_ms(500);
					Send_ZigbeeData_To_Fifo(SMG_TimClose,8);
					delay_ms(500);
           Send_ZigbeeData_To_Fifo(Charge_Open, 8);		   //开启无线充电
           delay_ms(500);
           Send_ZigbeeData_To_Fifo(SMG_TimClose, 8);		   //关闭计时
           delay_ms(500);
           
           //task_times = gt_get() + 90000; //设置超时处理时间为90秒  
           make=0x10;
         break;
      }
      
      case 0x10:
      { 
				//从车 
				//通信协议
				
         //-------------超时处理函数-------------------
         if(gt_get_sub(task_times) == 0)      //超时处理
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
			uint8_t zigbee[8]={0x55,0x02,0x02,0x00,0x00,0x00,0x02,0xBB}; //主车发给从车
				
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




/* 按键检测函数 */
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
					Send_Android(0xA1,0x00);//与安卓通信
					Send_WifiData_To_Fifo(Principal_Tab,12);//与安卓通信
//			   	Car_Right(240);
//					delay_ms(400);
//					Send_InfoData_To_Fifo();//发送函数说明，上传调试信息，显示在debugger区
						
					Can_WifiRx_Check();//接受WiFi数据交互处理
					Can_ZigBeeRx_Check();//接受zibe数据交互处理
        }
    }
    if(S3 == 0)
    {
        delay_ms(10);
        if(S3 == 0)
        {
            LED3 = !LED3;
            while(!S3);
            //多功能A
					//发送TFT多功能显示器翻页
				TFT_Test_Zigbee('A',0x01,0x02,0x00,0x00);//图片显示向下翻页模式
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
           	Garage_Cont_Zigbee('A', 1);		                  //立体车库A至1层
				
						Send_ZigbeeData_To_Fifo(SMG_TimClear, 8);		   //计时清零
			
						Send_ZigbeeData_To_Fifo(SEG_TimOpen, 8);		   //打开计时
				 
        }
    }
}

int main(void)
{
    uint16_t Light_Value = 0;               // 光强度值
    uint16_t CodedDisk_Value = 0;           // 码盘值
    Hardware_Init();                        // 硬件初始化
    LED_twinkle_times = gt_get() + 50;
    Power_check_times = gt_get() + 200;
    WIFI_Upload_data_times = gt_get() + 200;
    RFID_Init_Check_times = gt_get() + 200;
    Principal_Tab[0] = 0x55;                // 主车数据上传指令包头
    Principal_Tab[1] = 0xAA;
    Follower_Tab[0] = 0x55;                 // 智能运输车数据上传指令包头
    Follower_Tab[1] = 0x02;
    Send_UpMotor(0, 0);
    while(1)
    {
        KEY_Check();                                    // 按键检测
        Can_WifiRx_Check();                             // Wifi交互数据处理
        Can_ZigBeeRx_Check();                           // Zigbee交互数据处理
		  Car_Thread();									        // 全自动运行程序
        if(gt_get_sub(LED_twinkle_times) == 0)          // 运行指示灯
        {
            LED_twinkle_times = gt_get() + 50;          // LED4状态取反
            LED4 = !LED4;
        }

        if(gt_get_sub(Power_check_times) == 0)          // 电池电量检测
        {
            Power_check_times = gt_get() + 200;
            Power_Check();
        }

#if 1
        if(gt_get_sub(RFID_Init_Check_times) == 0)      // RFID初始化检测
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
        if(gt_get_sub(WIFI_Upload_data_times) == 0)         // 数据上传
        {
            WIFI_Upload_data_times =  gt_get() + 500;
            if(Host_AGV_Return_Flag == RESET)               // 主车数据上传
            {
                Principal_Tab[2] = Stop_Flag;               // 运行状态
                Principal_Tab[3] = Get_tba_phsis_value();   // 光敏状态

                Ultrasonic_Ranging();                       // 超声波数据采集
                Principal_Tab[4] = dis % 256;               // 超声波数据低八位
                Principal_Tab[5] = dis / 256;               // 超声波数据高八位

                Light_Value = Get_Bh_Value();               // 光强度传感器数据采集
                Principal_Tab[6] = Light_Value % 256;       // 光强度数据低八位
                Principal_Tab[7] = Light_Value / 256;       // 光强度数据高八位

                CodedDisk_Value = CanHost_Mp;               // 码盘值
                Principal_Tab[8] = CodedDisk_Value % 256;
                Principal_Tab[9] = CodedDisk_Value / 256;

                Principal_Tab[10] = coordinate;             //返回随机救援坐标点
//                Send_WifiData_To_Fifo(Principal_Tab, 13);   // 通过Wifi上传主车数据
                UartA72_TxClear();
                UartA72_TxAddStr(Principal_Tab, 13);        // 通过串口上传主车数据
                UartA72_TxStart();
				coordinate = 0 ;
            }
            else if((Host_AGV_Return_Flag == SET) && (AGV_data_Falg == SET))
            {

                UartA72_TxClear();
                UartA72_TxAddStr(Follower_Tab, 50);         // 通过串口上传从车数据
                UartA72_TxStart();
                Send_WifiData_To_Fifo(Follower_Tab, 50);    // 通过Wifi上传从车数据
                AGV_data_Falg = 0;
            }
        }
    }
}

/* 硬件初始化函数 */
void Hardware_Init(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);     // 中断分组
    delay_init(168);                                    // 延时初始化
    Tba_Init();                                         // 任务板初始化
    Infrared_Init();                                    // 红外初始化
    Cba_Init();                                         // 核心板初始化
    Ultrasonic_Init();                                  // 超声波初始化
    Hard_Can_Init();                                    // CAN总线初始化
    BH1750_Configure();                                 // BH1750初始化配置
    BKRC_Voice_Init();									// 小创语音模块初始化
    Electricity_Init();                                 // 电量检测初始化
    UartA72_Init();                                     // A72硬件串口通讯初始化
    Can_check_Init(7, 83);                              // CAN总线定时器初始化
    roadway_check_TimInit(999, 167);                   	// 路况检测
    Timer_Init(999, 167);                               // 串行数据通讯时间帧
    Readcard_daivce_Init();                         	// RFID初始化
}

