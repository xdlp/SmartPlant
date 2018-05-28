#include "embARC.h"
#include "embARC_debug.h"

#include "u8g.h"
#include "dev_gpio.h" //GPIO API头文件



unsigned char PCF8591_DATA_channlel[4];



//=====================================
//===============dht22 io 相关=========
//=====================================
typedef unsigned char  U8;       /* defined for unsigned 8-bits integer variable 	  无符号8位整型变量  */
typedef signed   char  S8;       /* defined for signed 8-bits integer variable		  有符号8位整型变量  */
typedef unsigned int   U16;      /* defined for unsigned 16-bits integer variable 	  无符号16位整型变量 */
typedef signed   int   S16;      /* defined for signed 16-bits integer variable 	  有符号16位整型变量 */
typedef unsigned long  U32;      /* defined for unsigned 32-bits integer variable 	  无符号32位整型变量 */
typedef signed   long  S32;      /* defined for signed 32-bits integer variable 	  有符号32位整型变量 */
typedef float          F32;      /* single precision floating point variable (32bits) 单精度浮点数（32位长度） */
typedef double         F64;      /* double precision floating point variable (64bits) 双精度浮点数（64位长度） */

U8  U8FLAG;
U8  U8temp;
U8  U8T_data_H,U8T_data_L,U8RH_data_H,U8RH_data_L,U8checkdata;
U8  U8T_data_H_temp,U8T_data_L_temp,U8RH_data_H_temp,U8RH_data_L_temp,U8checkdata_temp;
U8  U8comdata;

//======================================================================================
//===================定时器0终端服务子程序==============================================
//======================================================================================
volatile static int cnt0 = 1000;
static void timer0_isr(void *ptr)
{
	timer_int_clear(TIMER_0);
	cnt0--;
	if(cnt0<=0)
		{
			
			int_disable(INTNO_TIMER0);
			timer_stop(INTNO_TIMER0);
		}
	
	//EMBARC_PRINTF("timer0 interrupt:%d\n", cnt0);
}

static void time0_init()
{
	int_disable(INTNO_TIMER0);
	timer_stop(INTNO_TIMER0);
	int_handler_install(INTNO_TIMER0, timer0_isr);//关联中断服务子程序
	int_enable(INTNO_TIMER0);
	
		
}	










//========================================================================================















void delay_us(unsigned   long  t)
{
	cnt0=t;
	int_disable(INTNO_TIMER0);
	timer_stop(INTNO_TIMER0);
	int_handler_install(INTNO_TIMER0, timer0_isr);//关联中断服务子程序
	int_enable(INTNO_TIMER0);
	timer_start(TIMER_0, TIMER_CTRL_IE,BOARD_CPU_CLOCK/1000/100);
	while(cnt0>0)
		{
			//EMBARC_PRINTF("mmp:%d\n", cnt0);
			
		}
	

	
	
}
static DEV_GPIO_PTR port_dht22;
static void dht22_init()
{
	
//	set_pmod_mux(PMOD_MUX_CTRL,PM3_GPIO_AC);
	port_dht22 = gpio_get_dev(DW_GPIO_PORT_A);
	port_dht22->gpio_open(0x100);
	//port_dht22->gpio_control(GPIO_CMD_SET_BIT_DIR_INPUT,(void *)0x100);
	
	
}
#define dht22_out port_dht22->gpio_control(GPIO_CMD_SET_BIT_DIR_OUTPUT,(void *)0x100);//;(port_dht22,0x10000);
#define dht22_in port_dht22->gpio_control(GPIO_CMD_SET_BIT_DIR_INPUT,(void *)0x100);
	
void  dht22COM(void)
      {
     
	        U8 i;
			uint32_t data;

			
        dht22_in;
		
       for(i=0;i<8;i++)	   
	    {
		
	   	    U8FLAG=2;	
		port_dht22->gpio_read(&data,0x100);
		data =(data&0x00000100)>>8;
		
		while((!data) &&  U8FLAG++)
		{
			port_dht22->gpio_read(&data,0x100);
			data =(data&0x00000100)>>8;
			
		}
			delay_us(1);
            delay_us(1);				
            delay_us(1);
	  		U8temp=0;
		port_dht22->gpio_read(&data,0x100);
		 data =(data&0x00000100)>>8;
		 
	     if(data)
			U8temp=1;
					
		    U8FLAG=2;
		 port_dht22->gpio_read(&data,0x100);
		 data =(data&0x00000100)>>8;
		while( data &&   U8FLAG++)
		{
			
			port_dht22->gpio_read(&data,0x100);
		    data =(data&0x00000100)>>8;
		}
		//EMBARC_PRINTF("gpio_read is: %x\n",data);
	   	//超时则跳出for循环		  
	   	 if(U8FLAG==1)break;
	   	//判断数据位是0还是1	 
	   	   
		// 如果高电平高过预定0高电平值则数据位为 1 
	   	 
		   U8comdata<<=1;
	   	   U8comdata|=U8temp;        //0
	     }//rof
	   
	}

	//--------------------------------
	//-----湿度读取子程序 ------------
	//--------------------------------
	//----以下变量均为全局变量--------
	//----温度高8位== U8T_data_H------
	//----温度低8位== U8T_data_L------
	//----湿度高8位== U8RH_data_H-----
	//----湿度低8位== U8RH_data_L-----
	//----校验 8位 == U8checkdata-----
	//----调用相关子程序如下----------
	//---- Delay();, Delay_10us();,COM(); 
	//--------------------------------
	uint8_t dht22_temp;
	uint8_t dht22_wet;

	void dht22RH(void)
	{
	  //主机拉低18ms 
       uint32_t P2_0;
	   

	   
	   EMBARC_PRINTF("start dht22 rh !\n");
	  
	   dht22_out;
	   EMBARC_PRINTF("start dht22 rh !\n");
	   port_dht22->gpio_write(0x000,0x100);
	  // dw_gpio_write(port_dht22,~0x100,0x100);

	   delay_us(50);
	   port_dht22->gpio_write(0x100,0x100);
	  // dw_gpio_write(port_dht22,0x100,0x100);


	 //总线由上拉电阻拉高 主机延时20us
	   delay_us(4);


        
	 //主机设为输入 判断从机响应信号 
	
	  dht22_in;
	 //判断从机是否有低电平响应信号 如不响应则跳出，响应则向下运行
	 port_dht22->gpio_read(&P2_0,0x100);
	  P2_0=(P2_0&0x00000100)>>8;

	   if(!P2_0)		 //T !	  
	   {
	   U8FLAG=2;
	 //   EMBARC_PRINTF("if start\n");
	 //判断从机是否发出 80us 的低电平响应信号是否结束
		port_dht22->gpio_read(&P2_0,0x100);
	 	P2_0=(P2_0&0x00000100)>>8;
		while(!P2_0&&U8FLAG++)
		{
			port_dht22->gpio_read(&P2_0,0x100);
	 		P2_0=(P2_0&0x00000100)>>8;
			
		}
			
	   
	   U8FLAG=2;
	 //判断从机是否发出 80us 的高电平，如发出则进入数据接收状态
	 port_dht22->gpio_read(&P2_0,0x100);
	 P2_0=(P2_0&0x00000100)>>8;
	while(P2_0&&U8FLAG++)
		{
			 port_dht22->gpio_read(&P2_0,0x100);
			 P2_0=(P2_0&0x00000100)>>8;
		}
//	EMBARC_PRINTF("gpio_read is: %x\n",P2_0);
	 //数据接收状态		 
	   dht22COM();
	   U8RH_data_H_temp=U8comdata;
	   dht22COM();
	   U8RH_data_L_temp=U8comdata;
	   dht22COM();
	   U8T_data_H_temp=U8comdata;
	   dht22COM();
	   U8T_data_L_temp=U8comdata;
	   dht22COM();
	   U8checkdata_temp=U8comdata;
	   
	   dht22_out;
	 port_dht22->gpio_write(0x100,0x100);
	   //dw_gpio_write(port_dht22,0x100,0x100);
	  
	   //P2_0=1;
	 //数据校验 
   // EMBARC_PRINTF("wetness is :%d \n",U8checkdata_temp);
	   
	   U8temp=(U8T_data_H_temp+U8T_data_L_temp+U8RH_data_H_temp+U8RH_data_L_temp);
	   if(U8temp==U8checkdata_temp)
	   {
	   	  U8RH_data_H=U8RH_data_H_temp;
	   	  U8RH_data_L=U8RH_data_L_temp;
		  U8T_data_H=U8T_data_H_temp;
	   	  U8T_data_L=U8T_data_L_temp;
	   	  U8checkdata=U8checkdata_temp;
		  dht22_wet=(U8RH_data_H*256+U8RH_data_L)/(10)+40;
		  dht22_temp=(U8T_data_H*256+U8T_data_L)/(10)+40;
		 
		  
	   }//fi
	   }//fi
	   
	   

	}









//============================================================================================//
//==============ad有关函数===================================================================//
//===========================================================================================//
#define IIC_MASTER_ID		0
#define  PCF8591 0x90    //PCF8591 地址

#define SLVDEV_REG_channel_0  0x41
#define SLVDEV_REG_channel_1  0x42
#define SLVDEV_REG_channel_2  0x43
#define SLVDEV_REG_channel_3  0x40
#define PCF8591_SDA 0x800//porta[11]
#define PCF8591_SCL 0x400//porta[10]
#define PCF8591_SCL_SDA 0xc00
#define PCF8591_SDA_0 0x000
#define PCF8591_SCL_0 0x000
#define PCF8591_SDA_shift 11//porta[11]

static DEV_GPIO_PTR port_PCF8591;

void PCF8591_init()//初始化
{
	port_PCF8591 = gpio_get_dev(DW_GPIO_PORT_A);
	port_PCF8591->gpio_open(PCF8591_SCL_SDA);
	port_PCF8591->gpio_control(GPIO_CMD_SET_BIT_DIR_INPUT,(void *)PCF8591_SDA);//sda_poeta[10]
	port_PCF8591->gpio_control(GPIO_CMD_SET_BIT_DIR_OUTPUT,(void *)PCF8591_SCL);//scl_poeta[9]
}

#define PCF8591_SDA_IN  port_PCF8591->gpio_control(GPIO_CMD_SET_BIT_DIR_INPUT,(void *)PCF8591_SDA);//sda_poeta[10]
#define PCF8591_SDA_OUT port_PCF8591->gpio_control(GPIO_CMD_SET_BIT_DIR_OUTPUT,(void *)PCF8591_SDA);//sda_poeta[10]

void PCF8591_IIC_Start()//iic start
{
	PCF8591_SDA_OUT;
	 port_PCF8591->gpio_write(PCF8591_SCL_SDA,PCF8591_SCL_SDA);//sda=1,scl=1;
	 delay_us(4);
	 port_PCF8591->gpio_write(0x000,PCF8591_SDA);//sda=0;scl=1;
	 delay_us(4);
	 port_PCF8591->gpio_write(0x000,PCF8591_SCL);//sda=0,scl=0;
	 
}

void PCF8591_IIC_Stop()
{
	PCF8591_SDA_OUT;
	 port_PCF8591->gpio_write(0x000,PCF8591_SCL_SDA);//sda=0,scl=0;
	 delay_us(4);
	 port_PCF8591->gpio_write(PCF8591_SCL_SDA,PCF8591_SCL_SDA);//sda1;scl=1;
	 delay_us(4);
}

char PCF8591_IIC_Wait_ACK()//等待应答信号
{
	char ucErrTime=0;
	uint32_t PCF8591_data; 
	PCF8591_SDA_OUT;
	port_PCF8591->gpio_write(PCF8591_SDA,PCF8591_SDA);//sda=1,scl=0;
	delay_us(1);
	port_PCF8591->gpio_write(PCF8591_SCL,PCF8591_SCL);//sda=1,scl=1;
	delay_us(1);
	PCF8591_SDA_IN;
	port_PCF8591->gpio_read(&PCF8591_data,PCF8591_SDA);
    PCF8591_data =(PCF8591_data&PCF8591_SDA)>>PCF8591_SDA_shift;
	while(PCF8591_data)
		{
			port_PCF8591->gpio_read(&PCF8591_data,PCF8591_SDA);
    		PCF8591_data =(PCF8591_data&PCF8591_SDA)>>PCF8591_SDA_shift;
			ucErrTime++;
			if(ucErrTime>250)
				{
					PCF8591_IIC_Stop();
					return 1;
				}
		}
	port_PCF8591->gpio_write(0x000,PCF8591_SCL);//scl=0;
	return 0;
	
	
}


void PCF8591_IIC_ACK()//发送应答信号
{
	PCF8591_SDA_OUT;
	port_PCF8591->gpio_write(0x000,PCF8591_SCL_SDA);//sda=0,scl=0;
	delay_us(2);
	port_PCF8591->gpio_write(PCF8591_SCL,PCF8591_SCL);//sda=0,scl=1;
	delay_us(2);
	port_PCF8591->gpio_write(0x000,PCF8591_SCL);//sda=0,scl=0;
}


void PCF8591_IIC_NACK()//发送非应答信号
{
	PCF8591_SDA_OUT;
	port_PCF8591->gpio_write(0x000|PCF8591_SDA,PCF8591_SCL_SDA);//sda=1,scl=0;
	delay_us(2);
	port_PCF8591->gpio_write(PCF8591_SCL_SDA,PCF8591_SCL_SDA);//sda=1,scl=1;
	delay_us(2);
	port_PCF8591->gpio_write(0x000|PCF8591_SDA,PCF8591_SCL_SDA);//sda=1,scl=0;
}

void PCF8591_IIC_Send_Byte(unsigned char txd)//发送一byte数据
{
	unsigned char t;
	PCF8591_SDA_OUT;
	port_PCF8591->gpio_write(0x000,PCF8591_SCL);//scl=0
	for (t=0;t<8;t++)
		{
			if((txd&0x80)>>7)
				port_PCF8591->gpio_write(PCF8591_SDA,PCF8591_SDA);//sda=1;
			else
				port_PCF8591->gpio_write(0x000,PCF8591_SDA);//sda=0;	
			txd<<=1;
			delay_us(2);
			port_PCF8591->gpio_write(PCF8591_SCL,PCF8591_SCL);//scl=1;	
			delay_us(2);
			port_PCF8591->gpio_write(0x000,PCF8591_SCL);//scl=0;	
			delay_us(2);
			
		}
	
}


unsigned char  PCF8591_IIC_Read_Byte(unsigned char ack)//接收一byte数据
{
	unsigned char i,receive=0;
	uint32_t PCF8591_data; 
	PCF8591_SDA_IN;
	for (i=0;i<8;i++)
		{
			port_PCF8591->gpio_write(0x000,PCF8591_SCL);//scl=0;	
			delay_us(2);
			port_PCF8591->gpio_write(PCF8591_SCL,PCF8591_SCL);//scl=1;	
			receive<<=1;
			port_PCF8591->gpio_read(&PCF8591_data,PCF8591_SDA);
            PCF8591_data =(PCF8591_data&PCF8591_SDA)>>PCF8591_SDA_shift;
			if(PCF8591_data) 
				receive++;
			delay_us(1);
			
		}
	if(!ack)
		PCF8591_IIC_NACK();
	else
		PCF8591_IIC_ACK();
	return receive;
		
}

void PCF8591_write(unsigned char sla,unsigned char c)//ad的写命令
{
	PCF8591_IIC_Start();
	PCF8591_IIC_Send_Byte(sla);
	PCF8591_IIC_Wait_ACK();
	PCF8591_IIC_Send_Byte(c);
	PCF8591_IIC_Wait_ACK();
	PCF8591_IIC_Stop();
}

unsigned char PCF8591_read(unsigned char sla)//ad读寄存器
{
	unsigned char c;
	PCF8591_IIC_Start();
	
	PCF8591_IIC_Send_Byte(sla+1);
	PCF8591_IIC_Wait_ACK();
	
	c=PCF8591_IIC_Read_Byte(0);
		
	PCF8591_IIC_NACK();
	PCF8591_IIC_Stop();
	return c;
}


unsigned char  PCF8591_channel_0_3(int channel_i)//只读取其中四个模拟输入通道，不使用数字输出通道
{
	int i;
	unsigned char PCF8591_DATA_channel;

		switch(channel_i)
			{
				case 0:  PCF8591_write(PCF8591,SLVDEV_REG_channel_0);
						PCF8591_DATA_channel= PCF8591_read(PCF8591);
				
					    break;  
				
				case 1: PCF8591_write(PCF8591,SLVDEV_REG_channel_1);
						PCF8591_DATA_channel= PCF8591_read(PCF8591); //ADC1  模数转换2	  热敏电阻
					    
					    break;  
				
				case 2:PCF8591_write(PCF8591,SLVDEV_REG_channel_2);
						PCF8591_DATA_channel= PCF8591_read(PCF8591); //ADC2	模数转换3	   悬空
					
					 break;  
				
				case 3: PCF8591_write(PCF8591,SLVDEV_REG_channel_3);
						PCF8591_DATA_channel= PCF8591_read(PCF8591);//ADC3   模数转换4	   可调0-5v
					 	printf("");
					 break;  
			 }
	
	return PCF8591_DATA_channel;
}





















//=============================================================================================================================
//==============BH750==========================================================================================================
//=============================================================================================================================

#define IIC_MASTER_ID		0
#define  BH750_ADDRESS 0x46    //PCF8591 地址

#define BH750_SDA 0x20000//porta[17]
#define BH750_SCL 0x10000//porta[16]
#define BH750_SCL_SDA 0x30000
#define BH750_SDA_0 0x00000
#define BH750_SCL_0 0x00000
#define BH750_SDA_shift 17//porta[17]

static DEV_GPIO_PTR port_BH750;

void BH750_init()
{
	port_BH750 = gpio_get_dev(DW_GPIO_PORT_C);
	port_BH750->gpio_open(BH750_SCL_SDA);
	port_BH750->gpio_control(GPIO_CMD_SET_BIT_DIR_INPUT,(void *)BH750_SDA);//sda_poeta[10]
	port_BH750->gpio_control(GPIO_CMD_SET_BIT_DIR_OUTPUT,(void *)BH750_SCL);//scl_poeta[9]
}


#define BH750_SDA_IN  port_BH750->gpio_control(GPIO_CMD_SET_BIT_DIR_INPUT,(void *)BH750_SDA);//sda_poeta[10]
#define BH750_SDA_OUT port_BH750->gpio_control(GPIO_CMD_SET_BIT_DIR_OUTPUT,(void *)BH750_SDA);//sda_poeta[10]


void BH750_IIC_Start()//iic start
{
	BH750_SDA_OUT;
	 port_BH750->gpio_write(BH750_SCL_SDA,BH750_SCL_SDA);//sda=1,scl=1;
	 delay_us(5);
	 port_BH750->gpio_write(0x00000,BH750_SDA);//sda=0;scl=1;
	 delay_us(5);
	 port_BH750->gpio_write(0x00000,BH750_SCL);//sda=0,scl=0;
	 
}


void BH750_IIC_Stop()
{
	BH750_SDA_OUT;
	 port_BH750->gpio_write(0x00000|BH750_SCL,BH750_SCL_SDA);//sda=0,scl=0;
	 delay_us(5);
	 port_BH750->gpio_write(BH750_SDA,BH750_SDA);//sda1;scl=1;
	 delay_us(5);
}




char BH750_send_ACK(uint32_t ack)//等待应答信号
{
	BH750_SDA_OUT;
	port_BH750->gpio_write((BH750_SDA)&(ack<<BH750_SDA_shift),BH750_SDA);//sda=1,scl=0;
	port_BH750->gpio_write(BH750_SCL,BH750_SCL);//sda=1,scl=1;
	delay_us(5);
	port_BH750->gpio_write(0x00000,BH750_SCL);//;
	delay_us(5);
	
}



unsigned char  BH750_receive_ACK()//发送应答信号
{
	unsigned char tem;
	uint32_t BH750_data;
	BH750_SDA_IN;
	port_BH750->gpio_write(BH750_SCL,BH750_SCL);//sda=0,scl=0;
	delay_us(5);
	port_BH750->gpio_read(&BH750_data,BH750_SDA);
    BH750_data =(BH750_data&BH750_SDA)>>BH750_SDA_shift;
	tem=BH750_data;
	port_BH750->gpio_write(0x00000,BH750_SCL);//sda=0,scl=1;
	delay_us(5);
	return tem;
}



void BH750_Send_Byte(unsigned char txd)//发送一byte数据
{
	unsigned char t;
	BH750_SDA_OUT;
	for (t=0;t<8;t++)
		{
			if((txd&0x80)>>7)
				port_BH750->gpio_write(BH750_SDA,BH750_SDA);//sda=1;
			else
				port_BH750->gpio_write(0x00000,BH750_SDA);//sda=0;
			port_BH750->gpio_write(BH750_SCL,BH750_SCL);//scl=1;	
			txd<<=1;
			delay_us(5);
			port_BH750->gpio_write(0x00000,BH750_SCL);//scl=0;
			delay_us(5);
					
		}
	BH750_receive_ACK();
}


unsigned char  BH750_Read_Byte()//接收一byte数据
{
	unsigned char i,receive=0;
	uint32_t BH750_data; 
	BH750_SDA_OUT;
	port_BH750->gpio_write(BH750_SDA,BH750_SDA);//sda=1
	BH750_SDA_IN;
	for (i=0;i<8;i++)
		{
			port_BH750->gpio_write(BH750_SCL,BH750_SCL);//scl=1;	
			receive<<=1;
			delay_us(5);
			port_BH750->gpio_read(&BH750_data,BH750_SDA);
            BH750_data =(BH750_data&BH750_SDA)>>BH750_SDA_shift;
			if(BH750_data) 
				receive++;

			port_BH750->gpio_write(0x00000,BH750_SCL);//scl=1;
			delay_us(5);
			
		}
	return receive;
		
}



void BH750_write(unsigned char REG_Address)
{
    BH750_IIC_Start();                  
    BH750_Send_Byte(BH750_ADDRESS);   
    BH750_Send_Byte(REG_Address);         
    BH750_IIC_Stop();                   
}

unsigned char BH750_BUF[8];
void BH750_read(void)
{   unsigned char i;	
    BH750_IIC_Start();                          
    BH750_Send_Byte(BH750_ADDRESS+1);         
	
	 for (i=0; i<3; i++)                      
    {
        BH750_BUF[i] = BH750_Read_Byte();         
        if (i == 3)
        {

           BH750_send_ACK(1);                
        }
        else
        {		
          BH750_send_ACK(0);               
       }
   }

    BH750_IIC_Stop();                          
    delay_us(5000);
}

void BH750_reg_init()
{
   BH750_write(0x01);  
   BH750_write(0x10); 
   delay_us(180000);
}

uint32_t light_data;
char light3;
char light2;
char light1;
char light0;

void BH750_light()
{
	int dis_data;
	float F_light_data;
	BH750_reg_init();
	BH750_read();
	dis_data=BH750_BUF[0];
    dis_data=(dis_data<<8)+BH750_BUF[1];
	light_data=(uint32_t)((float)dis_data/(1.2))*10;
	F_light_data=(float)dis_data/(1.2);
	printf("light is mmp %f\n",F_light_data);

	light3=(light_data/((256)*256*256))%256;
	light2=(light_data/(256*256))%(256);
	light1=light_data/256%(256);
	light0=light_data%(256);

}


//================wifi8266==========================================//
DEV_UART *uart8266_obj;
uint8_t wifibuf[200];
uint32_t wifilen;

void uart_init_wifi()
{
	uart8266_obj = uart_get_dev(0);
	uart8266_obj->uart_close();
	int32_t ercd = uart8266_obj->uart_open(UART_BAUDRATE_9600);
	if(ercd==E_OPNED)
		{
			uart8266_obj->uart_control(UART_CMD_SET_BAUD,(void *)UART_BAUDRATE_9600);
		}
	

}

uint32_t wifi_read(uint32_t len)
{
	uint32_t rd_avail;
	uart8266_obj->uart_control(UART_CMD_GET_RXAVAIL,(void *)(&rd_avail));
	len=(len>rd_avail)? rd_avail : len;
	uart8266_obj->uart_read((void *)wifibuf, len);
	return len;

	
}




void wifi_send(uint8_t *buf ,uint32_t len)
{
	uart8266_obj->uart_write(buf,len);
}


//============wifi协议相关内容============================================//
//=======================================================================//

uint8_t reg_value9;
uint8_t reg_value10;
uint8_t pac_order;


char *string_temp;

void power_on_buf()
{
	uint8_t i;
	wifibuf[0]=0xff;
	wifibuf[1]=0xff;
	wifibuf[2]=0x00;
	wifibuf[3]=0x6f;
	wifibuf[4]=0x02;
	wifibuf[5]=0x00;
	
	wifibuf[6]=0x00;
	wifibuf[7]=0x00;
	
	wifibuf[8]='0';
	wifibuf[9]='0';
	wifibuf[10]='0';
	wifibuf[11]='0';
	wifibuf[12]='0';
	wifibuf[13]='0';
	wifibuf[14]='0';
	wifibuf[15]='4';
	
	wifibuf[16]='0';
	wifibuf[17]='0';
	wifibuf[18]='0';
	wifibuf[19]='0';
	wifibuf[20]='0';
	wifibuf[21]='0';
	wifibuf[22]='0';
	wifibuf[23]='2';
	
	for(i=0;i<=15;i++)
		{
			wifibuf[24+i]='0';
		}
	string_temp = "e189be51e86d4dae8db1c7d4562eca1f" ;
	
	for (i=0;i<=31;i++)
		{
			wifibuf[40+i]=string_temp[i];
	
		}
	
	wifibuf[72]=0x00;
	wifibuf[73]=0x00;
	
	for (i=0;i<=7;i++)
		{
			wifibuf[74+i]=0x00;
	
		}
	
	string_temp="b80bafa10cf44a87b1feb70554d29fa7";
	for (i=0;i<=31;i++)
			{
			wifibuf[82+i]=string_temp[i];
	
			}
	
	wifibuf[114]=0x46;
	wifilen=115;

	
}

void wifi_ctr_mcu_buf()
{
				wifibuf[0]=0xff;
				wifibuf[1]=0xff;
				wifibuf[2]=0x00;
				wifibuf[3]=0x05;
				wifibuf[4]=0x04;
				wifibuf[5]=0x01;
				wifibuf[6]=0x00;
				wifibuf[7]=0x00;
				wifibuf[8]=0x06;
				wifilen=9;
}


void wifi_search_mcu_buf()
{
				uint8_t i;
				wifibuf[0]=0xff;
				wifibuf[1]=0xff;
				wifibuf[2]=0x00;
				wifibuf[3]=0x11;
				
				wifibuf[4]=0x04;
				wifibuf[5]=pac_order;
				wifibuf[6]=0x00;
				wifibuf[7]=0x00;
				wifibuf[8]=0x03;
				
				wifibuf[9]=reg_value9&reg_value10;
				wifibuf[10]=PCF8591_DATA_channlel[0];
				wifibuf[11]=dht22_wet;
				wifibuf[12]=dht22_temp;
				wifibuf[13]=PCF8591_DATA_channlel[2];
				wifibuf[14]=PCF8591_DATA_channlel[3];
				wifibuf[15]=PCF8591_DATA_channlel[1];
				wifibuf[16]=light3;
				wifibuf[17]=light2;
				wifibuf[18]=light1;
				wifibuf[19]=light0;
				wifibuf[20]=0x00;

				for(i=2;i<=19;i++)
					{
				wifibuf[20]=wifibuf[20]+wifibuf[i];
					}
				wifibuf[20]=wifibuf[20]%256;
				
				wifilen=21;
}


void heart_buf()
{
				wifibuf[0]=0xff;
				wifibuf[1]=0xff;
				wifibuf[2]=0x00;
				wifibuf[3]=0x05;
				
				wifibuf[4]=0x08;
				wifibuf[5]=0x03;
				wifibuf[6]=0x00;
				wifibuf[7]=0x00;
				wifibuf[8]=0x10;
				wifilen=9;
}



void mcu_report2wifi()
{
	uint8_t i;
				wifibuf[0]=0xff;
				wifibuf[1]=0xff;
				wifibuf[2]=0x00;
				wifibuf[3]=0x10;
				
				wifibuf[4]=0x05;
				wifibuf[5]=0x04;
				wifibuf[6]=0x00;
				wifibuf[7]=0x00;
				wifibuf[8]=0x04;

				
				wifibuf[9]=reg_value9&reg_value10;
				wifibuf[10]=PCF8591_DATA_channlel[0];
				wifibuf[11]=dht22_wet;
				wifibuf[12]=dht22_temp;
				wifibuf[13]=PCF8591_DATA_channlel[2];
				wifibuf[14]=PCF8591_DATA_channlel[3];
				wifibuf[15]=PCF8591_DATA_channlel[1];
				wifibuf[16]=light3;
				wifibuf[17]=light2;
				wifibuf[18]=light1;
				wifibuf[19]=light0;
				wifibuf[20]=0x00;

				for(i=2;i<=19;i++)
					{
				wifibuf[20]=wifibuf[20]+wifibuf[i];
					}
				wifibuf[20]=wifibuf[20]%256;
				wifilen=21;
				
				uart8266_obj->uart_write(wifibuf,wifilen);

				
				
}



//=====================================================================
//=====================继电器+LED======================================



static DEV_GPIO_PTR port_relay_water;
static DEV_GPIO_PTR port_relay_Nu;
static DEV_GPIO_PTR port_relay_LED;


static void relay_water_init()
{
	
	port_relay_water = gpio_get_dev(DW_GPIO_PORT_C);
	port_relay_water->gpio_open(0x100000);
    port_relay_water->gpio_control(GPIO_CMD_SET_BIT_DIR_OUTPUT,(void *)0x100000);
	port_relay_water->gpio_write(0x100000,0x100000);
	
}


static void Nu_water_init()
{
	
	port_relay_Nu = gpio_get_dev(DW_GPIO_PORT_C);
	port_relay_Nu->gpio_open(0x200000);
    port_relay_Nu->gpio_control(GPIO_CMD_SET_BIT_DIR_OUTPUT,(void *)0x200000);
	port_relay_Nu->gpio_write(0x200000,0x200000);
	
}


static void LED_water_init()
{
	
	port_relay_LED = gpio_get_dev(DW_GPIO_PORT_C);
	port_relay_LED->gpio_open(0x400000);
    port_relay_LED->gpio_control(GPIO_CMD_SET_BIT_DIR_OUTPUT,(void *)0x400000);
	port_relay_LED->gpio_write(0x400000,0x400000);
	
}


void wifi_ctr_LED_Relay()
{
	uint32_t temp;
	
		if(reg_value9&0x01)
			{
				temp=0x01&reg_value10;
				port_relay_water->gpio_write(~((0x100000)&(temp<<20)),0x100000);
			}
	if(reg_value9&0x02)
		{
		temp=0x02&reg_value10;
		port_relay_Nu->gpio_write(~((0x200000)&(temp<<20)),0x200000);
		}
	if(reg_value9&0x04)
		{
		temp=0x04&reg_value10;
		port_relay_LED->gpio_write(~((0x400000)&(temp<<20)),0x400000);
		}
	
}









//======================================================================













	
#define wifisend uart8266_obj->uart_write(wifibuf,wifilen)

uint32_t test_time;
uint8_t wifi_link()
{
	uint8_t crc=0;
	uint16_t package_len;
	uint16_t i;
	uint8_t status=0;
	while(1)
		{
		
			delay_us(50000);
				wifilen=wifi_read(150);
			if(wifibuf[0]==0xff)
				{
					if(wifibuf[1]==0xff)
						{

							switch(wifibuf[4])
								{
									case 0x01:
											printf("0x01\n");
											package_len=wifibuf[2]*256+wifibuf[3]+1;
									        crc=package_len%256;
									       if(wifibuf[8]==crc)
												{
												power_on_buf();
												uart8266_obj->uart_write(wifibuf,wifilen);
												}
										   break;
									 case 0x03:
									 		printf("0x03\n");
											 if(wifibuf[8]==0x01)
											{
														
														crc=0;
														for(i=2;i<=10;i++)
															{
																crc=crc+wifibuf[i];
															}
														crc=crc%256;
														if(wifibuf[11]==crc)
														 {

														 
														 printf("action is 0x01\n");
														reg_value9=wifibuf[9];
														reg_value10=wifibuf[10];
														printf("reg_calue is %x\n",reg_value9);
														printf("reg_calue is %x\n",reg_value10);
														wifi_ctr_mcu_buf(); 																						 
														uart8266_obj->uart_write(wifibuf,wifilen);
																							 
														 }
												}
											 else if(wifibuf[8]==0x02)
											 	{
											 	
											 		crc=0;
											 		for(i=2;i<=8;i++)
															{
																crc=crc+wifibuf[i];
															}
														crc=crc%256;
																if(wifibuf[9]==crc)
																 {
																 	pac_order=wifibuf[5];
																	printf("action 0x02\n");
																wifi_search_mcu_buf(); 
																	for(i=0;i<21;i++)
																			{
																				printf("search is %x\n",wifibuf[i]);
																			}
																	uart8266_obj->uart_write(wifibuf,wifilen);
																									 
																 }

											
									 				}
									 break;
								
									case 0x07:
										printf("0x07\n");

													crc=0;
														for(i=2;i<=7;i++)
															{
																crc=crc+wifibuf[i];
															}
														crc=crc%256;
									       if(wifibuf[8]==crc)
												{
												heart_buf();
												uart8266_obj->uart_write(wifibuf,wifilen);
												}
										   break;
									 
							    }
	
						}
				}



	wifi_ctr_LED_Relay();
		if(test_time>=30)
			{
				PCF8591_DATA_channlel[0]=(char)((float)(PCF8591_channel_0_3(0))/255.0*100.0);
				PCF8591_DATA_channlel[1]=(char)((float)(PCF8591_channel_0_3(1))/255.0*100.0);
				PCF8591_DATA_channlel[2]=(char)((float)(PCF8591_channel_0_3(2))/255.0*30.0);
				PCF8591_DATA_channlel[3]=(char)((float)(PCF8591_channel_0_3(3))/255.0*30.0);
				BH750_light();
				dht22RH();
				test_time=0;
				mcu_report2wifi();
			}

	
	}
}







//====================================================================//
//lcd 初始化程序




u8g_t u8g;

void u8g_prepare(void) {
	u8g_SetFont(&u8g, u8g_font_unifont);		/* set the current font and reset the font reference position to "Baseline" */
	u8g_SetFontRefHeightExtendedText(&u8g);		/* define the calculation method for the ascent and descent of the current font */
	u8g_SetDefaultForegroundColor(&u8g);		/* assign one of the default colors as current color index */
	u8g_SetFontPosTop(&u8g);			/* set the reference position for the character and string draw procedure */
}

int y_pos = 0;

void draw(void) {
	u8g_DrawStr(&u8g, 0, y_pos, "mm");
}






static void timer1_isr(void *ptr);

static void time1_init();





//======================================================================//

/** main entry */
int main(void)
{

	int i;
	uint8_t wifiregvalue;


		
	cpu_lock(); /* lock cpu to do initializations */

	board_init(); /* board level init */

	cpu_unlock(); /* unlock cpu to let inetrrupt work */

	

		


//===============================================
	PCF8591_init(); /* IIC device initialization */
	dht22_init();
	BH750_init();
	BH750_write(0x01); 
//==============================================================
	time1_init();




///=========================================================

	uart_init_wifi();

//===============================================================

relay_water_init();
Nu_water_init();
LED_water_init();


				PCF8591_DATA_channlel[0]=(char)((float)(PCF8591_channel_0_3(0))/255.0*100.0);
				PCF8591_DATA_channlel[1]=(char)((float)(PCF8591_channel_0_3(1))/255.0*100.0);
				PCF8591_DATA_channlel[2]=(char)((float)(PCF8591_channel_0_3(2))/255.0*30.0);
				PCF8591_DATA_channlel[3]=(char)((float)(PCF8591_channel_0_3(3))/255.0*30.0);
				BH750_light();
				dht22RH();



//=======================================================================
	while (1) {
	


				wifi_link();

				//port_relay_water->gpio_write(0x000000,0x100000);

				
				//delay_us(50000);


	}


	return E_SYS;
}

/** @} */






//====================定时器1===============================================================

uint16_t cnt1;

static void timer1_isr(void *ptr)
{
	timer_int_clear(TIMER_1);
	cnt1++;
	test_time++;
/*f(cnt1>=2)
		{

				
		  printf("channel_0:%d channel_1:%d channel_2:%d channel_3:%d \n",(PCF8591_DATA_channlel[0]),(PCF8591_DATA_channlel[1]),(PCF8591_DATA_channlel[2]),(PCF8591_DATA_channlel[3]));

		  printf("light is: %d \n",light_data);

	      printf("3 is %x,2 is %x,1 is %x, 0 is %x\n",light3,light2,light1,light0);	
		  printf("wetness is :%f \n",dht22_wet);
		  printf("temp is :%f\n",dht22_temp);
		  printf("mmp\n");
			cnt1=0;
		}
	*/
	//EMBARC_PRINTF("timer0 interrupt:%d\n", cnt0);
}

static void time1_init()
{
	int_disable(INTNO_TIMER1);
	timer_stop(INTNO_TIMER1);
	int_handler_install(INTNO_TIMER1, timer1_isr);//关联中断服务子程序
	int_enable(INTNO_TIMER1);
	timer_start(TIMER_1, TIMER_CTRL_IE,BOARD_CPU_CLOCK);
	
		
}	












//=========================================================================================
/** main entry 
int main(void)
{
	 //Common Init Process Start 
    uint32_t pc8;
	int  *ad_num;
	ADT7420 *tempsensor = &adt7420_sensor;
	uint32_t tempvalue;
	uint32_t next_step = 0;
	
	cpu_lock();
	board_init(); /* board initial */
	//cpu_unlock();
 // u8g_InitComFn(&u8g, &u8g_dev_ssd1306_128x64_2x_i2c, U8G_COM_SSD_I2C); /* create a new interface to a graphics display */

	//EMBARC_PRINTF("u8glib\r\n");

	//EMBARC_PRINTF("Display Width: %u, Display Height: %u\r\n" , u8g_GetWidth(&u8g), u8g_GetHeight(&u8g));

	//u8g_Begin(&u8g); /* reset display and put it into default state */
	//u8g_prepare();

	//====================初始化==========================================//
	//====================================================================//
//	ad_slvdev_init(PCF8591);
	//dht22_init();


	
	//==============================================================================================================================
	//=========定时器相关===========================================================================================================

	//}



/** @} */
