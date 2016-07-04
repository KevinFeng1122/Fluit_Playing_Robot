/*

----------【硬件配置说明】↓--------------------------------------------------
//UART0:[MCU - SPEAKING MODULE]BAUD 9600, DC 5V，RXD0-PE0, TXD0-PE1
//UART1:[MCU - MOTOR BOARD] BAUD 9600, DC 5V, RXD1-PD2, TXD1-PD3


----------【单片机管脚连接说明说明】↓--------------------------------------------------
语音模块(UART0)：TXD-E0(MCU.RXD0)
舵机控制板(UART1):RXD-D3(MCU.TXD1)
继电器模块:IN1-A1→电磁换向阀, IN2-A2→电气比例阀
*/

//----------【头文件】↓--------------------------------------------------------
#include <iom128v.h>
#include <macros.h>
#include <string.h>

//----------【宏定义】↓--------------------------------------------------------
#define  uchar unsigned char
#define  uint  unsigned int
#define  mclk   8000000 //时钟频率8.0MHz


//----------【全局变量定义】↓--------------------------------------------------
uchar uart0_received_byte;//UART0每次接收到的单字节信息
uchar uart0_received_flag=0;

uchar Note_end=0X0D;//0X0d means <cr> 
uchar Note_test1[]="#0 P750 T100";
uchar Note_test2[]="#0 P1450 T100";
uchar Note_test3[]={0X23, 0X30, 0X20, 0X50, 0X31, 0X30, 0X30, 0X30, 0X20, 0X54, 0X31, 0X30, 0X30, 0X0d};
uchar Note_test4[]={0X23, 0X30, 0X20, 0X50, 0X32, 0X30, 0X30, 0X30, 0X20, 0X54, 0X31, 0X30, 0X30, 0X0d};

//---------------------------【乐曲保存区域】↓---------------------------------

//康定情歌
float KangDingQingGe[100][2]={
{3,0.5},{5,0.5},{6,0.5},{6,0.25},{5,0.5},{6,0.5},{3,0.5},{2,1},
{3,0.5},{5,0.5},{6,0.5},{6,0.25},{5,0.25},{6,0.5},{3,0.5},{3,1},
{3,0.5},{5,0.5},{6,0.5},{6,0.25},{5,0.25},{6,0.5},{3,0.5},{2,1},
{5,0.5},{3,0.5},{2,0.25},{3,0.25},{2,0.25},{1,0.25},{2,0.5},{26,0.5},{26,1},
{26,0.5},{2,0.5},{2,1},{5,0.5},{3,0.5},{3,1},{2,0.25},{1,0.25},{26,0.5},{26,1},
{5,0.5},{3,0.5},{2,0.25},{3,0.25},{2,0.25},{1,0.25},{2,0.5},{26,0.5},{26,1},
{5,0.5},{3,0.5},{2,0.25},{3,0.25},{2,0.25},{1,0.25},{2,0.5},{6,0.5},{6,1},
{30,30}};

//月光下的凤尾竹
float YueGuangXiaDeFengWeiZhu[100][2]={
{1,0.5},{26,0.5},{26,0.5},{1,0.5},{1,1},
{1,0.5},{2,0.5},{2,0.5},{3,0.5},{3,1},
{3,0.5},{2,0.5},{2,0.5},{1,0.5},{1,0.5},{26,0.5},
{1,2},{2,0.5},{1,0.5},{26,2},
{1,0.5},{1,0.5},{2,0.5},{3,0.5},{3,1},
{3,0.5},{1,0.5},{2,0.5},{3,0.5},{3,1},
{5,0.5},{1,0.5},{2,0.5},{3,0.5},{3,0.5},{3,0.5},
{3,0.5},{26,0.5},{1,0.5},{2,0.5},{2,1},
{3,1},{26,1},
{1,0.5},{1,0.5},{2,0.5},{3,0.5},{3,0.75},{3,0.25},
{5,0.5},{1,0.5},{2,0.5},{3,0.5},{3,1},
{5,0.5},{3,0.5},{5,0.5},{6,0.5},{6,0.75},{6,0.25},
{5,1},{1,0.5},{3,0.5},{2,0.5},{1,0.5},{1,3},
{30,30}};


//傣族民歌
/*
float DaiZuMinGe[100][2]={
{1,0.5},{2,0.5},{5,1},{3,2},{3,1.5},{5,0.5},{5,1},{3,0.5},{5,0.5},
{1,1},{1,0.5},{2,0.5},{1,0.5},{26,0.5},{1,0.5},{2,0.5},{1,1},{3,0.5},{5,0.5},{1,0.5},{2,0.5},{3,0.5},{5,0.5},
{,},{,},{,},{,},{,},{,},{,},{,},{,},{,},{,},{,},{,},{,},{,},{,},
}*/
//太阳



//---------------------------【音符指法指令保存区域】↓---------------------------------
//从低音5到中音6
char N25[]="#4 P1554 #8 P1384 #12 P1900 #16 P1213 #20 P1508 #24 P1415 #28 P1630 T100";//72字符
char N26[]="#4 P1089 #8 P1384 #12 P1900 #16 P1213 #20 P1508 #24 P1415 #28 P1630 T100";//72字符
char N27[]="#4 P1089 #8 P857 #12 P1900 #16 P1213 #20 P1508 #24 P1415 #28 P1630 T100";//71字符
 char N1[]="#4 P1089 #8 P857 #12 P1523 #16 P1213 #20 P1508 #24 P1415 #28 P1630 T100";//71字符
 char N2[]="#4 P1089 #8 P857 #12 P1523 #16 P888 #20 P1508 #24 P1415 #28 P1630 T100";//70字符
 char N3[]="#4 P1089 #8 P857 #12 P1523 #16 P888 #20 P1058 #24 P1415 #28 P1630 T100";//70字符
 char N4[]="#4 P1554 #8 P1384 #12 P1900 #16 P1213 #20 P1508 #24 P1116 #28 P1630 T100";//72字符
 char N5[]="#4 P1089 #8 P857 #12 P1523 #16 P888 #20 P1058 #24 P996 #28 P1630 T100";//69字符
 char N6[]="#4 P1089 #8 P857 #12 P1523 #16 P888 #20 P1058 #24 P996 #28 P1833 T100";//69字符
//演奏结束后抬起手指
char All_Rise[]="#4 P1089 #8 P857 #12 P1523 #16 P888 #20 P1058 #24 P996 #28 P1833 T100";//69字符=N6[]


//--------------【软件延时函数】------------------------------------------------

//延时函数，参数为要延时的毫秒数
void delay(uint ms)
{
    uint i,j;
	for(i=0;i<ms;i++)
	{
	 for(j=0;j<2000;j++);
    }
}


//----------【普通IO口相关函数】↓---------------------------------------------------------------

/*IO口初始化函数*/
void I0_init(void)
{
    //PA1和PA2设置为输出
	DDRA|=BIT(1);
	DDRA|=BIT(2);
	//开始时为低电平
	PORTA&=(~BIT(1));
	PORTA&=(~BIT(2));
}


//----------【UART0的相关函数】↓---------------------------------------------------------------

/*UART0的串口初始化函数*/
void uart0_init(uint baud)
{
   UCSR0B=0x00; 
   UCSR0A=0x00; 		   //控制寄存器清零
   UCSR0C=(0<<UPM00)|(3<<UCSZ00); //选择UCSRC，异步模式，禁止校验，1位停止位，8位数据位                       
   
   baud=mclk/16/baud-1;    //波特率最大为65K
   UBRR0L=baud; 					     	  
   UBRR0H=baud>>8; 		   //设置波特率

   
   UCSR0B|=(1<<RXEN0);
   //UCSR0B|=(1<<TXEN0);   //UART0发送使能
   UCSR0B|=BIT(RXCIE0);
   SREG=BIT(7);	           //全局中断开放
   //DDRE|=BIT(1);	           //配置TX为输出（很重要）
}

/*UART0的串口发送函数，每次发送一个字节（Byte）*//*
void UART0_Send_Byte(uchar data)
{
   while(!(UCSR0A&(BIT(UDRE0))));//判断准备就绪否
   UDR0=data;
   while(!(UCSR0A&(BIT(TXC0))));//判断完成发送否
   UCSR0A|=BIT(TXC0);//TXC0标志位手动清零，通过将TXC0置1实现
}*/

/*UART0字符串发送函数*//*
void UART0_Send_String(uchar *str_send,uchar str_num)//形参：待发送字符串
{
	 uchar i=0;
	 while(i<str_num)
	 {
	   UART0_Send_Byte(*(str_send+i));
	   i+=1;
	 }
}*/
#pragma interrupt_handler UART0_Receive_Byte:19

/*UART0的串口接收函数，每次接收一个字节（Byte）*/
void UART0_Receive_Byte(void)
{	
	UCSR0B&=~BIT(RXCIE0);//关闭RXCIE0，其余位保持不变
	uart0_received_byte=UDR0;
	uart0_received_flag=1;
}


//------------【UART1的相关函数】↓-------------------------------------------------------------

/*UART1的串口初始化函数*/
void uart1_init(uint baud)
{
    UCSR1B=0x00; 
    UCSR1A=0x00; 		   //控制寄存器清零
    UCSR1C=(0<<UPM10)|(3<<UCSZ10); //选择UCSRC，异步模式，禁止校验，1位停止位，8位数据位                       
   
    baud=mclk/16/baud-1;    //波特率最大为65K
    UBRR1L=baud; 					     	  
    UBRR1H=baud>>8; 		   //设置波特率
   
    UCSR1B|=(1<<TXEN1)|(1<<RXCIE1);   //接收、发送使能，接收中断使能 |(1<<RXEN1)
    SREG=BIT(7);	           //全局中断开放
    DDRD|=BIT(3);	           //配置TX为输出（很重要）
}


/*UART1的串口发送函数，每次发送一个字节（Byte）*/
void UART1_SendB(uchar data)
{
   while(!(UCSR1A&(BIT(UDRE1))));//判断准备就绪否
   UDR1=data;
   while(!(UCSR1A&(BIT(TXC1))));//判断完成发送否
   UCSR1A|=BIT(TXC1);//TXC1标志位手动清零，通过将TXC1置1实现
}

/*UART1字符串发送函数*/
void UART1_Send_String(uchar *str_send,uchar str_num)//形参：待发送字符串
{
	 uchar i=0;
	 while(i<str_num)
	 {
	     if(*(str_send+i)!=0)
	   	 {
	         UART1_SendB(*(str_send+i));
	   	 }
	   
	   i+=1;
	 }
}

//---------------------------【乐曲演奏相关函数】↓-----------------------------------------------------------
void Play_Music(float Music[100][2])
{
	char Note_Index=0;//指向要演奏的音符
	float Note;
	float Time;
	
	//电气比例阀打开
	
	//电磁换向阀打开
	Direction_Valve(1);
	delay(300);
	
	//播放乐曲
	while(Music[Note_Index][0]!=30)//30表示乐曲结束
	{
		Note=Music[Note_Index][0];
		Time=Music[Note_Index][1]*660;
		Play_Note(Note);
		delay(Time);
		Note_Index++;
	}
	
	//电磁换向阀关闭
	Direction_Valve(0);
	//电气比例阀关闭
	
	//所有手指抬起
	UART1_Send_String(All_Rise,69);
	UART1_SendB(Note_end);
}

void Play_Note(float Note)
{
    switch((char)Note)
	{
	    case 1://中音1
		{    
		    UART1_Send_String(N1,71);//UART舵机动作指令
			UART1_SendB(Note_end);
			break;
		}
		case 2://中音2
		{
		    UART1_Send_String(N2,70);
			UART1_SendB(Note_end);
			break;
		}
		case 3://中音3
		{
		    UART1_Send_String(N3,70);
			UART1_SendB(Note_end);
			break;
		}
		case 4://中音4
		{
		    UART1_Send_String(N4,72);
			UART1_SendB(Note_end);
			break;
		}
		case 5://中音5
		{
		    UART1_Send_String(N5,69);
			UART1_SendB(Note_end);
			break;
		}
		case 6://中音6
		{
		    UART1_Send_String(N6,69);
			UART1_SendB(Note_end);
			break;
		}
		case 25://低音5
		{
		    UART1_Send_String(N25,72);
			UART1_SendB(Note_end);
			break;
		}
		case 26://低音6
		{
		    UART1_Send_String(N26,72);
			UART1_SendB(Note_end);
			break;
		}
		case 27://低音7
		{
		    UART1_Send_String(N27,71);
			UART1_SendB(Note_end);
			break;
		}									
	}    
}


//---------------------------【继电器模块控制相关函数】↓----------------------------------------------------

void Numeric_Valve(uchar state)//电气比例阀
{
    if(state)
	{
	    //打开电气比例阀
		PORTA|=BIT(2);//PA2=1
	}
	else
	{
	    //关闭电气比例阀
		PORTA&=(~BIT(2));//PA2=0
	}
}

void Direction_Valve(uchar state)//换向阀
{
    if(state==1)
	{
	    //气流通向乐器
		PORTA|=BIT(1);//PA1=1
		UART1_Send_String("#0H",3);
		UART1_SendB(Note_end);
	}
	else
	{
	    //气流排入大气
		PORTA&=(~BIT(1));//PA1=0
		UART1_Send_String("#0L",3);
		UART1_SendB(Note_end);
	}
}


//---------------------------【主函数】↓-----------------------------------------------------------
void main(void)
{
    //初始化
	uart0_init(9600);//speaking module
	uart1_init(9600);//motor board
	
	//大循环
	while(1)
	{	    
	    if(uart0_received_flag==1)
		{	
			switch(uart0_received_byte)
			{
	    	    case 0x01://康定情歌
				{
		            delay(3000);
					Play_Music(KangDingQingGe);
					break;
				}
			    case 0x02://月光下的凤尾竹
		    	{
		            delay(3500);
					Play_Music(YueGuangXiaDeFengWeiZhu);
					break;
			    }
			 	case 0x03://傣族民歌
		    	{
			        UART1_Send_String(Note_test3,14);
			    	delay(50);
					break;
			    }
				case 0x04://太阳
		    	{
			        UART1_Send_String(Note_test4,14);
			    	delay(50);
					break;
			    }
		    }
			uart0_received_byte=0;
			uart0_received_flag=0;
			UCSR0B|=BIT(RXCIE0);//使能RXCIE0，其余位保持不变
        }
	}
}
