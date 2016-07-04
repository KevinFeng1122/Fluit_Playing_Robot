/*

----------��Ӳ������˵������--------------------------------------------------
//UART0:[MCU - SPEAKING MODULE]BAUD 9600, DC 5V��RXD0-PE0, TXD0-PE1
//UART1:[MCU - MOTOR BOARD] BAUD 9600, DC 5V, RXD1-PD2, TXD1-PD3


----------����Ƭ���ܽ�����˵��˵������--------------------------------------------------
����ģ��(UART0)��TXD-E0(MCU.RXD0)
������ư�(UART1):RXD-D3(MCU.TXD1)
�̵���ģ��:IN1-A1����Ż���, IN2-A2������������
*/

//----------��ͷ�ļ�����--------------------------------------------------------
#include <iom128v.h>
#include <macros.h>
#include <string.h>

//----------���궨�塿��--------------------------------------------------------
#define  uchar unsigned char
#define  uint  unsigned int
#define  mclk   8000000 //ʱ��Ƶ��8.0MHz


//----------��ȫ�ֱ������塿��--------------------------------------------------
uchar uart0_received_byte;//UART0ÿ�ν��յ��ĵ��ֽ���Ϣ
uchar uart0_received_flag=0;

uchar Note_end=0X0D;//0X0d means <cr> 
uchar Note_test1[]="#0 P750 T100";
uchar Note_test2[]="#0 P1450 T100";
uchar Note_test3[]={0X23, 0X30, 0X20, 0X50, 0X31, 0X30, 0X30, 0X30, 0X20, 0X54, 0X31, 0X30, 0X30, 0X0d};
uchar Note_test4[]={0X23, 0X30, 0X20, 0X50, 0X32, 0X30, 0X30, 0X30, 0X20, 0X54, 0X31, 0X30, 0X30, 0X0d};

//---------------------------�������������򡿡�---------------------------------

//�������
float KangDingQingGe[100][2]={
{3,0.5},{5,0.5},{6,0.5},{6,0.25},{5,0.5},{6,0.5},{3,0.5},{2,1},
{3,0.5},{5,0.5},{6,0.5},{6,0.25},{5,0.25},{6,0.5},{3,0.5},{3,1},
{3,0.5},{5,0.5},{6,0.5},{6,0.25},{5,0.25},{6,0.5},{3,0.5},{2,1},
{5,0.5},{3,0.5},{2,0.25},{3,0.25},{2,0.25},{1,0.25},{2,0.5},{26,0.5},{26,1},
{26,0.5},{2,0.5},{2,1},{5,0.5},{3,0.5},{3,1},{2,0.25},{1,0.25},{26,0.5},{26,1},
{5,0.5},{3,0.5},{2,0.25},{3,0.25},{2,0.25},{1,0.25},{2,0.5},{26,0.5},{26,1},
{5,0.5},{3,0.5},{2,0.25},{3,0.25},{2,0.25},{1,0.25},{2,0.5},{6,0.5},{6,1},
{30,30}};

//�¹��µķ�β��
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


//�������
/*
float DaiZuMinGe[100][2]={
{1,0.5},{2,0.5},{5,1},{3,2},{3,1.5},{5,0.5},{5,1},{3,0.5},{5,0.5},
{1,1},{1,0.5},{2,0.5},{1,0.5},{26,0.5},{1,0.5},{2,0.5},{1,1},{3,0.5},{5,0.5},{1,0.5},{2,0.5},{3,0.5},{5,0.5},
{,},{,},{,},{,},{,},{,},{,},{,},{,},{,},{,},{,},{,},{,},{,},{,},
}*/
//̫��



//---------------------------������ָ��ָ������򡿡�---------------------------------
//�ӵ���5������6
char N25[]="#4 P1554 #8 P1384 #12 P1900 #16 P1213 #20 P1508 #24 P1415 #28 P1630 T100";//72�ַ�
char N26[]="#4 P1089 #8 P1384 #12 P1900 #16 P1213 #20 P1508 #24 P1415 #28 P1630 T100";//72�ַ�
char N27[]="#4 P1089 #8 P857 #12 P1900 #16 P1213 #20 P1508 #24 P1415 #28 P1630 T100";//71�ַ�
 char N1[]="#4 P1089 #8 P857 #12 P1523 #16 P1213 #20 P1508 #24 P1415 #28 P1630 T100";//71�ַ�
 char N2[]="#4 P1089 #8 P857 #12 P1523 #16 P888 #20 P1508 #24 P1415 #28 P1630 T100";//70�ַ�
 char N3[]="#4 P1089 #8 P857 #12 P1523 #16 P888 #20 P1058 #24 P1415 #28 P1630 T100";//70�ַ�
 char N4[]="#4 P1554 #8 P1384 #12 P1900 #16 P1213 #20 P1508 #24 P1116 #28 P1630 T100";//72�ַ�
 char N5[]="#4 P1089 #8 P857 #12 P1523 #16 P888 #20 P1058 #24 P996 #28 P1630 T100";//69�ַ�
 char N6[]="#4 P1089 #8 P857 #12 P1523 #16 P888 #20 P1058 #24 P996 #28 P1833 T100";//69�ַ�
//���������̧����ָ
char All_Rise[]="#4 P1089 #8 P857 #12 P1523 #16 P888 #20 P1058 #24 P996 #28 P1833 T100";//69�ַ�=N6[]


//--------------�������ʱ������------------------------------------------------

//��ʱ����������ΪҪ��ʱ�ĺ�����
void delay(uint ms)
{
    uint i,j;
	for(i=0;i<ms;i++)
	{
	 for(j=0;j<2000;j++);
    }
}


//----------����ͨIO����غ�������---------------------------------------------------------------

/*IO�ڳ�ʼ������*/
void I0_init(void)
{
    //PA1��PA2����Ϊ���
	DDRA|=BIT(1);
	DDRA|=BIT(2);
	//��ʼʱΪ�͵�ƽ
	PORTA&=(~BIT(1));
	PORTA&=(~BIT(2));
}


//----------��UART0����غ�������---------------------------------------------------------------

/*UART0�Ĵ��ڳ�ʼ������*/
void uart0_init(uint baud)
{
   UCSR0B=0x00; 
   UCSR0A=0x00; 		   //���ƼĴ�������
   UCSR0C=(0<<UPM00)|(3<<UCSZ00); //ѡ��UCSRC���첽ģʽ����ֹУ�飬1λֹͣλ��8λ����λ                       
   
   baud=mclk/16/baud-1;    //���������Ϊ65K
   UBRR0L=baud; 					     	  
   UBRR0H=baud>>8; 		   //���ò�����

   
   UCSR0B|=(1<<RXEN0);
   //UCSR0B|=(1<<TXEN0);   //UART0����ʹ��
   UCSR0B|=BIT(RXCIE0);
   SREG=BIT(7);	           //ȫ���жϿ���
   //DDRE|=BIT(1);	           //����TXΪ���������Ҫ��
}

/*UART0�Ĵ��ڷ��ͺ�����ÿ�η���һ���ֽڣ�Byte��*//*
void UART0_Send_Byte(uchar data)
{
   while(!(UCSR0A&(BIT(UDRE0))));//�ж�׼��������
   UDR0=data;
   while(!(UCSR0A&(BIT(TXC0))));//�ж���ɷ��ͷ�
   UCSR0A|=BIT(TXC0);//TXC0��־λ�ֶ����㣬ͨ����TXC0��1ʵ��
}*/

/*UART0�ַ������ͺ���*//*
void UART0_Send_String(uchar *str_send,uchar str_num)//�βΣ��������ַ���
{
	 uchar i=0;
	 while(i<str_num)
	 {
	   UART0_Send_Byte(*(str_send+i));
	   i+=1;
	 }
}*/
#pragma interrupt_handler UART0_Receive_Byte:19

/*UART0�Ĵ��ڽ��պ�����ÿ�ν���һ���ֽڣ�Byte��*/
void UART0_Receive_Byte(void)
{	
	UCSR0B&=~BIT(RXCIE0);//�ر�RXCIE0������λ���ֲ���
	uart0_received_byte=UDR0;
	uart0_received_flag=1;
}


//------------��UART1����غ�������-------------------------------------------------------------

/*UART1�Ĵ��ڳ�ʼ������*/
void uart1_init(uint baud)
{
    UCSR1B=0x00; 
    UCSR1A=0x00; 		   //���ƼĴ�������
    UCSR1C=(0<<UPM10)|(3<<UCSZ10); //ѡ��UCSRC���첽ģʽ����ֹУ�飬1λֹͣλ��8λ����λ                       
   
    baud=mclk/16/baud-1;    //���������Ϊ65K
    UBRR1L=baud; 					     	  
    UBRR1H=baud>>8; 		   //���ò�����
   
    UCSR1B|=(1<<TXEN1)|(1<<RXCIE1);   //���ա�����ʹ�ܣ������ж�ʹ�� |(1<<RXEN1)
    SREG=BIT(7);	           //ȫ���жϿ���
    DDRD|=BIT(3);	           //����TXΪ���������Ҫ��
}


/*UART1�Ĵ��ڷ��ͺ�����ÿ�η���һ���ֽڣ�Byte��*/
void UART1_SendB(uchar data)
{
   while(!(UCSR1A&(BIT(UDRE1))));//�ж�׼��������
   UDR1=data;
   while(!(UCSR1A&(BIT(TXC1))));//�ж���ɷ��ͷ�
   UCSR1A|=BIT(TXC1);//TXC1��־λ�ֶ����㣬ͨ����TXC1��1ʵ��
}

/*UART1�ַ������ͺ���*/
void UART1_Send_String(uchar *str_send,uchar str_num)//�βΣ��������ַ���
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

//---------------------------������������غ�������-----------------------------------------------------------
void Play_Music(float Music[100][2])
{
	char Note_Index=0;//ָ��Ҫ���������
	float Note;
	float Time;
	
	//������������
	
	//��Ż��򷧴�
	Direction_Valve(1);
	delay(300);
	
	//��������
	while(Music[Note_Index][0]!=30)//30��ʾ��������
	{
		Note=Music[Note_Index][0];
		Time=Music[Note_Index][1]*660;
		Play_Note(Note);
		delay(Time);
		Note_Index++;
	}
	
	//��Ż��򷧹ر�
	Direction_Valve(0);
	//�����������ر�
	
	//������ָ̧��
	UART1_Send_String(All_Rise,69);
	UART1_SendB(Note_end);
}

void Play_Note(float Note)
{
    switch((char)Note)
	{
	    case 1://����1
		{    
		    UART1_Send_String(N1,71);//UART�������ָ��
			UART1_SendB(Note_end);
			break;
		}
		case 2://����2
		{
		    UART1_Send_String(N2,70);
			UART1_SendB(Note_end);
			break;
		}
		case 3://����3
		{
		    UART1_Send_String(N3,70);
			UART1_SendB(Note_end);
			break;
		}
		case 4://����4
		{
		    UART1_Send_String(N4,72);
			UART1_SendB(Note_end);
			break;
		}
		case 5://����5
		{
		    UART1_Send_String(N5,69);
			UART1_SendB(Note_end);
			break;
		}
		case 6://����6
		{
		    UART1_Send_String(N6,69);
			UART1_SendB(Note_end);
			break;
		}
		case 25://����5
		{
		    UART1_Send_String(N25,72);
			UART1_SendB(Note_end);
			break;
		}
		case 26://����6
		{
		    UART1_Send_String(N26,72);
			UART1_SendB(Note_end);
			break;
		}
		case 27://����7
		{
		    UART1_Send_String(N27,71);
			UART1_SendB(Note_end);
			break;
		}									
	}    
}


//---------------------------���̵���ģ�������غ�������----------------------------------------------------

void Numeric_Valve(uchar state)//����������
{
    if(state)
	{
	    //�򿪵���������
		PORTA|=BIT(2);//PA2=1
	}
	else
	{
	    //�رյ���������
		PORTA&=(~BIT(2));//PA2=0
	}
}

void Direction_Valve(uchar state)//����
{
    if(state==1)
	{
	    //����ͨ������
		PORTA|=BIT(1);//PA1=1
		UART1_Send_String("#0H",3);
		UART1_SendB(Note_end);
	}
	else
	{
	    //�����������
		PORTA&=(~BIT(1));//PA1=0
		UART1_Send_String("#0L",3);
		UART1_SendB(Note_end);
	}
}


//---------------------------������������-----------------------------------------------------------
void main(void)
{
    //��ʼ��
	uart0_init(9600);//speaking module
	uart1_init(9600);//motor board
	
	//��ѭ��
	while(1)
	{	    
	    if(uart0_received_flag==1)
		{	
			switch(uart0_received_byte)
			{
	    	    case 0x01://�������
				{
		            delay(3000);
					Play_Music(KangDingQingGe);
					break;
				}
			    case 0x02://�¹��µķ�β��
		    	{
		            delay(3500);
					Play_Music(YueGuangXiaDeFengWeiZhu);
					break;
			    }
			 	case 0x03://�������
		    	{
			        UART1_Send_String(Note_test3,14);
			    	delay(50);
					break;
			    }
				case 0x04://̫��
		    	{
			        UART1_Send_String(Note_test4,14);
			    	delay(50);
					break;
			    }
		    }
			uart0_received_byte=0;
			uart0_received_flag=0;
			UCSR0B|=BIT(RXCIE0);//ʹ��RXCIE0������λ���ֲ���
        }
	}
}
