/*
* UART.c
*
* Created: 31.03.2013 16:34:53
*  Author: Alex_EXE
*/
#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include </Users/DR/Documents/Atmel Studio/Termo_v2/Termo_v2/lcd.h>



char s[2];//Массив для приема байтов
int T1,T2,T,i=0,temp=0, count=0, j=0, k=0, sec=0, min=0, Val1, Val2, rpm1, rpm=400, count2=500, count3=25, buffer[10], buffer1[10], buffer2[10], buffer3[10], buffer4[10],buffer5[10];//счетчик принятых байтов
int n, rpmi[20],Im,Tavrg,avr,OCR1A_Flag,OCR1B_Flag,Temp_I[20],l,click=0,butt_pos=0,click1=0,butt_pos1=0, tick=0;
double Tavr[100];
float  Uref=5.12,Km=0.0055,Ikm=0.000005,Tn,Ti,It,Pt=0.01,Itk=0.0005;;
unsigned int U;


int init_UART(void)
{
	//	Установка скорости 9600
	UBRRH=0;	//	UBRR=f/(16*band)-1 f=8000000Гц band=9600,
	UBRRL=103;	//	нормальный асинхронный двунаправленный режим работы
	UCSRA=0b00000000;
	UCSRB=0b10011000;	//	разрешен приём и передача по UART
	UCSRC=0b10000110;	//	8-битовая посылка
}

void send_Uart_str(unsigned char *s)//	Отправка строки
{
	while (*s != 0) send_Uart(*s++);
}

//	UART
void send_Uart(unsigned char c)//	Отправка байта
{
	while(!(UCSRA&(1<<UDRE)))	//	Устанавливается, когда регистр свободен
{}
	UDR = c;
}


void send_int_Uart(unsigned int c)//	Отправка числа от 0000 до 9999
{
	unsigned int temp,temp1;
	c=c%100000000;
	temp=c/10000;
	temp1=temp/100;
	send_Uart(temp1/10+'0');
	send_Uart(temp1%10+'0');
	temp1=temp%100;
	send_Uart(temp1/10+'0');
	send_Uart(temp1%10+'0');
	temp=c%10000;
	temp1=temp/100;
	send_Uart(temp1/10+'0');
	send_Uart(temp1%10+'0');
	temp1=temp%100;
	send_Uart(temp1/10+'0');
	send_Uart(temp1%10+'0');
	
}

/*

void send_int_Uart(unsigned int c)//	Отправка числа от 0000 до 9999
{
	unsigned char temp;
	c=c%10000;
	temp=c/100;
	send_Uart(temp/10+'0');
	send_Uart(temp%10+'0');
	temp=c%100;
	send_Uart(temp/10+'0');
	send_Uart(temp%10+'0');
}
*/

unsigned char getch_Uart(void)//	Получение байта
{
	while(!(UCSRA&(1<<RXC)))	//	Устанавливается, когда регистр свободен
{}
	return UDR;
}


ISR(USARTRXC_vect)
{
	/*
	s[i]=UDR;//принимаем байт в массив char
	i++;
	if (i == 2)//если приняли 2 байта
	{
		//send_Uart_str("T=");
		if (s[0] == 'o')
		{
			if (s[1] == 'n')
			
			//проверяем что приняли, если команду "on"
			{
				send_Uart_str("speed=160");
				i=0;
				s[0]='0';
				s[1]='0';
				speed=160;
				send_Uart(10);
				send_Uart(13);
			}
			else
			i=0;
			s[0]='0';
			s[1]='0';
		}
	}
	*/

	switch (UDR)
	{

		
		case '+':
		//	OCR1A = OCR1A+200;
		OCR1B = OCR1B+1000;
		OCR0=OCR0+2;
		//	OCR2=OCR2+2;
		if (OCR1A>54100)
		{
			OCR1A=44000;
		}
		if (OCR1B>54100)
		{
			OCR1B=44000;
		}
		if (OCR2>200)
		{
			OCR2=190;
		}
		send_int_Uart(OCR1B);
		send_Uart(13);
		send_Uart(10);
		break;
		
		case '-':
		//	OCR1A = OCR1A-200;
		OCR1B = OCR1B-1000;
		OCR0=OCR0-2;
		if (OCR1A<2100)
		{
			OCR1A=4000;
		}
		if (OCR1B<2100)
		{
			OCR1B=4000;
		}
		//	OCR0=OCR0-2;
		send_int_Uart(OCR1B);
		send_Uart(13);
		send_Uart(10);
		break;
		
		case 'd':
		DDRD|= (1<<PD7);
		send_Uart_str("d");
		send_Uart(13);
		send_Uart(10);
		//	PORTD= (1<<PD7);
		OCR2 = 0x3F; //to generate 20% duty cycle
		//TCCR2 = (1<<WGM21)|(1<<WGM20)|(1<<COM21)|(0<<COM20)|(1<<CS20);
		break;
		
		/*
		case 'n':
		//_delay_ms(1000);
		DDRA |= (1<<PA1);
		PORTA|= (0<<PA1);
		DDRB |= (0<<PB3);
		_delay_ms(100);
		send_Uart_str("went");
		send_Uart(13);
		send_Uart(10);
		DDRA = (1<<PA0);
		PORTA= (1<<PA0);
		DDRD |= (1<<PD4);
		OCR1B = 0xFFF; //to generate 20% duty cycle
		ICR1=0xFFFF;
		TCCR1A = (1<<COM1A1)|(1<<COM1B1)|(0<<WGM10)|(1<<WGM11); //Configure fast PWM, non-inverted, without prescaler
		TCCR1B = (1<<WGM12)|(1<<WGM13)|(0<<CS12)|(0<<CS11)|(1<<CS10);
		break;
		
		
		case 'p':
		//_delay_ms(250);
		DDRA |= (1<<PA0);
		PORTA|= (0<<PA0);
		DDRD |= (0<<PD4);
		_delay_ms(100);
		send_Uart_str("go");
		send_Uart(13);
		send_Uart(10);
		DDRA = (1<<PA1);
		PORTA= (1<<PA1);
		PORTB|= (1<<PB3);
		DDRB |= (1<<PB3);
		OCR0=5;
		TCCR0=(1<<COM01)|(0<<COM00)|(1<<WGM00)|(1<<WGM01)|(1<<CS02)|(0<<CS01)|(1<<CS00);
		break;
		*/

		case 'o':
		DDRA |= (1<<PA0);
		PORTA|=PORTA^(1<<PA0);
		send_Uart_str("o");
		send_Uart(13);
		send_Uart(10);
		break;
		case 't':
		send_int_Uart(10*T);
		send_Uart_str("*10E-1");
		send_Uart(13);
		send_Uart(10);
		break;
		
		case 's':
		DDRC = (1<<PC6);
		PORTC= (0<<PC6);
		DDRD = (0<<PD5);
		PORTD= (0<<PD5);
		DDRC = (1<<PC7);
		PORTC= (0<<PC7);
		DDRD = (0<<PD4);
		PORTD= (0<<PD4);
		send_Uart_str("s");
		send_Uart(13);
		send_Uart(10);
		break;
		
		case 'a':
		send_int_Uart(OCR1A);
		send_Uart(13);
		send_Uart(10);
		send_int_Uart(OCR1B);
		send_Uart(13);
		send_Uart(10);
		break;
		
	}
}


ISR(ADC_vect)
{

	Tavr[avr]=100*(((ADC*Uref/1024+17.0333)/6.11)*100-273);// умножаю на десять, чтобы сравнивать десятые доли
	avr++;
	//avr%=100;
	//Tavrg+=Tavr[avr]/100;
	/*	if (avr>=10)
	{
		avr=0;
		Tavrg=((Tavr[0]+Tavr[1]+Tavr[2]+Tavr[3]+Tavr[4]+Tavr[5]+Tavr[6]+Tavr[7]+Tavr[8]+Tavr[9])/10);
	}
	*/
	

	if (avr>=100)
	{
		avr=0;
		Tavrg=( (Tavr[0] +Tavr[1] +Tavr[2] +Tavr[3] +Tavr[4] )/100+(Tavr[5] +Tavr[6] +Tavr[7] +Tavr[8] +Tavr[9] )/100+
		(Tavr[10]+Tavr[11]+Tavr[12]+Tavr[13]+Tavr[14])/100+(Tavr[15]+Tavr[16]+Tavr[17]+Tavr[18]+Tavr[19])/100+
		(Tavr[20]+Tavr[21]+Tavr[22]+Tavr[23]+Tavr[24])/100+(Tavr[25]+Tavr[26]+Tavr[27]+Tavr[28]+Tavr[29])/100+
		(Tavr[30]+Tavr[31]+Tavr[32]+Tavr[33]+Tavr[34])/100+(Tavr[35]+Tavr[36]+Tavr[37]+Tavr[38]+Tavr[39])/100+
		(Tavr[40]+Tavr[41]+Tavr[42]+Tavr[43]+Tavr[44])/100+(Tavr[45]+Tavr[46]+Tavr[47]+Tavr[48]+Tavr[49])/100+
		(Tavr[50]+Tavr[51]+Tavr[52]+Tavr[53]+Tavr[54])/100+(Tavr[55]+Tavr[56]+Tavr[57]+Tavr[58]+Tavr[59])/100+
		(Tavr[60]+Tavr[61]+Tavr[62]+Tavr[63]+Tavr[64])/100+(Tavr[65]+Tavr[66]+Tavr[67]+Tavr[68]+Tavr[69])/100+
		(Tavr[70]+Tavr[71]+Tavr[72]+Tavr[73]+Tavr[74])/100+(Tavr[75]+Tavr[76]+Tavr[77]+Tavr[78]+Tavr[79])/100+
		(Tavr[80]+Tavr[81]+Tavr[82]+Tavr[83]+Tavr[84])/100+(Tavr[85]+Tavr[86]+Tavr[87]+Tavr[88]+Tavr[89])/100+
		(Tavr[90]+Tavr[91]+Tavr[92]+Tavr[93]+Tavr[94])/100+(Tavr[95]+Tavr[96]+Tavr[97]+Tavr[98]+Tavr[99])/100);
	}
	


ADCSRA|=1<<ADSC;

}



ISR(TIMER2_COMP_vect)
{

	TCNT2=0;
	
	j++;
	if (j==30)
	{j=0;
		switch(PINA&(1<<PC5))
		{
			case 0:
			if (butt_pos==1)
			{click++;
				butt_pos=0;
			}
			if (butt_pos==0)
			{
				click=click;
			}
			
			case 32:
			click=click;
			butt_pos=1;
		}


		switch(PINA&(1<<PC4))
			{
				case 0:
			
				if (butt_pos1==1)
				{click1++;
					butt_pos1=0;
				}
				if (butt_pos1==0)
				{
					click1=click1;
				}
				
				case 16:
				
				
				click1=click1;
				butt_pos1=1;
			}
 
		
		i++;
			T1=Tavrg/100;
			T2=Tavrg%100;
		lcd_init(LCD_DISP_ON_CURSOR);
		lcd_clrscr();
		lcd_home();

	
		itoa(T1, buffer, 10);
		if (T2/10==0)
		{itoa(0,buffer5,10);
		itoa(T2, buffer4, 10);	} 
		else
		{itoa(T2, buffer4, 10);	}
		itoa(T2, buffer4, 10);
		itoa(count3, buffer3, 10);
		lcd_puts("temp: ");
		
		if (click%2==0)
		{
			lcd_puts(" off ");
		} 
		else
		{
			lcd_puts(buffer);
			lcd_puts(".");
			if (T2/10==0)
			{lcd_puts(buffer5);
			lcd_puts(buffer4);	}
			else
		{lcd_puts(buffer4);	}
			lcd_puts("  ");
			lcd_puts(buffer3);
		}
		
		
		lcd_gotoxy(0,1);
		itoa(count2, buffer1, 10);
		itoa((100*count3-Tavrg)/*rpm1*/, buffer2, 10);
		
		if (click1%2==0)
		{
			lcd_puts("rpm : ");
			//lcd_puts("   ");
			lcd_puts(" off ");
		} 
		else
		{
		if (rpm1/1000==0)
		{
			lcd_puts("rpm : ");
			lcd_puts(buffer2);
			lcd_puts(" ");
		} 
		else
		{
			lcd_puts("rpm : ");
			lcd_puts(buffer2);
		}
//		lcd_puts("rpm : ");
//		lcd_puts(buffer2);
		lcd_puts(" ");
		lcd_puts(buffer1);
		
		}	
	
		
		
		if (i==8)//было 8 - при 8 отчитываем секунду
		{
			if (click1%2==0)
			{DDRB=DDRB&~(1<<PB3);
				PORTB=PORTB&~(1<<PB3);
				i--;
			} 
			else
			{
				send_int_Uart(1);
				send_Uart(13);
				send_Uart(10);
				DDRB=(1<<PB3);
			i=0;
			rpm1=60*count/10;
			//rpmi[k]=rpm1;
			//делаем кольцнвой буфер//
			k++;
			k%=20;
			Im-=rpmi[k];
			rpmi[k]=rpm1;
			Im+=rpm1;
			//***********************//
			
			n=Km*(count2-rpm1)+Ikm*(20*count2-Im);
			
			//	send_int_Uart(Tavrg);
			//	send_Uart(13);
			//	send_Uart(10);
			
			if (count2==0)
		{OCR0=0;}
			else
			{OCR0=OCR0+n;
				if (OCR0>=100)
			{OCR0=80;}
			}
			if (OCR0<=0)
		{OCR0=0;}
			
			/*		if (rpm1>=count2)
			{
				if (count2==0)
			{OCR0=0;}
				else
				{OCR0 = OCR0-n;
				}
			}
			else if (rpm1<count2)
			{
				OCR0 = OCR0+2;
			}
			*/
			count=0;
			PORTD=PORTD^(1<<PD6);
			}
			
			//	if (100*count3>=Tavrg)				//гребанны AVR GCC не знает signed
			//		{
				
					if (click%2==0)
					{
						DDRD  = DDRD &~(1<<PD4);
						DDRD  = DDRD &~(1<<PD5);
						PORTD = PORTD&~(1<<PD4);
						PORTD = PORTD&~(1<<PD5);
						DDRA  = DDRA &~(1<<PA0);
						DDRA  = DDRA &~(1<<PA1);
						PORTA = PORTA&~(1<<PA0);
						PORTA = PORTA&~(1<<PA1);
					}
					else
					{
				Tn=Pt*(100*count3-Tavrg);//+Itk*(20*100*count3-It);
				Ti=(Itk*(20*100)*count3-Itk*It);///2000;
			//			}		//поэтому меняем знак искусственно
		//		else{Tn=Pt*(Tavrg-100*count3);}//+Itk*(It-20*100*count3);}		//
		
		if (Tavrg>=(count3*100+5))
		{
			OCR1B_Flag=0;
			if (OCR1A_Flag==0)
			{
				OCR1A_Flag++;
				OCR1A = 0x7F;
			}
			
			if (PORTA&(1<<PA0))
			{
				TCCR1A = TCCR1A&~(1<<COM1B1);
				DDRA |= (1<<PA0);
				PORTA =PORTA&~(1<<PA0);
				DDRD |= (1<<PD4);
				PORTD =PORTD&~(1<<PD4);
				OCR1B = 0x00;
				_delay_ms(60);
				
			}
			else
			{
				TCCR1A |=(1<<COM1A1);
				DDRA |= (1<<PA1);
				PORTA|= (1<<PA1);
				DDRD|= (1<<PD5);
				//OCR1A = OCR1A+Tn;
				if (OCR1A>=250)
			{OCR1A=240;}
			}
		}
		
		
		else if (Tavrg<=(count3*100-5))
		{
			OCR1A_Flag=0;
			if (OCR1B_Flag==0)
			{OCR1B_Flag++;
				OCR1B = 0x84;
			}
			
			if (PORTA&(1<<PA1))
			{
				TCCR1A = TCCR1A&~(1<<COM1A1);
				DDRA |= (1<<PA1);
				PORTA =PORTA&~(1<<PA1);
				DDRD |= (1<<PD5);
				PORTD=PORTD&~(1<<PD5);
				OCR1A = 0x00;
				_delay_ms(60);
			}
			
			else
			{
				TCCR1A|= (1<<COM1B1);
				DDRA |= (1<<PA0);
				PORTA|= (1<<PA0);
				DDRD|= (1<<PD4);
				//OCR1B = OCR1B+Tn; //to generate 20% duty cycle
				if (OCR1B>=245)
			{OCR1B=235;}
			}
		}
			
			/*
			send_int_Uart(tick++);
			send_Uart(13);
			send_Uart(10);
			*/
			if (tick==59)
			{tick=0;
			}
			/*
			send_int_Uart(Tavrg);
			send_Uart(13);
			send_Uart(10);
				*/
			
			sec++;
			if (sec==15)
			{
			
			l++;
			l%=20;
			It-=Temp_I[l];
			Temp_I[l]=Tavrg;
			It+=Temp_I[l];
				sec=0;
				
				if (Tavrg>=(count3*100+5))
					{OCR1A = OCR1A-Tn;}
						
				else if (Tavrg<(count3*100-5))	
					{OCR1B = OCR1B+Tn;}
			}
			
					}			
		}
	}
}

ISR(INT2_vect)
{
	if (click%2==0)
	{
	} 
	else
	{
	
	
	switch(PINC&(1<<PC7))
	{
		case 128:
		count3--;
			
		break;
		
		case 0:
		count3++;
	
	//	_delay_ms(100);
		break;
		
		default:
		count3=count3;
	
	}
	if (count3<=15)
	{
		count3=15;
	}
	if (count3>=75)
	{
		count3=75;
	}
		//send_int_Uart(PINC&(1<<PC7));
		//send_Uart(13);
		//send_Uart(10);
		/*
	send_int_Uart(count);
	send_Uart(13);
	send_Uart(10);
*/
	//PORTD=PORTD^(1<<PD6);
}
	}

ISR(INT1_vect)
{
	count++;
}


ISR(INT0_vect)
{
	if (click1%2==0)
	{
	} 
	else
	{
	
	switch(PIND&(1<<PD7))
	{
		case 128:
		count2-=50;
			
		break;
		
		case 0:
		count2+=50;
	
	//	_delay_ms(100);
		break;
		
		default:
		count2=count2;
	
	}
	if (count2<=0)
	{
		count2=0;
	}
	if (count2>=3200)
	{
		count2=3200;
	}
		//send_int_Uart(PIND&(1<<PD7));
		//send_Uart(13);
		//send_Uart(10);
		/*
	send_int_Uart(count);
	send_Uart(13);
	send_Uart(10);
*/
	//PORTD=PORTD^(1<<PD6);
}
}
int main(void)
{
	
	//обнуляем кольцевой буфер для интегральной компоненты ПИД регулятора//
	for (k=0;k<20;k++)
	{rpmi[k]=0;}
		k=0;
		
	for (l=0;l<20;l++)
	{Temp_I[l]=0;}
		l=0;
	//********************************************************************//	
	
	//усредняем темературу и работаем относительно нее
	for (avr=0;avr<10;avr++)
	{Tavr[avr]=0;}
		avr=0;
	//**********************************************//
	ADMUX|=(0<<REFS1)|(1<<REFS0)|(0<<ADLAR)|(0<<MUX4)|(0<<MUX3)|(0<<MUX2)|(1<<MUX1)|(1<<MUX0);
	ADCSRA|=(1<<ADEN)|(1<<ADSC)|(1<<ADIE)|(1<<ADPS0)|(1<<ADPS1)|(1<<ADPS2);
	//OCR0=78;
	
	
	
	DDRB=(1<<PB3);
	OCR0=0;
	TCCR0=(1<<COM01)|(0<<COM00)|(1<<WGM00)|(1<<WGM01)|(1<<CS02)|(0<<CS01)|(1<<CS00);
	

	
	//OCR1B=0x0F;
	OCR1A = 0x00; //to generate 20% duty cycle
	ICR1=0xFF;
	TCCR1A = (0<<COM1A1)|(0<<COM1B1)|(0<<WGM10)|(1<<WGM11); //Configure fast PWM, non-inverted, without prescaler
	TCCR1B = (1<<WGM12)|(1<<WGM13)|(0<<CS12)|(0<<CS11)|(1<<CS10);
	//PORTD|= (1<<PD5);
	DDRD |= (1<<PD5);
	DDRD |= (1<<PD4);
	
	DDRA|=(0<<PA5);
	PORTA|=(1<<PA5);
	
		
		
	OCR2=255;
	TCCR2=(0<<COM21)|(0<<COM20)|(0<<WGM20)|(1<<WGM21)|(1<<CS22)|(1<<CS21)|(0<<CS20);
	TIMSK|=(1<<OCIE2);
	
	MCUCSR=(1<<ISC2);
	MCUCR|=(1<<ISC10)|(1<<ISC11)|(1<<ISC00)|(1<<ISC01);
	GICR|=(1<<INT2)|(1<<INT1)|(1<<INT0);
	
	DDRD|= (1<<PD6);
	

	sei();
	
	//DDRD = 0XFF;
init_UART();					//	инициализация UART
		_delay_ms(1000);				//	задержка 1c
/*	send_Uart_str("alex-exe.ru");	//	отправка строки
	send_Uart(13);	
	send_Uart(10);				//	перенос строки
	send_int_Uart(2013);			//	отправка числа
	send_Uart(13);	
	send_Uart(10);				//	перенос строки
*/	
    while(1)						//	бесконечный рабочий цикл
    {}
}