#include "stm32f10x.h"

void TIM_init(void);
void delay_ms(uint16_t t);
void delay_us(uint16_t t);
void trig_inside(void);
void trig_outside(void);
u32 getdistance_inside(void);
u32 getdistance_outside(void);
u32 calculer(u32 count);
void SysTick_Handler(void);
void delay_PWM(uint16_t t);
void EXTI_init(void);
void EXTI4_IRQHandler(void);
void bright_500ms(void);
void quench_500ms(void);
void usart_init(void);
void usart1_sendByte(unsigned char c);
void USART1_IRQHandler(void);

int car = 20;
uint16_t light[] = {0x3F,0x06,0x5B,0x4F,0x66,0x6D,0x7D,0x07,0x7F,0x6F};

int main()
{
	RCC->APB2ENR |= 0xFC | (1 << 14 | 1 << 1);	//enable clocks for GPIO, USART1 and AFIO
	RCC->APB1ENR |= (1 << 0);  //enalbe TIM2 clock
	GPIOA->CRL = 0x44443B83;		//PA2: TIM2 chennel 3. PA1: echo. PA0: trig. PA3 : test. PA4: as EXTI4 input, but this place we need to set it is output, because when it toggle, interrupt will receive notice.
	GPIOA->CRH = 0x834448b4;		//PA15: echo, PA14: trig, PA9:TX, PA10:RX
	GPIOB->CRL = 0x33333333;		//PB0-PB7 all output 
	GPIOB->CRH = 0x33333333;		//PB8: LSB, PB9: MSB 
	TIM_init();		//TIM2 initialize
	EXTI_init();
	usart_init();
	TIM2->CCR3 = 100;	//At first, remain close
	SysTick->LOAD = 9000000 - 1;
	SysTick->CTRL = 0x03;
	while(1)
	{
		GPIOA->ODR ^= (1 << 4);		//Continue monitor the EXTI4, and get car value to print
	}
}

void EXTI_init(void)
{
	GPIOA->ODR |= (1 << 4);	//pull-up PA4
	AFIO->EXTICR[1] = (1 << 0);
	EXTI->RTSR = (1 << 4);		//rising edge, so if I make PB4 high, it will call interrupt
	EXTI->IMR = (1 <<4);
	NVIC_EnableIRQ(EXTI4_IRQn);
}

void usart_init(void)
{
	GPIOA->ODR |= (1 << 10); //pull-up PA10
	USART1->CR1 = 0x2048;		//receive int.enable, receive enable
	USART1->BRR = 7500;		//72MHz/9600bps = 7500
	NVIC_EnableIRQ(USART1_IRQn);
}

void delay_ms(uint16_t t)		//delay t ms
{
	TIM2->PSC = 2*t - 1;
	TIM2->ARR = 36000 - 1;
	TIM2->CR1 = 1;
	while((TIM2->SR & 1) == 0);
	TIM2->CR1 = 0;
	TIM2->SR = 0;		//clear UIF flag
}

void delay_us(uint16_t t)		//delay t us
{
	TIM2->ARR = 72*t - 1;
	TIM2->CR1 = 1;
	while((TIM2->SR & 1) == 0);
	TIM2->CR1 = 0;
	TIM2->SR = 0;
}

void delay_PWM(uint16_t t)
{
	TIM2->PSC = 1000 - 1;
	TIM2->ARR = 1440 - 1;
	for(int i=0;i<t;i++)
	{
		TIM2->CR1 = 1;	//start counting
		while((TIM2->SR & 1) == 0);
		TIM2->CR1 = 0;	//stop counting
		TIM2->SR = 0;		//clear UIF
	}
}

void TIM_init(void)
{
	/*TIM2 init*/
	TIM2->CCR3 = 3000;	
	TIM2->CCER = 0x1 << 8;	//CC3P = 0, CC3E = 1
	TIM2->CCMR2 = 0x0030;		//toggle chennel 3
}

void trig_inside(void)		//To send a 10 us square wave
{
	GPIOA->ODR |= (1 << 0);
	delay_us(10);
	GPIOA->ODR ^= (1 << 0);
}

void trig_outside(void)
{
	GPIOA->ODR |= (1 << 14);
	delay_us(10);
	GPIOA->ODR ^= (1 << 14);
}

u32 getdistance_inside(void)
{
	u32 count = 0, Distance = 0;
	trig_inside();		
	
	TIM2->ARR = 65535;	// count times will not bigger than 2^16 - 1
	TIM2->CNT = 0;	//To use this count time
	while((GPIOA->IDR & (1 << 1)) == 0);	//Wait until PA1 is high
	TIM2->CR1 = 1; //start counting
	while((GPIOA->IDR & (1 << 1)) == 1); 	//Wait until PA1 is low
	TIM2->CR1 = 0;	//Stop counting
	count = TIM2->CNT;
	Distance = calculer(count);
	return Distance;
}

u32 getdistance_outside(void)
{
	u32 count = 0, Distance = 0;
	trig_outside();		
	
	TIM2->ARR = 65535;	// count times will not bigger than 2^16 - 1
	TIM2->CNT = 0;	//To use this count time
	while((GPIOA->IDR & (1 << 15)) == 0);	//Wait until PA15 is high
	TIM2->CR1 = 1; //start counting
	while((GPIOA->IDR & (1 << 15)) == 1); 	//Wait until PA15 is low
	TIM2->CR1 = 0;	//Stop counting
	count = TIM2->CNT;
	Distance = calculer(count);
	return Distance;
}


u32 calculer(u32 count)
{
	u32 Distance;
	Distance = (float)(count / 72000000);
	return Distance;
}

void SysTick_Handler(void)	//To receive data every second
{
	u32 dis1 = 652, dis2 = 652;	//dis1 is outside HC-SR04, dis2 is inside HC-SR04, and because the longest distance it can detect is 2m
	dis1 = getdistance_outside();
	dis2 = getdistance_inside();
	
	if(dis1 < 100 && (TIM2->CCR3 == 100) && dis2 > 430 && car < 20)	//car is incoming
	{
		TIM2->CCR3 = 75;	//open
	}
	else if(dis2 < 100 && (TIM2->CCR3 == 100) && dis1 > 430 && car < 20)		//car is outcoming
	{
		TIM2->CCR3 = 75;	//open
	}
	else if(dis1 > 430 && (TIM2->CCR3 == 75) && dis2 > 100)
	{
		TIM2->CCR3 = 100;	//close
	}
	else if(dis1 > 100 && (TIM2->CCR3 == 75) && dis2 > 430)
	{
		TIM2->CCR3 = 100;	//close
	}
	else
	{
		delay_PWM(30);	//delay
	}
}

void bright_500ms(void)
{
	TIM2->PSC = 2*500 - 1;	//500 ms
	TIM2->ARR = 36000 - 1;
	TIM2->CR1 = 1;
	while((TIM2->SR & 1) != 0)
	{
		GPIOB->ODR = light[0];
		GPIOB->ODR |= (1 << 9);
		delay_ms(10);
		GPIOB->ODR ^= (1 << 9);
		GPIOB->ODR = light[0];
		GPIOB->ODR |= (1 << 8);
		delay_ms(10);
		GPIOB->ODR ^= (1 << 8);
	}
	TIM2->CR1 = 0;
	TIM2->SR = 0;		//clear UIF flag
}

void quench_500ms(void)
{
	TIM2->PSC = 2*500 - 1;	//500 ms
	TIM2->ARR = 36000 - 1;
	TIM2->CR1 = 1;
	GPIOB->ODR &= ~(1 << 8);
	GPIOB->ODR &= ~(1 << 9);
	while((TIM2->SR & 1) == 0);
	TIM2->CR1 = 0;
	TIM2->SR = 0;		//clear UIF flag
}

void EXTI4_IRQHandler()
{
	EXTI->PR = (1 << 4);	//clear pending flag.
	if(car < 10)		//MSB will print 0, LSB will print car number
	{
		GPIOB->ODR = light[0];
		GPIOB->ODR |= (1 << 9);
		delay_ms(10);
		GPIOB->ODR ^= (1 << 9);
		GPIOB->ODR = light[car];
		GPIOB->ODR |= (1 << 8);
		delay_ms(10);
		GPIOB->ODR ^= (1 << 8);
	}
	else if(car >= 10)	//MSB will print number of (car / 10), LSB will print number of (car % 10)
	{
		int first_number = car % 10;
		GPIOB->ODR = light[first_number];
		GPIOB->ODR |= (1 << 9);
		delay_ms(10);
		GPIOB->ODR ^= (1 << 9);
		int second_number = car / 10;
		GPIOB->ODR = light[second_number];
		GPIOB->ODR |= (1 << 8);
		delay_ms(10);
		GPIOB->ODR ^= (1 << 8);
	}
	else if(car == 0)
	{
		bright_500ms();		//bright 500ms and change to quench.
		quench_500ms();		//quench 500ms and change to bright.
	}
}

void USART1_IRQHandler(void)		//It can work,I have tested. When USART1->CR1's TCIE and TC is 1, it will keep continuously sending.   
{
	if(car < 10)
	{
		char c = '0' + car;
		usart1_sendByte(c);
		usart1_sendByte(' ');
		delay_ms(100);
	}
	else
	{
		char c1 = '0' + (car / 10), c2 = '0' + (car % 10);
		usart1_sendByte(c1);
		usart1_sendByte(c2);
		usart1_sendByte(' ');
		delay_ms(100);
	}
}

void usart1_sendByte(unsigned char c)		//The function of sending data.
{
	USART1->DR = c;
	while((USART1->SR & (1 << 6)) == 0);	//wait until the TC flag is set
	USART1->SR &= ~(1 << 6);
}
