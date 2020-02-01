#include "main.h"

GPIO_InitTypeDef gpioInit;

void init(void);
void delay(unsigned long uDelay);

int main() {
	init();
	while(1) {
		GPIO_SetBits(GPIOC, GPIO_Pin_13);
		delay(0xfffff);
		GPIO_ResetBits(GPIOC, GPIO_Pin_13);
		delay(0xfffff);
	}
}

void init(void) {
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	gpioInit.GPIO_Mode=GPIO_Mode_Out_PP;
	gpioInit.GPIO_Speed=GPIO_Speed_50MHz;
	gpioInit.GPIO_Pin=GPIO_Pin_13;
	GPIO_Init(GPIOC, &gpioInit);
}

void delay(unsigned long uDelay) {
	while(--uDelay);
}
