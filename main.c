#include <stdint.h>
#include <stm32f10x.h>

void delay(uint32_t ticks) {
	for (int i=0; i<ticks; i++) {
		__NOP();
	}
}

void setup_ADC(uint8_t adc_num)
{
	ADC_TypeDef* adc;
	uint32_t enr;
	if (adc_num == 1)
	{
		adc = ADC1;
		RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
	}
	else if (adc_num == 2)
	{
		adc = ADC2;
		RCC->APB2ENR |= RCC_APB2ENR_ADC2EN;
	}
	else
	{
		return;
	}

	RCC->APB2ENR |= enr;
	RCC->CFGR |= RCC_CFGR_ADCPRE_DIV8;

	adc->CR2 |= ADC_CR2_ADON;

	adc->CR2 |= ADC_CR2_CAL;
	while (adc->CR2 & ADC_CR2_CAL)
	{}
	delay(2);

	adc->CR2 &= ~ADC_CR2_ALIGN;

	adc->CR2 |= ADC_CR2_CONT;

	adc->CR1 |= ADC_CR1_EOCIE;

	adc->SQR3 |= ADC_SQR3_SQ1_0;

	// samples speed
	adc->SMPR2 |= ADC_SMPR2_SMP0;

	NVIC_ClearPendingIRQ(ADC1_2_IRQn);
	NVIC_SetPriority(ADC1_2_IRQn, 2);
	NVIC_EnableIRQ(ADC1_2_IRQn);

	adc->CR2 |= ADC_CR2_SWSTART;
}

void setup_stepper_motor()
{
	GPIOA->CRH &= ~GPIO_CRH_CNF8;
	GPIOA->CRH |= GPIO_CRH_MODE8_0;
	
	GPIOA->CRH &= ~GPIO_CRH_CNF9;
	GPIOA->CRH |= GPIO_CRH_MODE9_0;

	GPIOA->CRH &= ~GPIO_CRH_CNF10;
	GPIOA->CRH |= GPIO_CRH_MODE10_0;

	GPIOA->CRH &= ~GPIO_CRH_CNF11;
	GPIOA->CRH |= GPIO_CRH_MODE11_0;

}

void setup_TIM3()
{
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

	TIM2->PSC = 17999;						// tick of timer is one millisecond
	TIM2->ARR = 1;

	TIM3->DIER |= TIM_DIER_UIE;

	NVIC_ClearPendingIRQ(TIM3_IRQn);
	NVIC_SetPriority(TIM3_IRQn, 1);
	NVIC_EnableIRQ(TIM3_IRQn);

	TIM3->CR1 |= TIM_CR1_CEN;
}

void setup_servo_motor()
{
	GPIOB->CRH &= ~GPIO_CRH_CNF11;
	GPIOB->CRH |= GPIO_CRH_MODE11_0;
}

void run_stepper_motor_cycle(uint8_t isBackward)
{
	uint16_t steps_pins[] = {
		GPIO_ODR_ODR8,
		GPIO_ODR_ODR9,
		GPIO_ODR_ODR10,
		GPIO_ODR_ODR11,
	};

	if (isBackward & 1)
	{
		for (uint8_t i = 0; i < 4; i++)
		{
			GPIOA->BSRR = steps_pins[i]
			| (steps_pins[0] << 16U)
			| (steps_pins[1] << 16U)
			| (steps_pins[2] << 16U)
			| (steps_pins[3] << 16U);

			delay(27000);
		}
	}
	else
	{
		for (int8_t i = 3; i >= 0; i--)
		{
			
			GPIOA->BSRR = steps_pins[i]
			| (steps_pins[0] << 16U)
			| (steps_pins[1] << 16U)
			| (steps_pins[2] << 16U)
			| (steps_pins[3] << 16U);

			delay(27000);
		}
	}

}

uint8_t angle = 0;

uint16_t servo_counter = 0;

void run_servo_motor_cycle()
{
	servo_counter++;
	
	if (servo_counter <= angle)
	{
		GPIOB->BSRR = GPIO_ODR_ODR11;
	}
	else
	{
		GPIOB->BSRR = GPIO_ODR_ODR11 << 16;
	}
	
	if (servo_counter >= 40U)
	{
		servo_counter = 0;
	}
}

#define SERVO_WORK_DURATION 10000000
#define SERVO_OPEN_ANGLE 3U
#define SERVO_CLOSE_ANGLE 1U
#define STEPPER_WORK_DURATION 250

uint8_t isClosed = 0;
uint32_t open_counter = 0;

void open_window()
{
	if (!isClosed)
	{
		return;
	}

	angle = SERVO_OPEN_ANGLE;
	delay(SERVO_WORK_DURATION);

	angle = 0;

	for (; open_counter < STEPPER_WORK_DURATION; open_counter++)
	{
		run_stepper_motor_cycle(0);
	}

	open_counter = 0;
	isClosed = 0;
}

uint32_t close_counter = 0;

void close_window()
{
	if (isClosed)
	{
		return;
	}

	for (; close_counter < STEPPER_WORK_DURATION; close_counter++)
	{
		run_stepper_motor_cycle(1);
	}

	angle = SERVO_CLOSE_ANGLE;
	delay(SERVO_WORK_DURATION);
	angle = 0;

	close_counter = 0;
	isClosed = 1;
}

int __attribute((noreturn)) main(void)
{
	// Enable clock for GPIOA, GPIOB, GPIOC,
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN;
	RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;

	// Enable PC13 push-pull mode
	GPIOC->CRH &= ~GPIO_CRH_CNF13; //clear cnf bits
	GPIOC->CRH |= GPIO_CRH_MODE13_0; //Max speed = 10Mhz

	setup_TIM3();
	setup_stepper_motor();
	setup_servo_motor();
	setup_ADC(1);

    while (1)
	{
	}
}

void TIM3_IRQHandler(void)
{
	if ((TIM3->SR & TIM_SR_UIF))
	{
		TIM3->SR &= ~TIM_SR_UIF;
		// GPIOC->BSRR = ((GPIOC->ODR & GPIO_ODR_ODR13) << 16) | (~GPIOC->ODR & GPIO_ODR_ODR13);
		if (angle > 0)
		{
			GPIOC->BSRR = GPIO_ODR_ODR13;
		}
		else
		{
			GPIOC->BSRR = GPIO_ODR_ODR13 << 16u;
		}

		run_servo_motor_cycle();
	}
}

void ADC1_2_IRQHandler(void)
{
	if (ADC1->SR & ADC_SR_EOC)
	{
		if ((ADC1->DR & 0x0FFF) > 0x0100)
		{
			//GPIOC->BSRR = GPIO_ODR_ODR13 << 16;
			open_window();
			
		}
		else
		{
			//GPIOC->BSRR = GPIO_ODR_ODR13;
			close_window();
		}
	}
}
