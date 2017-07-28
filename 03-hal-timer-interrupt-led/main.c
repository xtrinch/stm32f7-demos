#include "stm32f7xx.h"
#include "stm32f7xx_hal.h"
#include <stdio.h>

#define LED_GREEN					0x0020U
#define LED_RED						0x2000U

static TIM_HandleTypeDef s_TimerInstance = {
	.Instance = TIM2
};

// 16mHz clock on STM32F769I peripherals by default
void InitializeTimer()
{
	__TIM2_CLK_ENABLE();
	// 16 bit prescaler on timer 2, max value 65535
	s_TimerInstance.Init.Prescaler = (16000-1);
	s_TimerInstance.Init.CounterMode = TIM_COUNTERMODE_UP;
	// 16000000 / 16000 = 1000
	s_TimerInstance.Init.Period = (1000-1);
	s_TimerInstance.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	s_TimerInstance.Init.RepetitionCounter = 0;
	HAL_TIM_Base_Init(&s_TimerInstance);
	// automatically generate update events once timer reaches set value
	HAL_TIM_Base_Start_IT(&s_TimerInstance); // triggers interrupt
}

void InitializeLED()
{
	__GPIOJ_CLK_ENABLE();
	GPIO_InitTypeDef GPIO_Init;
	GPIO_Init.Pin = (LED_GREEN | LED_RED);
	GPIO_Init.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_Init.Pull = GPIO_NOPULL;
	GPIO_Init.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOJ, &GPIO_Init);
	// turn leds off
	HAL_GPIO_WritePin(GPIOJ, (LED_GREEN | LED_RED), GPIO_PIN_RESET);
}

int main(void) {
	SystemInit();

	InitializeLED();
	InitializeTimer();

	for (;;) {
		if (__HAL_TIM_GET_FLAG(&s_TimerInstance, TIM_FLAG_UPDATE) != RESET) {
			__HAL_TIM_CLEAR_IT(&s_TimerInstance, TIM_IT_UPDATE);
			HAL_GPIO_TogglePin(GPIOJ, (LED_GREEN | LED_RED));
		}
	}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line)
{
	/* Infinite loop /
	/ Use GDB to find out why we're here */
	while (1);
}
#endif
