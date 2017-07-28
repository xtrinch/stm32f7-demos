#include "stm32f769i_discovery.h"
#include "stm32f7xx.h"

int main(void)
{
    int i = 0;

    BSP_LED_Init(LED_GREEN);
    BSP_LED_Init(LED_RED);
    BSP_LED_Toggle(LED_GREEN);
    
    for(;;)
    {
        BSP_LED_Toggle(LED_GREEN);
        BSP_LED_Toggle(LED_RED);

        for (int j = 0; j < 10000000; j++)
            ;

        printf("Hello, world! -- %i\n", i++);
    }

    return 0;
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line)
{
	/* Infinite loop */
	/* Use GDB to find out why we're here */
	while (1);
}
#endif