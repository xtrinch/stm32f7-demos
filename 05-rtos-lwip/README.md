# lwip

Doesn't work yet. Check back later.

## Headers
  - 05-rtos-lwip/app_ethernet.h -          header of app_ethernet.c file
  - 05-rtos-lwip/ethernetif.h -            header for ethernetif.c file
  - 05-rtos-lwip/lcd_log_conf.h -          LCD Log configuration file
  - 05-rtos-lwip/stm32f7xx_hal_conf.h -    HAL configuration file
  - 05-rtos-lwip/stm32f7xx_it.h -          STM32 interrupt handlers header file
  - 05-rtos-lwip/main.h -                  Main program header file
  - 05-rtos-lwip/lwipopts.h -              LwIP stack configuration options
  - 05-rtos-lwip/FreeRTOSConfig.h -        FreeRTOS configuration options
  - 05-rtos-lwip/httpserver-socket.h -     header for httpserver-socket.c
  
## C files

  - 05-rtos-lwip/stm32f7xx_it.c -          STM32 interrupt handlers
  - 05-rtos-lwip/app_ethernet.c -          Ethernet specific module
  - 05-rtos-lwip/main.c -                  Main program
  - 05-rtos-lwip/syscalls.c -              System calls for bare metal
  - 05-rtos-lwip/system_stm32f7xx.c -      STM32F7xx system clock configuration file
  - 05-rtos-lwip/ethernetif.c -            Interfacing LwIP to ETH driver
  - 05-rtos-lwip/httpserver-socket.c -     httpserver socket main thread
  - 05-rtos-lwip/fsdata_custom.c -         ROM filesystem data (html pages)
