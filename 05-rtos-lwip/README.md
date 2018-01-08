# lwip

Doesn't work yet. Check back later.

## Headers
  - 05-rtos-lwip/inc/app_ethernet.h -             header of app_ethernet.c file
  - 05-rtos-lwip/inc/ethernetif.h -               header for ethernetif.c file
  - 05-rtos-lwip/inc/lcd_log_conf.h -             LCD Log configuration file
  - 05-rtos-lwip/inc/stm32f7xx_hal_conf.h -       HAL configuration file
  - 05-rtos-lwip/inc/stm32f7xx_it.h -             STM32 interrupt handlers header file
  - 05-rtos-lwip/inc/main.h -                     Main program header file
  - 05-rtos-lwip/inc/lwipopts.h -                 LwIP stack configuration options
  - 05-rtos-lwip/inc/FreeRTOSConfig.h -           FreeRTOS configuration options
  - 05-rtos-lwip/inc/httpserver-socket.h -        header for httpserver-socket.c
  
## C files

  - 05-rtos-lwip/src/stm32f7xx_it.c -             STM32 interrupt handlers
  - 05-rtos-lwip/src/app_ethernet.c -             Ethernet specific module
  - 05-rtos-lwip/src/main.c -                     Main program
  - 05-rtos-lwip/src/syscalls.c -                 System calls for bare metal
  - 05-rtos-lwip/src/system_stm32f7xx.c -         STM32F7xx system clock configuration file
  - 05-rtos-lwip/src/ethernetif.c -               Interfacing LwIP to ETH driver
  - 05-rtos-lwip/src/httpserver-socket.c -        httpserver socket main thread
  - 05-rtos-lwip/src/fsdata_custom.c -            ROM filesystem data (html pages)
  - 05-rtos-lwip/src/stm32f7xx_hal_timebase_tim - 1hmz counter clock
