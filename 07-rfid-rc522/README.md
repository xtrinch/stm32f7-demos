# rfid-rc522

Uses a modified version of [tilz0r's library for rfid-rc522](https://github.com/MaJerle/stm32f429/blob/master/00-STM32F429_LIBRARIES/tm_stm32f4_mfrc522.c), which does not rely on their SPI libraries, but rather on the HAL SPI libraries. Logging of found tag is done via lcd log utility that can be found in STM32F7Cube framework.

[logo]: demo.jpeg "Demo in action"
