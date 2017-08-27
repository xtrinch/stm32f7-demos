/**
 * |----------------------------------------------------------------------
 * | Copyright (C) Tilen Majerle, 2014
 * | Copyright (C) xtrinch, 2017
 * | This program is free software: you can redistribute it and/or modify
 * | it under the terms of the GNU General Public License as published by
 * | the Free Software Foundation, either version 3 of the License, or
 * | any later version.
 * |
 * | This program is distributed in the hope that it will be useful,
 * | but WITHOUT ANY WARRANTY; without even the implied warranty of
 * | MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * | GNU General Public License for more details.
 * |
 * | You should have received a copy of the GNU General Public License
 * | along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * |----------------------------------------------------------------------
 */
#include "rfid-rc522.h"
#include "lcd_log.h"
#include <stdlib.h>

extern SPI_HandleTypeDef SpiHandle;

void RFID_RC522_Init(void) {

	TM_MFRC522_CS_Write(GPIO_PIN_SET);

	TM_MFRC522_Reset();

	TM_MFRC522_WriteRegister(MFRC522_REG_T_MODE, 0x8D);
	TM_MFRC522_WriteRegister(MFRC522_REG_T_PRESCALER, 0x3E);
	TM_MFRC522_WriteRegister(MFRC522_REG_T_RELOAD_H, 0x03);
	TM_MFRC522_WriteRegister(MFRC522_REG_T_RELOAD_L, 0xE8);

	// 48dB gain
	TM_MFRC522_WriteRegister(MFRC522_REG_RF_CFG, 0x70);

	TM_MFRC522_WriteRegister(MFRC522_REG_TX_AUTO, 0x40);
	TM_MFRC522_WriteRegister(ModeReg, 0x3D);

	TM_MFRC522_AntennaOn();		//Open the antenna
}

TM_MFRC522_Status_t TM_MFRC522_Check(uint8_t* id) {
	TM_MFRC522_Status_t status;
	//Find cards, return card type

	status = TM_MFRC522_Request(PICC_REQIDL, id);

	if (status == MI_OK) {
		//Card detected
		//Anti-collision, return card serial number 4 bytes
		status = TM_MFRC522_Anticoll(id);
	}
	TM_MFRC522_Halt();			//Command card into hibernation

	return status;
}

TM_MFRC522_Status_t TM_MFRC522_Compare(uint8_t* CardID, uint8_t* CompareID) {
	uint8_t i;
	for (i = 0; i < 5; i++) {
		if (CardID[i] != CompareID[i]) {
			return MI_ERR;
		}
	}
	return MI_OK;
}

void handleError() {
	while(1) {
		BSP_LED_On(LED_RED);
		HAL_Delay(100);
		BSP_LED_Off(LED_RED);
		HAL_Delay(100);
	}
}

void TM_MFRC522_WriteRegister(uint8_t addr, uint8_t val) {
	//CS low
	TM_MFRC522_CS_Write(GPIO_PIN_RESET);

	HAL_StatusTypeDef transmitStatus;
	
	//Send address ## HAL_MAX_DELAY --> infinite poll until process is successful
	addr = (addr << 1) & 0x7E;

	transmitStatus = HAL_SPI_Transmit(&SpiHandle, &addr, 1, HAL_MAX_DELAY);
	if (transmitStatus != HAL_SPI_ERROR_NONE) {
		handleError();
	}
	//Send data
	transmitStatus = HAL_SPI_Transmit(&SpiHandle, &val, 1, HAL_MAX_DELAY);
	if (transmitStatus != HAL_SPI_ERROR_NONE) {
		handleError();
	}

	//CS high
	TM_MFRC522_CS_Write(GPIO_PIN_SET);
}

uint8_t TM_MFRC522_ReadRegister(uint8_t addr) {
	uint8_t val = 0x00;
	uint8_t retval = 0x00;
	//CS low
	TM_MFRC522_CS_Write(GPIO_PIN_RESET);

	HAL_StatusTypeDef transmitStatus;
	addr = (addr << 1) | 0x80;

	transmitStatus = HAL_SPI_Transmit(&SpiHandle, &addr, 1, HAL_MAX_DELAY);
	if (transmitStatus != HAL_SPI_ERROR_NONE) {
		handleError();
	}

	uint8_t dummy = MFRC522_DUMMY;
	transmitStatus = HAL_SPI_TransmitReceive(&SpiHandle, &dummy, &val, 1, HAL_MAX_DELAY);
	if (transmitStatus != HAL_SPI_ERROR_NONE) {
		handleError();
	}

	//CS high
	TM_MFRC522_CS_Write(GPIO_PIN_SET);

	return val;
}

void TM_MFRC522_SetBitMask(uint8_t reg, uint8_t mask) {
	TM_MFRC522_WriteRegister(reg, TM_MFRC522_ReadRegister(reg) | mask);
}

void TM_MFRC522_ClearBitMask(uint8_t reg, uint8_t mask){
	TM_MFRC522_WriteRegister(reg, TM_MFRC522_ReadRegister(reg) & (~mask));
}

void TM_MFRC522_AntennaOn(void) {
	uint8_t temp;

	temp = TM_MFRC522_ReadRegister(MFRC522_REG_TX_CONTROL);
	if (!(temp & 0x03)) {
		TM_MFRC522_SetBitMask(MFRC522_REG_TX_CONTROL, 0x03);
	}
}

void TM_MFRC522_AntennaOff(void) {
	TM_MFRC522_ClearBitMask(MFRC522_REG_TX_CONTROL, 0x03);
}

void TM_MFRC522_Reset(void) {
	TM_MFRC522_WriteRegister(MFRC522_REG_COMMAND, PCD_RESETPHASE);
	HAL_Delay(50);
}

TM_MFRC522_Status_t TM_MFRC522_Request(uint8_t reqMode, uint8_t* TagType) {
	TM_MFRC522_Status_t status;
	uint16_t backBits;			//The received data bits

	TM_MFRC522_WriteRegister(MFRC522_REG_BIT_FRAMING, 0x07);		//TxLastBists = BitFramingReg[2..0]	???

	TagType[0] = reqMode;
	status = TM_MFRC522_ToCard(PCD_TRANSCEIVE, TagType, 1, TagType, &backBits);

	if (status == MI_OK && backBits != 0x10) {
		status = MI_ERR;
	}
	return status;
}

TM_MFRC522_Status_t TM_MFRC522_ToCard(uint8_t command, // the command to execute - one of the PCD_Command enums
										uint8_t* sendData, // pointer to the data to transfer to the FIFO
										uint8_t sendLen, // number of bytes to transfer to the FIFO
										uint8_t* backData, // NULL or pointer to buffer if data should be read back after executing the command
										uint16_t* backLen // in: max number of bytes to write to *backData, out: the number of bytes returned
									) {
	TM_MFRC522_Status_t status = MI_ERR;
	uint8_t irqEn = 0x00;
	uint8_t waitIRq = 0x00;
	uint8_t lastBits;
	uint8_t n;
	uint16_t i;

	switch (command) {
		case PCD_AUTHENT: {
			irqEn = 0x12;
			waitIRq = 0x10; // bit 4
			break;
		}
		case PCD_TRANSCEIVE: {
			irqEn = 0x77; //
			waitIRq = 0x30; // bit 4 IdleIRq, 5 RxIRq
			break;
		}
		default:
			break;
	}

	TM_MFRC522_WriteRegister(MFRC522_REG_COMM_IE_N, irqEn | 0x80);

	TM_MFRC522_WriteRegister(MFRC522_REG_COMMAND, PCD_IDLE); // Stop any active command.

	TM_MFRC522_ClearBitMask(MFRC522_REG_COLL, 0x80); // clear collision register

	//TM_MFRC522_ClearBitMask(MFRC522_REG_COMM_IRQ, 0x80); // Clear all seven interrupt request bits
	TM_MFRC522_WriteRegister(MFRC522_REG_COMM_IRQ, 0x7F); // Clear all seven interrupt request bits via ComIrqReg[7] - Set1, when 0, clear interrupts
	TM_MFRC522_SetBitMask(MFRC522_REG_FIFO_LEVEL, 0x80); // FlushBuffer = 1, FIFO initialization
	//TM_MFRC522_WriteRegister(MFRC522_REG_BIT_FRAMING, 0x00); // make sure to clear bit adjustments (should be calculated though, missing some parameters)

	//Writing data to the FIFO
	for (i = 0; i < sendLen; i++) {
		TM_MFRC522_WriteRegister(MFRC522_REG_FIFO_DATA, sendData[i]);
	}

	//Execute the command
	TM_MFRC522_WriteRegister(MFRC522_REG_COMMAND, command);
	if (command == PCD_TRANSCEIVE) {
		TM_MFRC522_SetBitMask(MFRC522_REG_BIT_FRAMING, 0x80);		//StartSend=1,transmission of data starts
	}

	//Waiting to receive data to complete
	i = 36000;	//i according to the clock frequency adjustment, the operator M1 card maximum waiting time 25ms???
	do {
		//CommIrqReg[7..0]
		//Set1 TxIRq RxIRq IdleIRq HiAlerIRq LoAlertIRq ErrIRq TimerIRq
		n = TM_MFRC522_ReadRegister(MFRC522_REG_COMM_IRQ);
		i--;
	} while ((i!=0) // i=0 is timeout
				&& !(n&0x01) // timer interrupt - nothing received in 25ms
				&& !(n&waitIRq) // one of the interrupts that signal success has been sent
			);

	TM_MFRC522_ClearBitMask(MFRC522_REG_BIT_FRAMING, 0x80);			//StartSend=0

	uint8_t errorRegValue = 0x00;
	errorRegValue = TM_MFRC522_ReadRegister(MFRC522_REG_ERROR);
	if (errorRegValue & 0x13) {	 // BufferOvfl ParityErr ProtocolErr	
		//LCD_UsrLog ((char *)"We have an error.\n");

		status = MI_ERR;
		return status;
	}

	if (i == 0) {
		//LCD_UsrLog ((char *)"I went to zero.\n");
		return MI_TIMEOUT;
	}

	if (n & 0x01 && !(n&waitIRq)) {
		//char inty[15];
		//sprintf(inty, "%d", i);
		//LCD_UsrLog ((char *)"Timer timeouted\n");
		//LCD_UsrLog (inty);
		//LCD_UsrLog ((char *)"\n");

		return MI_TIMEOUT;
	}

	if (n&waitIRq) {
		//LCD_UsrLog ((char *)"Something was transmitted by the rc522.\n");
	}

	if (i != 0)  {
		if (!(TM_MFRC522_ReadRegister(MFRC522_REG_ERROR) & 0x1B)) {
			//LCD_UsrLog ((char *)"No errors and i != 0..\n");

			status = MI_OK;
			// if (n & irqEn & 0x01) {
			// 	status = MI_NOTAGERR;
			// }

			if (command == PCD_TRANSCEIVE) {
				n = TM_MFRC522_ReadRegister(MFRC522_REG_FIFO_LEVEL);
				lastBits = TM_MFRC522_ReadRegister(MFRC522_REG_CONTROL) & 0x07;

				if (n == 0) {
					n = 1;
				}

				if (lastBits) {
					*backLen = (n - 1) * 8 + lastBits;
				} else {
					*backLen = n * 8;
				}

				//char inty[15];
				//LCD_UsrLog ((char *)"\n");
				//sprintf(inty, "%d", *backLen);
				//LCD_UsrLog (inty);
				//LCD_UsrLog ((char *)"Back length\n");

				if (n > MFRC522_MAX_LEN) {
					n = MFRC522_MAX_LEN;
				}

				//Reading the received data in FIFO
				for (i = 0; i < n; i++) {
					backData[i] = TM_MFRC522_ReadRegister(MFRC522_REG_FIFO_DATA);
				}
			}
		} else {
		}
	} else {

	}

	if (errorRegValue & 0x08) {		// CollErr
		//LCD_UsrLog ((char *)"We have a colision error.\n");

		return MI_ERR;
	}

	return status;
}

TM_MFRC522_Status_t TM_MFRC522_Anticoll(uint8_t* serNum) {
	TM_MFRC522_Status_t status;
	uint8_t i;
	uint8_t serNumCheck = 0;
	uint16_t unLen;

	TM_MFRC522_WriteRegister(MFRC522_REG_BIT_FRAMING, 0x00);		//TxLastBists = BitFramingReg[2..0]

	serNum[0] = PICC_ANTICOLL;
	serNum[1] = 0x20;
	status = TM_MFRC522_ToCard(PCD_TRANSCEIVE, serNum, 2, serNum, &unLen);

	if (status == MI_OK) {
		//Check card serial number
		for (i = 0; i < 4; i++) {
			serNumCheck ^= serNum[i];
		}
		if (serNumCheck != serNum[i]) {
			status = MI_ERR;
		}
	}
	return status;
}

void TM_MFRC522_CalculateCRC(uint8_t*  pIndata, uint8_t len, uint8_t* pOutData) {
	uint8_t i, n;

	TM_MFRC522_ClearBitMask(MFRC522_REG_DIV_IRQ, 0x04);			//CRCIrq = 0
	TM_MFRC522_SetBitMask(MFRC522_REG_FIFO_LEVEL, 0x80);			//Clear the FIFO pointer
	//Write_MFRC522(CommandReg, PCD_IDLE);

	//Writing data to the FIFO
	for (i = 0; i < len; i++) {
		TM_MFRC522_WriteRegister(MFRC522_REG_FIFO_DATA, *(pIndata+i));
	}
	TM_MFRC522_WriteRegister(MFRC522_REG_COMMAND, PCD_CALCCRC);

	//Wait CRC calculation is complete
	i = 0xFF;
	do {
		n = TM_MFRC522_ReadRegister(MFRC522_REG_DIV_IRQ);
		i--;
	} while ((i!=0) && !(n&0x04));			//CRCIrq = 1

	//Read CRC calculation result
	pOutData[0] = TM_MFRC522_ReadRegister(MFRC522_REG_CRC_RESULT_L);
	pOutData[1] = TM_MFRC522_ReadRegister(MFRC522_REG_CRC_RESULT_M);
}

uint8_t TM_MFRC522_SelectTag(uint8_t* serNum) {
	uint8_t i;
	TM_MFRC522_Status_t status;
	uint8_t size;
	uint16_t recvBits;
	uint8_t buffer[9];

	buffer[0] = PICC_SElECTTAG;
	buffer[1] = 0x70;
	for (i = 0; i < 5; i++) {
		buffer[i+2] = *(serNum+i);
	}
	TM_MFRC522_CalculateCRC(buffer, 7, &buffer[7]);		//??
	status = TM_MFRC522_ToCard(PCD_TRANSCEIVE, buffer, 9, buffer, &recvBits);

	if ((status == MI_OK) && (recvBits == 0x18)) {
		size = buffer[0];
	} else {
		size = 0;
	}

	return size;
}

void TM_MFRC522_Halt(void) {
	uint16_t unLen;
	uint8_t buff[4];

	buff[0] = PICC_HALT;
	buff[1] = 0;
	TM_MFRC522_CalculateCRC(buff, 2, &buff[2]);

	TM_MFRC522_ToCard(PCD_TRANSCEIVE, buff, 4, buff, &unLen);
}

void TM_MFRC522_CS_Write(uint8_t val) {
	HAL_GPIO_WritePin(SPIx_CS_GPIO_PORT, SPIx_CS_PIN, val);
}

void bin_to_strhex(unsigned char *bin, unsigned int binsz, char **result)
{
  char          hex_str[]= "0123456789abcdef";
  unsigned int  i;

  *result = (char *)malloc(binsz * 2 + 1);
  (*result)[binsz * 2] = 0;

  if (!binsz)
    return;

  for (i = 0; i < binsz; i++)
    {
      (*result)[i * 2 + 0] = hex_str[(bin[i] >> 4) & 0x0F];
      (*result)[i * 2 + 1] = hex_str[(bin[i]     ) & 0x0F];
    }  
}
