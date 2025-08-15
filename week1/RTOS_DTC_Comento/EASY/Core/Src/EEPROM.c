/*
 * EEPROM.c
 *
 *  Created on: Aug 13, 2025
 *      Author: jh
 */

#include "EEPROM.h"
#include "main.h"

void EEPROM_ReadDTC_DMA(DTC_Table_t DTC_Table, uint16_t mem_addr) {
  uint8_t cmd[3];
  cmd[0] = EEPROM_CMD_READ;
  cmd[1] = (mem_addr >> 8) & 0xFF;
  cmd[2] = mem_addr & 0xFF;
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, cmd, 3, HAL_MAX_DELAY);
  HAL_SPI_Receive_DMA(&hspi1, (uint8_t*)&DTC_Table, sizeof(DTC_Table));
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
}

void EEPROM_WriteEnable(void) {
  uint8_t cmd = EEPROM_CMD_WREN;
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
}

void EEPROM_WriteDTC_DMA(DTC_Table_t DTC_Table, uint16_t mem_addr) {
  uint8_t cmd[3];
  EEPROM_WriteEnable();
  cmd[0] = EEPROM_CMD_WRITE;
  cmd[1] = (mem_addr >> 8) & 0xFF;
  cmd[2] = mem_addr & 0xFF;
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, cmd, 3, HAL_MAX_DELAY);
  HAL_SPI_Transmit_DMA(&hspi1, (uint8_t*)&DTC_Table, sizeof(DTC_Table));
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
}

int counter = 0;
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
	counter++;
	if (counter==13){
		can_transmit_dtc_flag = 1;
		counter = 0;
	}
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{

}
