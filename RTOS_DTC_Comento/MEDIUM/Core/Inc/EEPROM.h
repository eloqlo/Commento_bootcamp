/*
 * EEPROM.h
 *
 *  Created on: Aug 13, 2025
 *      Author: jh
 */

#ifndef INC_EEPROM_H_
#define INC_EEPROM_H_

#include "stm32f4xx_hal.h"
#include "main.h"

//typedef struct {
//	unsigned int DTC_Code;
//	char Description[50];
//	unsigned char active;
//} DTC_Table_t;

void EEPROM_ReadDTC_DMA(DTC_Table_t DTC_Table, uint16_t mem_addr);
void EEPROM_WriteEnable(void);
void EEPROM_WriteDTC_DMA(DTC_Table_t DTC_Table, uint16_t mem_addr);

// SPI EEPROM 명령어
#define EEPROM_CMD_READ  0x03	// read data from memory array beginning at selected address
#define EEPROM_CMD_WRITE 0x02	// write data to memory array beginning at selected address
#define EEPROM_CMD_WRDI  0x04 	// disable write operations
#define EEPROM_CMD_WREN  0x06	// enable write operation
#define EEPROM_DTC_ADDR  0x0000	// DTC 주소

extern SPI_HandleTypeDef hspi1;

extern DTC_Table_t DTC_Table_UV_A;
extern DTC_Table_t DTC_Table_OV_A;
extern DTC_Table_t DTC_Table_OC_A;
extern DTC_Table_t DTC_Table_UV_B;
extern DTC_Table_t DTC_Table_OV_B;
extern DTC_Table_t DTC_Table_OC_B;
extern DTC_Table_t DTC_Table_UV_C;
extern DTC_Table_t DTC_Table_OV_C;
extern DTC_Table_t DTC_Table_OC_C;
extern DTC_Table_t DTC_Table_UV_D;
extern DTC_Table_t DTC_Table_OV_D;
extern DTC_Table_t DTC_Table_OC_D;
extern DTC_Table_t DTC_Table_TEMP;

extern unsigned char can_transmit_dtc_flag;


#endif /* INC_EEPROM_H_ */
