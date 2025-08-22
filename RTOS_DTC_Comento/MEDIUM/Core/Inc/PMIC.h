/*
 * PMIC.h
 *
 *  Created on: Aug 13, 2025
 *      Author: jh
 */

#ifndef INC_PMIC_H_
#define INC_PMIC_H_

#include "stm32f4xx_hal.h"
#include "main.h"

// PMIC Addresses & Registers
#define PMIC_DEV_ADDR (0x60 << 1)		// PMIC I2C PHY 주소
#define PMIC_SYSEN_REG 0x40
#define PMIC_FAULT_STATUS1_REG 0x07		// UV, OV
#define PMIC_FAULT_STATUS2_REG 0x08		// OC
#define PMIC_FAULT_STATUS3_REG 0x09		// Thermal shutdown
#define PMIC_BUCKA_REG 0x13
#define PMIC_BUCKB_REG 0x1B
#define PMIC_BUCKC_REG 0x23
#define PMIC_BUCKD_REG 0x2B

#define EEPROM_DTC_ADDR_UV_A 0x0000
#define EEPROM_DTC_ADDR_OV_A ((uint16_t)(0x0000 + 55*1))
#define EEPROM_DTC_ADDR_OC_A ((uint16_t)(0x0000 + 55*2))
#define EEPROM_DTC_ADDR_UV_B ((uint16_t)(0x0000 + 55*3))
#define EEPROM_DTC_ADDR_OV_B ((uint16_t)(0x0000 + 55*4))
#define EEPROM_DTC_ADDR_OC_B ((uint16_t)(0x0000 + 55*5))
#define EEPROM_DTC_ADDR_UV_C ((uint16_t)(0x0000 + 55*6))
#define EEPROM_DTC_ADDR_OV_C ((uint16_t)(0x0000 + 55*7))
#define EEPROM_DTC_ADDR_OC_C ((uint16_t)(0x0000 + 55*8))
#define EEPROM_DTC_ADDR_UV_D ((uint16_t)(0x0000 + 55*9))
#define EEPROM_DTC_ADDR_OV_D ((uint16_t)(0x0000 + 55*10))
#define EEPROM_DTC_ADDR_OC_D ((uint16_t)(0x0000 + 55*11))
#define EEPROM_DTC_ADDR_TEMP ((uint16_t)(0x0000 + 55*12))


extern I2C_HandleTypeDef hi2c1;
extern DMA_HandleTypeDef hdma_i2c1_rx;
extern DMA_HandleTypeDef hdma_i2c1_tx;

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

extern unsigned char eeprom_read_dtc_flag;


enum PMIC_OV_BITS {BUCKD_OV, BUCKC_OV, BUCKB_OV, BUCKA_OV};
enum PMIC_UV_BITS {BUCKD_UV=4, BUCKC_UV, BUCKB_UV, BUCKA_UV};
enum PMIC_OC_BITS {BUCKD_OC=4, BUCKC_OC, BUCKB_OC, BUCKA_OC};
enum PMIC_TEMP_BIT {PMIC_HIGH_TEMP_SHUTDOWN};
enum PMIC_SYS_BITS {
	SYSEN=7,
	ENA=5,
	ENB=4,
	ENC=3,
	END=2,
	VOUT_SELECTx=7,
	VREFx_HIGH_0=0,
	VREFx_HIGH_1=1,
	VREFx_LOW_0=0,
	VREFx_LOW_1=1,
	VREFx_LOW_2=2,
	VREFx_LOW_3=3,
	VREFx_LOW_4=4,
	VREFx_LOW_5=5,
	VREFx_LOW_6=6,
	VREFx_LOW_7=7,
};
enum PMIC_SYS_DEFAULT_VALUES {
	ENA_default = 1,
	ENB_default = 1,
	ENC_default = 1,
	END_default=1,
	V_SELECTA_default=0,
	V_SELECTB_default=0,
	V_SELECTC_default=0,
	V_SELECTD_default=1,
	VrefA_default = 0b1000100110,
	VrefB_default = 0b1000100110,
	VrefC_default = 0b1000100110,
	VrefD_default = 0b0111000010
};


typedef union {
	struct{
		unsigned char bit0 : 1;
		unsigned char bit1 : 1;
		unsigned char bit2 : 1;
		unsigned char bit3 : 1;
		unsigned char bit4 : 1;
		unsigned char bit5 : 1;
		unsigned char bit6 : 1;
		unsigned char bit7 : 1;
	};
	unsigned char data;
}Data;

typedef struct{
	unsigned char BUCK_USE_A;
	unsigned char BUCK_USE_B;
	unsigned char BUCK_USE_C;
	unsigned char BUCK_USE_D;

	unsigned char VOUT_select_A;
	unsigned char VOUT_select_B;
	unsigned char VOUT_select_C;
	unsigned char VOUT_select_D;

	unsigned short VREF_A;
	unsigned short VREF_B;
	unsigned short VREF_C;
	unsigned short VREF_D;
}PMIC_Handler;

typedef struct{
	unsigned char UV;
	unsigned char OV;
	unsigned char OC;
	unsigned char TEMP;
}PMIC_Buckx_Data;



// 프로토타입 선언
void PMIC_PowerOn(void);
void PMIC_BUCKx_Vout(unsigned char PMIC_BUCKx_REG, unsigned char VOUT_select, unsigned short VREF);
void PMIC_Init(void);
void PMIC_ReadUVOV(void);
void PMIC_ReadOC(void);
void PMIC_ReadTEMP(void);
void PMIC_DTC_Update(void);

#endif /* INC_PMIC_H_ */
