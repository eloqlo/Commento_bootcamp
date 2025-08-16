/*
 * PMIC.c
 *
 *  Created on: Aug 13, 2025
 *      Author: jh
 *
 *      MP5475
 */

#include "PMIC.h"
#include "EEPROM.h"
#include "main.h"

//////////////////////////////////////////////////////////////////////////////////////////////////
// PMIC INITIALIZATION

// Datasheet 를 참조한 Default Setting
PMIC_Handler pmic_handler = {
		1,1,1,1,	// ENABLE A, B, C, D
		0,0,0,1, 	// V_SELECT A, B, C, D
		0b1000100110,	// Vref A
		0b1000100110,	// Vref B
		0b1000100110,	// Vref C
		0b0111000010	// Vref D
};


/* PMIC의 POWER ON(SYSEN 레지스터 set / Buck A,B,C,D 중 쓸 Buck들 켜기) */
void PMIC_PowerOn(void){
	uint8_t* pmic_data;

	// 기존 PMIC register data 읽어오기 -> RESERVED bit들 때문에
	if(HAL_I2C_Mem_Read(&hi2c1, PMIC_DEV_ADDR, PMIC_SYSEN_REG, I2C_MEMADD_SIZE_8BIT, pmic_data, 1,100)){
		Error_Handler();
	}

	// SYSEN 레지스터 set, Enable ABCD 레지스터 set
	//TODO 이 부분 그림으로 그리기
	*pmic_data = (0x01 << SYSEN) |
			(pmic_handler.BUCK_USE_A << ENA) |
			(pmic_handler.BUCK_USE_B << ENB) |
			(pmic_handler.BUCK_USE_C << ENC) |
			(pmic_handler.BUCK_USE_D << END) |
			*pmic_data;

	// PMIC System 설정 register data write
	if(HAL_I2C_Mem_Write(&hi2c1, PMIC_DEV_ADDR, PMIC_SYSEN_REG, I2C_MEMADD_SIZE_8BIT, pmic_data, 1, 100)){
		Error_Handler();
	}
}

/* BuckA,B,C,D의 출력전압 설정*/
void PMIC_BUCKx_Vout(uint8_t PMIC_BUCKx_REG, uint8_t VOUT_select, uint16_t VREF){
	Data pmic_data;
	if(HAL_I2C_Mem_Read(&hi2c1, PMIC_DEV_ADDR, PMIC_SYSEN_REG, I2C_MEMADD_SIZE_8BIT, &pmic_data, 1, 100)){
		Error_Handler();
	}

	// VOUT_SELECTx 레지스터 설정 (전압 증가 step 설정)
	pmic_data.bit0 = VOUT_select;

	// V_REFx_HIGH 2bits	(기준 전압 Vref 설정: 300mV ~ 2.048V)
	pmic_data.bit6 = (VREF >> 9);
	pmic_data.bit7 = (VREF >> 9) << 1;
	if(HAL_I2C_Mem_Write(&hi2c1, PMIC_DEV_ADDR, PMIC_BUCKx_REG, I2C_MEMADD_SIZE_8BIT, &pmic_data, 1, 100)){
		Error_Handler();
	}
	// V_REFx_LOW 8bits
	pmic_data.data = VREF;
	if(HAL_I2C_Mem_Write(&hi2c1, PMIC_DEV_ADDR, PMIC_BUCKx_REG, I2C_MEMADD_SIZE_8BIT, &pmic_data, 1, 100)){
		Error_Handler();
	}
}

/* < PMIC Initialization 함수 >
 * 1. PMIC POWER ON
 * 2. 사용할 BUCK ENABLE
 * 3. BUCK 별 출력전압 설정
 *  */
void PMIC_Init(void){

	// POWER ON + ENABLE BUCKs
	PMIC_PowerOn();

	// Buck Regulator A,B,C,D Voltage Level Control
	if (pmic_handler.BUCK_USE_A == 1){
		PMIC_BUCKx_Vout(PMIC_BUCKA_REG, pmic_handler.VOUT_select_A, pmic_handler.VREF_A);
	}
	if (pmic_handler.BUCK_USE_B == 1){
		PMIC_BUCKx_Vout(PMIC_BUCKB_REG, pmic_handler.VOUT_select_B, pmic_handler.VREF_B);
	}
	if (pmic_handler.BUCK_USE_C == 1){
		PMIC_BUCKx_Vout(PMIC_BUCKC_REG, pmic_handler.VOUT_select_C, pmic_handler.VREF_C);
	}
	if (pmic_handler.BUCK_USE_D == 1){
		PMIC_BUCKx_Vout(PMIC_BUCKD_REG, pmic_handler.VOUT_select_D, pmic_handler.VREF_D);
	}
}

//////////////////////////////////////////////////////////////////////////////////////////////////
// PMIC Fault Data 처리
Data fault_data;

uint8_t UVOV_flag = 0;
void PMIC_ReadUVOV(void){
	uint8_t UVOV_flag = 1;
	HAL_I2C_Mem_Read_DMA(&hi2c1, PMIC_DEV_ADDR, PMIC_FAULT_STATUS1_REG, I2C_MEMADD_SIZE_8BIT, &fault_data.data, 1);
}

uint8_t OC_flag = 0;
void PMIC_ReadOC(void){
	uint8_t OC_flag = 1;
	HAL_I2C_Mem_Read_DMA(&hi2c1, PMIC_DEV_ADDR, PMIC_FAULT_STATUS2_REG, I2C_MEMADD_SIZE_8BIT, &fault_data.data, 1);
}

uint8_t TEMP_flag = 0;
void PMIC_ReadTEMP(void){
	uint8_t TEMP_flag = 1;
	HAL_I2C_Mem_Read_DMA(&hi2c1, PMIC_DEV_ADDR, PMIC_FAULT_STATUS3_REG, I2C_MEMADD_SIZE_8BIT, &fault_data.data, 1);
}


PMIC_Buckx_Data pmic_A_data = {0,0,0,0};
PMIC_Buckx_Data pmic_B_data = {0,0,0,0};
PMIC_Buckx_Data pmic_C_data = {0,0,0,0};
PMIC_Buckx_Data pmic_D_data = {0,0,0,0};

void PMIC_DTC_Update(void){
	// Update Temperature DTC
	if ((DTC_Table_TEMP.active ^ pmic_A_data.TEMP) == 1){		// 기존 DTC와 현재 register 값이 다르면, DTC update
		if (DTC_Table_TEMP.active==0) {DTC_Table_TEMP.active = 1;}
		if (DTC_Table_TEMP.active==1) {DTC_Table_TEMP.active = 0;}
		// Write DTC to EEPROM
		EEPROM_WriteDTC_DMA(DTC_Table_TEMP, EEPROM_DTC_ADDR_TEMP);
	}
	// Buck A
	if (pmic_handler.BUCK_USE_A){
		if ((DTC_Table_UV_A.active ^ pmic_A_data.UV) == 1){
			if (DTC_Table_UV_A.active==0) {DTC_Table_UV_A.active = 1;}
			if (DTC_Table_UV_A.active==1) {DTC_Table_UV_A.active = 0;}
			// Write DTC to EEPROM
			EEPROM_WriteDTC_DMA(DTC_Table_UV_A, EEPROM_DTC_ADDR_UV_A);
		}
		if ((DTC_Table_OV_A.active ^ pmic_A_data.OV) == 1){
			if (DTC_Table_OV_A.active==0) {DTC_Table_OV_A.active = 1;}
			if (DTC_Table_OV_A.active==1) {DTC_Table_OV_A.active = 0;}
			// Write DTC to EEPROM
			EEPROM_WriteDTC_DMA(DTC_Table_OV_A, EEPROM_DTC_ADDR_OV_A);
		}
		if ((DTC_Table_OC_A.active ^ pmic_A_data.OC) == 1){
			if (DTC_Table_OC_A.active==0) {DTC_Table_OC_A.active = 1;}
			if (DTC_Table_OC_A.active==1) {DTC_Table_OC_A.active = 0;}
			// Write DTC to EEPROM
			EEPROM_WriteDTC_DMA(DTC_Table_OC_A, EEPROM_DTC_ADDR_OC_A);
		}
	}
	//Buck B
	if (pmic_handler.BUCK_USE_B){
		if ((DTC_Table_UV_B.active ^ pmic_B_data.UV) == 1){
			if (DTC_Table_UV_B.active==0) {DTC_Table_UV_B.active = 1;}
			if (DTC_Table_UV_B.active==1) {DTC_Table_UV_B.active = 0;}
			// Write DTC to EEPROM
			EEPROM_WriteDTC_DMA(DTC_Table_UV_B, EEPROM_DTC_ADDR_UV_B);
		}
		if ((DTC_Table_OV_B.active ^ pmic_B_data.OV) == 1){
			if (DTC_Table_OV_B.active==0) {DTC_Table_OV_B.active = 1;}
			if (DTC_Table_OV_B.active==1) {DTC_Table_OV_B.active = 0;}
			// Write DTC to EEPROM
			EEPROM_WriteDTC_DMA(DTC_Table_OV_B, EEPROM_DTC_ADDR_OV_B);
		}
		if ((DTC_Table_OC_B.active ^ pmic_B_data.OC) == 1){
			if (DTC_Table_OC_B.active==0) {DTC_Table_OC_B.active = 1;}
			if (DTC_Table_OC_B.active==1) {DTC_Table_OC_B.active = 0;}
			// Write DTC to EEPROM
			EEPROM_WriteDTC_DMA(DTC_Table_OC_B, EEPROM_DTC_ADDR_OC_B);
		}
	}
	// Buck C
	if (pmic_handler.BUCK_USE_C){
		if ((DTC_Table_UV_C.active ^ pmic_C_data.UV) == 1){
			if (DTC_Table_UV_C.active==0) {DTC_Table_UV_C.active = 1;}
			if (DTC_Table_UV_C.active==1) {DTC_Table_UV_C.active = 0;}
			// Write DTC to EEPROM
			EEPROM_WriteDTC_DMA(DTC_Table_UV_C, EEPROM_DTC_ADDR_UV_C);
		}
		if ((DTC_Table_OV_C.active ^ pmic_C_data.OV) == 1){
			if (DTC_Table_OV_C.active==0) {DTC_Table_OV_C.active = 1;}
			if (DTC_Table_OV_C.active==1) {DTC_Table_OV_C.active = 0;}
			// Write DTC to EEPROM
			EEPROM_WriteDTC_DMA(DTC_Table_OV_C, EEPROM_DTC_ADDR_OV_C);
		}
		if ((DTC_Table_OC_C.active ^ pmic_C_data.OC) == 1){
			if (DTC_Table_OC_C.active==0) {DTC_Table_OC_C.active = 1;}
			if (DTC_Table_OC_C.active==1) {DTC_Table_OC_C.active = 0;}
			// Write DTC to EEPROM
			EEPROM_WriteDTC_DMA(DTC_Table_OC_C, EEPROM_DTC_ADDR_OC_C);
		}
	}

	// Buck D
	if (pmic_handler.BUCK_USE_D){
		if ((DTC_Table_UV_D.active ^ pmic_D_data.UV) == 1){
			if (DTC_Table_UV_D.active==0) {DTC_Table_UV_D.active = 1;}
			if (DTC_Table_UV_D.active==1) {DTC_Table_UV_D.active = 0;}
			// Write DTC to EEPROM
			EEPROM_WriteDTC_DMA(DTC_Table_UV_D, EEPROM_DTC_ADDR_UV_D);
		}
		if ((DTC_Table_OV_D.active ^ pmic_D_data.OV) == 1){
			if (DTC_Table_OV_D.active==0) {DTC_Table_OV_D.active = 1;}
			if (DTC_Table_OV_D.active==1) {DTC_Table_OV_D.active = 0;}
			// Write DTC to EEPROM
			EEPROM_WriteDTC_DMA(DTC_Table_OV_D, EEPROM_DTC_ADDR_OV_D);
		}
		if ((DTC_Table_OC_D.active ^ pmic_D_data.OC) == 1){
			if (DTC_Table_OC_D.active==0) {DTC_Table_OC_D.active = 1;}
			if (DTC_Table_OC_D.active==1) {DTC_Table_OC_D.active = 0;}
			// Write DTC to EEPROM
			EEPROM_WriteDTC_DMA(DTC_Table_OC_D, EEPROM_DTC_ADDR_OC_D);
		}
	}

	eeprom_read_dtc_flag = 1;	// 다음 STEP으로 이동.
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	if (UVOV_flag){
		UVOV_flag=0;
		if (pmic_handler.BUCK_USE_A){
			pmic_A_data.UV = (fault_data.data >> BUCKA_UV) & 0x01;
			pmic_A_data.OV = (fault_data.data >> BUCKA_OV) & 0x01;
		}
		if (pmic_handler.BUCK_USE_B){
			pmic_B_data.UV = (fault_data.data >> BUCKB_UV) & 0x01;
			pmic_B_data.OV = (fault_data.data >> BUCKB_OV) & 0x01;
		}
		if (pmic_handler.BUCK_USE_C){
			pmic_C_data.UV = (fault_data.data >> BUCKC_UV) & 0x01;
			pmic_C_data.OV = (fault_data.data >> BUCKC_OV) & 0x01;
		}
		if (pmic_handler.BUCK_USE_A){
			pmic_D_data.UV = (fault_data.data >> BUCKD_UV) & 0x01;
			pmic_D_data.OV = (fault_data.data >> BUCKD_OV) & 0x01;
		}
	}
	else if (OC_flag){
		OC_flag=0;
		if (pmic_handler.BUCK_USE_A) {pmic_A_data.OC = (fault_data.data >> BUCKA_OC) & 0x01;}
		if (pmic_handler.BUCK_USE_B) {pmic_B_data.OC = (fault_data.data >> BUCKB_OC) & 0x01;}
		if (pmic_handler.BUCK_USE_C) {pmic_C_data.OC = (fault_data.data >> BUCKC_OC) & 0x01;}
		if (pmic_handler.BUCK_USE_D) {pmic_D_data.OC = (fault_data.data >> BUCKD_OC) & 0x01;}
	}
	else if (TEMP_flag){
		TEMP_flag=0;
		if (pmic_handler.BUCK_USE_A) {pmic_A_data.TEMP = (fault_data.data >> PMIC_HIGH_TEMP_SHUTDOWN) & 0x01;}
		if (pmic_handler.BUCK_USE_B) {pmic_B_data.TEMP = (fault_data.data >> PMIC_HIGH_TEMP_SHUTDOWN) & 0x01;}
		if (pmic_handler.BUCK_USE_C) {pmic_C_data.TEMP = (fault_data.data >> PMIC_HIGH_TEMP_SHUTDOWN) & 0x01;}
		if (pmic_handler.BUCK_USE_D) {pmic_D_data.TEMP = (fault_data.data >> PMIC_HIGH_TEMP_SHUTDOWN) & 0x01;}
	}
}


