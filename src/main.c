#include "uart.h"
#include "printf.h"
#include "MFRC522.h"
#include <stdint.h>

void print_sys_info()
{
    uint32_t sysclk = HAL_RCC_GetSysClockFreq();
    uint32_t hclk   = HAL_RCC_GetHCLKFreq();
    uint32_t pclk1  = HAL_RCC_GetPCLK1Freq();
    uint32_t pclk2  = HAL_RCC_GetPCLK2Freq();

    printf("SYSCLK: %lu Hz\r\n", sysclk);
    printf("HCLK:   %lu Hz\r\n", hclk);
    printf("PCLK1:  %lu Hz\r\n", pclk1);
    printf("PCLK2:  %lu Hz\r\n", pclk2);
}

int main(void){
	//print_sys_info();
	UART_config();
	uint8_t read_val = 0;
	uint16_t reg_read_ret_val;
	uint16_t reg_write_ret_val;
	MFRC522_t MFRC522;
	MFRC522_init(&MFRC522);

	reg_read_ret_val = MFRC522_read_reg(&MFRC522, CommandReg);
	if(reg_read_ret_val == MFRC522_OK)
		printf("CommandReg val: %x\n", MFRC522_get_rx_buf(&MFRC522));
	else
		printf("MFRC522 read reg failed with val: %i\n", MFRC522.error);
	
	reg_write_ret_val = MFRC522_write_reg(&MFRC522, CommandReg, CalcCRC );
	
	if(reg_write_ret_val == MFRC522_OK)
		printf("CommandReg val: %x\n", MFRC522_get_rx_buf(&MFRC522));
	else
		printf("MFRC522 read reg failed with val: %i\n", MFRC522.error);

	reg_read_ret_val = MFRC522_read_reg(&MFRC522, CommandReg);
	if(reg_read_ret_val == MFRC522_OK)
		printf("CommandReg val: %x\n", MFRC522_get_rx_buf(&MFRC522));
	else
		printf("MFRC522 read reg failed with val: %i\n", MFRC522.error);
	


/*
	MFRC522_write_reg(&MFRC522, FIFODataReg, 0xA6);
	MFRC522_write_reg(&MFRC522, FIFODataReg, 0x79);
	read_val = MFRC522_read_reg(&MFRC522,FIFODataReg);
	printf("Received val = %X\n", read_val);
	
	if((!read_val) && (MFRC522.status == MFRC522_ERROR) ){
		printf("MFRC522 module is uninitialized\n");
	}
	else{
		printf("Received val = %X\n", read_val);
	}
*/

	MFRC522_deinit(&MFRC522);
	while(1);

	return 0;
}
