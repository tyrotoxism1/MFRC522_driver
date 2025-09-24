#include "stm32f4xx_hal.h"
#include "uart.h"
#include "printf.h"
#include "MFRC522.h"
#include <stdint.h>

void read_test(MFRC522_t *MFRC522)
{
	uint16_t reg_read_ret_val;

	reg_read_ret_val = MFRC522_read_reg(MFRC522, CommandReg);
	if(reg_read_ret_val == MFRC522_OK)
		printf("CommandReg val: %x\n", MFRC522_get_rx_buf(MFRC522));
	else
		printf("MFRC522 read reg failed with val: %i\n", MFRC522->error);
}

void write_test(MFRC522_t *MFRC522)
{
	uint16_t reg_write_ret_val;
	
	reg_write_ret_val = MFRC522_write_reg(MFRC522, CommandReg, CalcCRC );
	
	if(reg_write_ret_val == MFRC522_OK)
		printf("CommandReg val: %x\n", MFRC522_get_rx_buf(MFRC522));
	else
		printf("MFRC522 read reg failed with val: %i\n", MFRC522->error);


}

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
	uint8_t CL1_buf[5] = {0};
	uint8_t SEL_buf[9] = {0};
	uint8_t default_key[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF ,0xFF};

	MFRC522_t MFRC522;
	MFRC522_init(&MFRC522);
	MFRC522_REQA(&MFRC522);
	MFRC522_CL1(&MFRC522, CL1_buf);
	MFRC522_SEL( &MFRC522, CL1_buf);
	MFRC522_auth_PICC(&MFRC522, 0x06, default_key, CL1_buf);	
	MFRC522_read_PICC(&MFRC522, 0x07);
	MFRC522_stop_encrypt_comm(&MFRC522);

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
