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
		printf("CommandReg val: %x\n", MFRC522->Rx_buf);
	else
		printf("MFRC522 read reg failed with val: %i\n", MFRC522->error);
}

void write_test(MFRC522_t *MFRC522)
{
	uint16_t reg_write_ret_val;

	reg_write_ret_val = MFRC522_write_reg(MFRC522, CommandReg, CalcCRC );

	if(reg_write_ret_val == MFRC522_OK)
		printf("CommandReg val: %x\n", MFRC522->Rx_buf);
	else
		printf("MFRC522 read reg failed with val: %i\n", MFRC522->error);


}

void test_block_read_and_print(MFRC522_t *MFRC522)
{
	uint8_t default_key[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF ,0xFF};
	if(MFRC522_REQA(MFRC522) == ATQA_RECIEVED){ 
		if(MFRC522_select_PICC(MFRC522) == MFRC522_OK){
			print_picc_select_info(MFRC522);
			MFRC522_auth_PICC(MFRC522, 0x00, default_key);
			dump_sector_info(MFRC522, 0);
			MFRC522_auth_PICC(MFRC522, 0x00, default_key);
			dump_sector_info(MFRC522, 1);
			MFRC522_stop_encrypt_comm(MFRC522);
			MFRC522_deinit(MFRC522);
			return;
		}
	}
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
	uint8_t block_write_data[16] = {0x0, 0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0x8, 0x9, 0xA, 0xB, 0xC, 0xD, 0xE, 0xF};
	uint8_t block_data[16] = {0};
	uint8_t block_addr = 0x04;

	MFRC522_t MFRC522;

	MFRC522_init(&MFRC522);
	MFRC522_clear_FIFO(&MFRC522);

	while(1){
		if(MFRC522_REQA(&MFRC522) == ATQA_RECIEVED){ 
			if(MFRC522_select_PICC(&MFRC522) == MFRC522_OK){
				print_picc_select_info(&MFRC522);
				MFRC522_auth_PICC(&MFRC522, block_addr, default_key);
				dump_sector_info(&MFRC522, 1);
				if(MFRC522_write_PICC(&MFRC522, block_addr, block_write_data) == MFRC522_OK){
					printf("successfully written to block %i\n", block_addr);
					dump_sector_info(&MFRC522, 1);
				}
				MFRC522_stop_encrypt_comm(&MFRC522);
				MFRC522_deinit(&MFRC522);
				break;
			}
		}
	}


	return 0;
}
