#include "uart.h"
#include "printf.h"
#include "MFRC522.h"
#include <stdint.h>


int main(void){
	UART_config();
	uint8_t read_val = 0;
	MFRC522_t MFRC522;
	MFRC522_init(&MFRC522);
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

	MFRC522_self_test(&MFRC522);


	MFRC522_deinit(&MFRC522);
	while(1);

	return 0;
}
