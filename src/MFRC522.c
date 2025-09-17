/*
 * Major TODO:
 * - Verify all MFRC522_write_reg are not overwriting default register vals
 * - Create enums of usefule register specific values, ei Tx1EN for enabling
 *   transmitter 1 in the TxControlReg
 * - Note down troubleshooting notes
 *
 *
 */
#include "MFRC522.h"
#include <stdint.h>

/**
 * _EXTI0_init() - Sets up EXTI0 of STM32F446 to wait for rising edge of MFRC522 interrupts
 * via PA0
 */
void _EXTI0_init()
{
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PA;
	EXTI->IMR |= EXTI_IMR_IM0;
	EXTI->RTSR |= EXTI_RTSR_TR0;
	EXTI->FTSR &= ~(EXTI_FTSR_TR0);
	NVIC_EnableIRQ(EXTI0_IRQn);
	//TODO: REMOVE, only for debugging EXTI IRQ is working
	GPIOA->MODER |= GPIO_MODER_MODER5_0;
}


void GPIO_init()
{
	GPIO_InitTypeDef led;
	led.Pin = GPIO_PIN_5;
	led.Mode = GPIO_MODE_OUTPUT_PP;	
	led.Pull = GPIO_PULLDOWN;
	led.Speed = GPIO_SPEED_FREQ_MEDIUM;
	HAL_GPIO_Init(GPIOA, &led);
}

void MFRC522_SPI_init(MFRC522_t *me)
{
	me->hspi.Instance = SPI1;
 	me->hspi.Init.Mode = SPI_MODE_MASTER;
	me->hspi.Init.Direction = SPI_DIRECTION_2LINES;
	me->hspi.Init.DataSize = SPI_DATASIZE_8BIT; 
	me->hspi.Init.CLKPolarity = SPI_POLARITY_LOW; 
	me->hspi.Init.CLKPhase = SPI_PHASE_1EDGE; 
	me->hspi.Init.NSS = SPI_NSS_SOFT; 
	//TODO: Check what master clock since communication clock is derived from that
	me->hspi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16; 
	me->hspi.Init.FirstBit = SPI_FIRSTBIT_MSB; 
	me->hspi.Init.TIMode = SPI_TIMODE_DISABLED; 
	me->hspi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED; 
	me->hspi.State = HAL_SPI_STATE_RESET;
}

/**
 * MFRC522_init() - Initializes SPI, GPIO and basic status of MFRC522 struct
 *
 * Return: Status of initialization
 * 0 = Success
 * -1 = SPI failure  
 * -2 = Version verification failed
 */
int MFRC522_init(MFRC522_t *me)
{
	HAL_Init();
	MFRC522_SPI_init(me);
	GPIO_init();
	//TODO: move to SPI_init and change return vals
	if( HAL_SPI_Init(&(me->hspi)) != HAL_OK)
		return -1;
	me->status = IDLE;
	me->error = NO_ERROR;
	MFRC522_soft_reset(me);

	//Following init from
	//https://github.com/OSSLibraries/Arduino_MFRC522v2/blob/master/src/MFRC522v2.cpp#L132
	//to test for now
	//Autostart timer
	MFRC522_write_reg(me, TModeReg, 0x80 );
	// TPreScaler = TModeReg[3..0]:TPrescalerReg, ie 0x0A9 = 169 => f_timer=40kHz, ie a timer period of 25μs.
	MFRC522_write_reg(me, TPrescalarReg, 0xA9);
	MFRC522_write_reg(me, TReloadHiReg, 0x03);
	MFRC522_write_reg(me, TReloadLoReg, 0xE8);
	MFRC522_write_reg(me, TxASKReg, 0x40);
	MFRC522_write_reg(me, ModeReg, 0x3D);
	MFRC522_TxEnable(me);

	//Verify by reading version reg
	MFRC522_read_reg(me, VersionReg);
	if(me->Rx_buf == 0x92){
		printf("Successfully initialized MFRC522\n"); 
		return 0;
	}
	else{
		printf("Failed to initialize MFRC522\n"); 
		return -2;
	}
}

void MFRC522_TxEnable(MFRC522_t *me)
{
	MFRC522_read_reg(me, TxControlReg);
	uint8_t TxControlReg_default = me->Rx_buf;
	//Enable Tx1 and Tx2 lines(user manual section 9.3.2.5)
	MFRC522_write_reg(me, TxControlReg, TxControlReg_default|0x03);
}

void MFRC522_deinit(MFRC522_t *me)
{
	HAL_SPI_DeInit(&(me->hspi));	
}


/**
 * is_initialized() - Helper function to check if `MFRC522_init` has been
 * called where status is set not UNKOWN state.
 *
 * Return: 0 if status is UNKOWN(0), 1 otherwise
 */
uint8_t is_initialized(MFRC522_t *me)
{
	if( me->status == UNKOWN ){
		me->status = MFRC522_ERROR;
		me->error = UNINITIALIZED;
		return 0;
	}
	return 1;
}

/**
 * addr_trans() - Helper function to translates register address to read address
 *
 * When interacting with MFRC522 via SPI, the MSB defines the r/w mode and the LSB
 * is reserved. The register addresses only go up to 3Fh leaving 2 bits of room
 * for mode and reserved bit. Function shifts `addr` right 1, then writes MSB
 * based on `rw_mode`.
 *
 * @addr: Desired MFRC522 register address 
 * @rw_mode: 0=read, 1=write
 *
 * Return: Converted MFRC522 register address 
 */
uint8_t addr_trans(uint8_t addr, uint8_t rw_mode)
{
	addr = (addr<<1);
	if(rw_mode==READ)
		addr |= (0b10000000);
	else
		addr &= ~(0b10000000);
	return addr;
}

/** 
 * MFRC522_read_reg() - Read register byte and wait to return value
 *
 * Return: Read value of register, 0 indicates potential error and user should
 * check object error status to confirm
 */
uint8_t MFRC522_read_reg(MFRC522_t *me, PCD_reg reg)
{
	uint8_t rx[2];
	uint8_t tx[2];	

	if( !(is_initialized(me)) )
		return me->status;

	tx[0] = addr_trans(reg, READ);
	tx[1] = 0x00;

	if(me->hspi.State != HAL_SPI_STATE_READY)
		printf("SPI not ready\n");

	HAL_GPIO_WritePin(GPIOB, CSS_PIN, GPIO_PIN_RESET);
	if(HAL_SPI_TransmitReceive( &(me->hspi), tx, rx, 2, SPI_TIMEOUT) != HAL_OK){
		printf("read reg failed\n");
		HAL_GPIO_WritePin(GPIOB, CSS_PIN, GPIO_PIN_SET);
		me->status = MFRC522_ERROR; 
		me->error = READ_REG_FAILURE; 
		return MFRC522_ERROR;
	}

	HAL_GPIO_WritePin(GPIOB, CSS_PIN, GPIO_PIN_SET);
	if(me->hspi.State != HAL_SPI_STATE_ERROR){
		me->Rx_buf = rx[1];
		me->status = MFRC522_OK;	
		return MFRC522_OK;
	}
	//Might want to error handle here and not return 0, as 0 is a valid value
	else{
		me->status = MFRC522_ERROR;
		me->error = READ_REG_FAILURE;
		return MFRC522_ERROR; 
	}
}

uint8_t MFRC522_write_reg(MFRC522_t *me, PCD_reg reg, uint8_t data)
{
	if( !(is_initialized(me)) )
		return me->status;
	uint8_t tx[2];
	tx[0] = addr_trans(reg, WRITE);
	tx[1] = data; 
	
	HAL_GPIO_WritePin(GPIOB, CSS_PIN, GPIO_PIN_RESET);
	if(HAL_SPI_Transmit( &(me->hspi), tx, 2, SPI_TIMEOUT) != HAL_OK){
		printf("write reg failed\n");
		HAL_GPIO_WritePin(GPIOB, CSS_PIN, GPIO_PIN_SET);
		me->status = MFRC522_ERROR; 
		me->error = READ_REG_FAILURE; 
		return MFRC522_ERROR;
	}
	HAL_GPIO_WritePin(GPIOB, CSS_PIN, GPIO_PIN_SET);
	if(me->hspi.State != HAL_SPI_STATE_ERROR)
		return HAL_OK;
	else{
		me->status = MFRC522_ERROR;
		me->error = READ_REG_FAILURE;
		return MFRC522_ERROR; 
	}
}

void MFRC522_write_cmd(MFRC522_t *me, MFRC522_cmd cmd)
{
	if( !(is_initialized(me)) )
		return;
	MFRC522_write_reg(me, CommandReg, cmd);
}	

/**
 * MFRC522_soft_reset - Executes `SoftReset` cmd within MFRC522 module
 * CommandReg then wait for `PowerDown` bit within CommandReg
 */
void MFRC522_soft_reset(MFRC522_t *me)
{
	if( !(is_initialized(me)) )
		return;
	uint8_t cmd_reg = 0;
	MFRC522_write_cmd(me, SoftReset);
	do{
		MFRC522_read_reg(me, CommandReg);
		cmd_reg = me->Rx_buf;
		HAL_Delay(10);
	}while(cmd_reg & (CMDREG_POWERDOWN));
}

void MFRC522_flush_FIFO(MFRC522_t *me)
{
	if( !(is_initialized(me)) )
		return;
	MFRC522_write_reg(me, FIFOLevelReg, 0x80);
}


/**
 * MFRC522_self_test() - Configures and runs self test ensuring reading data
 * and FIFO is working as expected
 *
 * Clears 25 bytes of FIFO buffer, Configures AutoTestReg, then read FIFO after
 * starting selftest with CalcCRC command. Have to wait until FIFOLevelReg is
 * 64 to validate FIFO data. 
 * Steps described in section '16.1.1 Self Test' of MFRC522 datasheet
 * rev 3.9.
 *
 * Return: 0 on failure, 1 on success
 */

uint8_t MFRC522_self_test(MFRC522_t *me)
{
	uint8_t tmp_val = 0;
	uint8_t irq, cmd_reg; 
	uint8_t FIFO_level = 0;
	if( !(is_initialized(me)) )
		return 0;
	MFRC522_write_reg(me, ComIrqReg, 0);

	MFRC522_read_reg(me, ComIrqReg);
	irq = MFRC522_get_rx_buf(me);
	//printf("IRQ: %X \n", irq);

	MFRC522_write_reg(me, CommandReg, SoftReset);
	HAL_Delay(50);
	//TODO: Add timeout to break out of while loop
	do{
		MFRC522_read_reg(me, CommandReg);
		cmd_reg = MFRC522_get_rx_buf(me);
		printf("CMD: %X \n", cmd_reg);
	} while( (cmd_reg & MFRC522_CMD_PWRDWN) );

	MFRC522_write_reg(me, FIFOLevelReg, 0x80);
	// TODO: create write stream function
	for(int i=0; i<25; i++){
		MFRC522_write_reg(me, FIFODataReg, 0x00);
	}
	MFRC522_write_reg(me, CommandReg, Mem);
	MFRC522_write_reg(me, AutoTestReg, 0x09);
	MFRC522_write_reg(me, FIFODataReg, 0x00);
	MFRC522_write_reg(me, CommandReg, CalcCRC);
	do{
		MFRC522_read_reg(me, FIFOLevelReg);
		FIFO_level = MFRC522_get_rx_buf(me);
	}
	while( FIFO_level < 64 );
	MFRC522_write_reg(me, CommandReg, Idle);
	for(int i=0; i<64; i++){
		if(i%8==0)
			printf("\n");
		MFRC522_read_reg(me, FIFODataReg);
		tmp_val = MFRC522_get_rx_buf(me);
		printf("%X, ", tmp_val);
	}
	printf("\n");

	MFRC522_write_reg(me, AutoTestReg, 0x00);
	return 0;
}

void MFRC522_REQA(MFRC522_t *me)
{	
	uint8_t atqa[2] = {0};
	MFRC522_write_reg(me, CollReg, 0x80);
	MFRC522_clear_IRQ(me);
	MFRC522_flush_FIFO(me);
	MFRC522_read_reg(me, Status1Reg);
	printf("Status1 before transceive: %X\n", me->Rx_buf);

	MFRC522_transeive(me, REQA, NULL, 0);

}

void MFRC522_transeive(MFRC522_t *me, PCD_CMD cmd, uint8_t *data_buf, uint8_t data_buf_len)
{	
	uint8_t irq_val = 0;
	uint8_t tmp = 0;

	//First send cmd to FIFO
	MFRC522_write_reg(me, FIFODataReg, cmd);
	//TODO: modularize, just here to test REQA command 
	MFRC522_write_reg(me, BitFramingReg, 0x07);

	//Check if data is not null, then write to FIFO
	if(*data_buf){
		for(int i=0; i<data_buf_len; i++){
			MFRC522_write_reg(me, FIFODataReg, data_buf[i]);
		}
	}
	//write transceive command 
	MFRC522_write_reg(me, CommandReg,Transceive);
	//write `start` bit within BitFramingReg to start sending data
	MFRC522_read_reg(me, BitFramingReg);
	MFRC522_write_reg(me, BitFramingReg, (me->Rx_buf|STARTSEND));
	//wait for transceive command to finish, indicated with either RxIRQ, IdleIrq or TimerIRQ
	do{
		MFRC522_read_reg(me, ComIrqReg);
		irq_val = me->Rx_buf;
	}while(!( (irq_val & (RxIRQ|IdleIRQ)) || (irq_val & TimerIRQ) ));
	if(irq_val & RxIRQ)
		printf("Rx IRQ received\n");
	if(irq_val & IdleIRQ)
		printf("Idle IRQ received\n");
	if(irq_val & TimerIRQ)
		printf("Timer timed out received\n");
	MFRC522_read_reg(me, RxModeReg);	
	printf("irq_val: %X, RxModeReg: %X\n", irq_val, me->Rx_buf);
	//Print to test for now
	MFRC522_read_reg(me, FIFOLevelReg);
	printf("Level Reg after transceive: %X\n", me->Rx_buf);

	MFRC522_read_reg(me, FIFODataReg);
	printf("Byte 1: %X\n",me->Rx_buf);
	MFRC522_read_reg(me, FIFODataReg);
	printf("Byte 2: %X\n",me->Rx_buf);

}

void MFRC522_clear_IRQ(MFRC522_t *me)
{
	MFRC522_write_reg(me, ComIrqReg, 0x7F);
}


uint8_t MFRC522_get_rx_buf(MFRC522_t *me)
{
	return me->Rx_buf; 
}
