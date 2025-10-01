/*
 * Major TODO:
 * - Verify all MFRC522_write_reg are not overwriting default register vals
 * - Complete anticollision algorithm
 * - Validate irq complete criteria logic 
 *
 *
 */
#include "MFRC522.h"

/**
 * clear_reg_bits() - Helper function to clear specific bits within register
 * without clearing already set bits unintentionally 
 *
 * @reg: desired register to modify
 * @clear_bitmask: Bits set in bitmask are cleared (set to logic 0) within `reg` 
 */
void clear_reg_bits(MFRC522_t *me, PCD_reg reg, uint8_t clear_bitmask)
{
	MFRC522_read_reg(me, reg);
	MFRC522_write_reg(me, reg, me->Rx_buf&(~clear_bitmask));
}

/**
 * set_reg_bits() - Sets bits within MFRC522 register without altering other
 * bits within desired register.
 *
 * @reg: desired register to modify
 * @clear_bitmask: Bits set in bitmask are set (to logic 0) within `reg` 
 */
void set_reg_bits(MFRC522_t *me, PCD_reg reg, uint8_t set_bitmask)
{
	MFRC522_read_reg(me, reg);
	MFRC522_write_reg(me, reg, me->Rx_buf|set_bitmask);
}

/**
 * GPIO_init() - Additional GPIO resources initialized for debugging or
 * non-primary MFRC522 use.
 */ 
void GPIO_init()
{
	GPIO_InitTypeDef led;
	led.Pin = GPIO_PIN_5;
	led.Mode = GPIO_MODE_OUTPUT_PP;	
	led.Pull = GPIO_PULLDOWN;
	led.Speed = GPIO_SPEED_FREQ_MEDIUM;
	HAL_GPIO_Init(GPIOA, &led);
}

/**
 * MFRC522_SPI_init() - Configures SPI1 to interact with MFRC522. Must be
 * configured and called before `HAL_SPI_Init` to correctly setup SPI1. 
 */
void MFRC522_SPI_init(MFRC522_t *me)
{
	me->hspi.Instance = SPI1;
 	me->hspi.Init.Mode = SPI_MODE_MASTER;
	me->hspi.Init.Direction = SPI_DIRECTION_2LINES;
	me->hspi.Init.DataSize = SPI_DATASIZE_8BIT; 
	me->hspi.Init.CLKPolarity = SPI_POLARITY_LOW; 
	me->hspi.Init.CLKPhase = SPI_PHASE_1EDGE; 
	me->hspi.Init.NSS = SPI_NSS_SOFT; 
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
	set_reg_bits(me, TxControlReg, TxControl_Tx1RFEn|TxControl_Tx2RFEn);
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
 * MFRC522_read_reg() - Read register byte and wait for value stored at desired
 * register address. Value is placed in `me->Rx_buf` and the status of the
 * function is returned.  
 *
 * Return: `MFRC522_ERROR` if error occurs, otherwise `MFRC522_OK`.
 */
uint8_t MFRC522_read_reg(MFRC522_t *me, PCD_reg reg)
{
	uint8_t rx[2];
	uint8_t tx[2];	

	if( !(is_initialized(me)) )
		return me->status;

	tx[0] = addr_trans(reg, READ);
	tx[1] = 0x00; //Dummy transmit byte

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
	}while(cmd_reg & (CMDREG_PWRDWN));
}

void MFRC522_flush_FIFO(MFRC522_t *me)
{
	if( !(is_initialized(me)) )
		return;
	MFRC522_write_reg(me, FIFOLevelReg, FIFOLevel_FlushBuffer);
}

void MFRC522_calc_CRC(MFRC522_t *me, uint8_t *data, uint8_t data_size, uint8_t *result)
{
	MFRC522_write_reg(me, CommandReg, IDLE);	
	// Clear CRCIRQ
	MFRC522_write_reg(me, DivIrqReg, DivIrq_CRCIrq);	
	MFRC522_flush_FIFO(me);
	for(int i=0; i<data_size; i++){
		MFRC522_write_reg(me, FIFODataReg, data[i]);
	}
	MFRC522_write_reg(me, CommandReg, CalcCRC);

	//wait for CRCIRQ
	do{
		MFRC522_read_reg(me, DivIrqReg);
	}while( !(me->Rx_buf & DivIrq_CRCIrq) ); 
	MFRC522_read_reg(me, CRCResultLSBReg);
	result[0] = me->Rx_buf; 
	MFRC522_read_reg(me, CRCResultMSBReg);
	result[1] = me->Rx_buf; 

	MFRC522_write_reg(me, CommandReg, IDLE);	

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
	MFRC522_clear_IRQ(me);

	MFRC522_write_reg(me, CommandReg, SoftReset);
	HAL_Delay(50);
	//TODO: Add timeout to break out of while loop
	do{
		MFRC522_read_reg(me, CommandReg);
		cmd_reg = me->Rx_buf;
		printf("CMD: %X \n", cmd_reg);
	} while( (cmd_reg & CMDREG_PWRDWN) );

	MFRC522_flush_FIFO(me);
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
		FIFO_level = me->Rx_buf;
	}
	while( FIFO_level < 64 );
	MFRC522_write_reg(me, CommandReg, Idle);
	for(int i=0; i<64; i++){
		if(i%8==0)
			printf("\n");
		MFRC522_read_reg(me, FIFODataReg);
		tmp_val = me->Rx_buf;
		printf("%X, ", tmp_val);
	}
	printf("\n");

	MFRC522_write_reg(me, AutoTestReg, 0x00);
	return 0;
}

void MFRC522_REQA(MFRC522_t *me)
{	
	uint8_t atqa[2] = {0};
	//Require var to send to `MFRC522_transceive` 
	uint8_t reqa = REQA; 
	MFRC522_write_reg(me, CollReg, Coll_ValuesAfterColl);
	//TODO: modularize, just here to test REQA command 
	MFRC522_write_reg(me, BitFramingReg, BitFraming_StartSend);
	MFRC522_clear_IRQ(me);
	MFRC522_flush_FIFO(me);

	MFRC522_transeive(me, REQA, &reqa, 1);

	//Print to test for now
	MFRC522_read_reg(me, FIFOLevelReg);
	printf("Level Reg after REQA: %X\n", me->Rx_buf);

	MFRC522_read_reg(me, FIFODataReg);
	printf("Byte 1: %X\n",me->Rx_buf);
	MFRC522_read_reg(me, FIFODataReg);
	printf("Byte 2: %X\n",me->Rx_buf);


}

void MFRC522_CL1(MFRC522_t *me, uint8_t *res_buf)
{
	uint8_t sel_cl1_buf[2] = {SELECT_CL1, 0x20};
	uint8_t fifo_datalevel = 0;
	//Ensure BitFramingReg is set for sending 8 bits
	MFRC522_read_reg(me, BitFramingReg);	
	//Clear first 3 bits to reset sending to 8 bits instead of 7 from REQA
	clear_reg_bits(me, BitFramingReg, BitFraming_StartSend);
	MFRC522_clear_IRQ(me);
	//Clear collisions to prep MFRC522 
	clear_reg_bits(me, CollReg, Coll_ValuesAfterColl);
	MFRC522_transeive(me, SELECT_CL1, sel_cl1_buf, 2);

	//Print to test for now
	MFRC522_read_reg(me, FIFOLevelReg);
	printf("Level Reg after SEL CL1: %X\n", me->Rx_buf);
	fifo_datalevel = me->Rx_buf;

	for(int i=0; i<fifo_datalevel; i++){
		MFRC522_read_reg(me, FIFODataReg);
		res_buf[i] = me->Rx_buf;
		printf("Byte %i: %X\n",i, me->Rx_buf);
	}
}

/**
 * MFRC522_select_PICC() - Called after response from any num of PICCs,
 * completes selection sequence defined in ISO/IEC14443. 
 *
 * REQA or WAKUP command shall be executed before calling this function to
 * assert UID size from ATQA PICC response. Outer loop is for Collision Level
 * (CLn), then inner loop is to control max collisions within CLn to 32.  
 *
 */
void MFRC522_select_PICC(MFRC522_t *me, uint8_t UID_size)
{
	uint8_t select_buf[9] = {0};
	// CLn increments by 2 because difference of CL values(0x93,0x95,0x97) 
	for(int CLn = SELECT_CL1; CLn<=SELECT_CL3; CLn+2){
		select_buf[SEL_INDEX] = CLn;
		select_buf[NVB_INDEX] = 0x20;
		MFRC522_transeive(me, SELECT, select_buf, 2);
			
				
	}
}

/* 
 * MFRC522_SEL - Performs the final selection of PICC after anticollision has
 * determined PICC with UID bytes
 *
 * Prepares and populates FIFO for `MFRC522_transeive`. The `sel_data` is the
 * data required for selection. The passed `uid_buf` shall include the
 * calculated BCC of the UID bytes, then the CRC is appended from the CRC
 * coprocessor of MFRC522 
 */
void MFRC522_SEL(MFRC522_t *me, uint8_t *uid_buf )
{
	//byte for SEL: SEL_CL1, NVB, UID[0..3], BCC and 2 CRC
	uint8_t sel_buf[SEL_NUM_BYTES] = {SELECT_CL1, 0x70};
	uint8_t CRC_res[2] = {0};
	uint8_t fifo_datalevel = 0;
	//Populate uid and bcc
	for(int i=2; i<SEL_NUM_BYTES; i++){
		sel_buf[i] = uid_buf[i-2];
	}
	
	//Grab CRC from coprocessor if it's ready
	MFRC522_calc_CRC(me, sel_buf, SEL_NUM_BYTES-2, CRC_res);		
	//index 0 low is CRC and index 1 is CRC high result
	sel_buf[SEL_NUM_BYTES-2] = CRC_res[0];
	sel_buf[SEL_NUM_BYTES-1] = CRC_res[1];

	// Printing for testing
	for(int i=0; i<SEL_NUM_BYTES; i++)
		printf("Byte %i: %X\n", i, sel_buf[i]);

	MFRC522_transeive(me, SELECT_CL1, sel_buf, SEL_NUM_BYTES);	

	//Print to test for now
	MFRC522_read_reg(me, FIFOLevelReg);
	printf("Level Reg after SEL CL1: %X\n", me->Rx_buf);
	fifo_datalevel = me->Rx_buf;

	for(int i=0; i<fifo_datalevel; i++){
		MFRC522_read_reg(me, FIFODataReg);
		printf("Byte %i: %X\n",i, me->Rx_buf);
	}
}

void MFRC522_read_PICC(MFRC522_t *me, uint8_t block_addr)
{
	uint8_t read_fifo_data[4] = {READ_PICC, block_addr};
	uint8_t crc_res[2] = {0};
	uint8_t fifo_datalevel = 0;
	MFRC522_calc_CRC(me, read_fifo_data, 2, crc_res);
	read_fifo_data[2] = crc_res[0];
	read_fifo_data[3] = crc_res[1];
	for(int i=0; i<4; i++){
		printf("read byte %i: %X\n", i, read_fifo_data[i]);
	}

	MFRC522_transeive(me, READ_PICC, read_fifo_data, 4);

	//Print to test for now
	MFRC522_read_reg(me, FIFOLevelReg);
	printf("Level Reg after READ: %i\n", me->Rx_buf);
	fifo_datalevel = me->Rx_buf;

	for(int i=0; i<fifo_datalevel; i++){
		MFRC522_read_reg(me, FIFODataReg);
		printf("Byte %i: %X\n",i, me->Rx_buf);
	}
}	


/**
 * MFRC522_auth_PICC - Establishes a encrypted communication between PCD and
 * PICC using sector key to allow access to specific block.
 *
 * MFRC522 `MFAuthent` command expects auth command, block address, sector key
 * then serial number of card(UID) in FIFO. Successful authentication is
 * indicated by `MFCrypto1On` bit set in `Status2Reg`. Unsuccessful
 * authentication requires timeout.  
 *
 *
 */
void MFRC522_auth_PICC(MFRC522_t *me, uint8_t block_addr, uint8_t sector_key[6], uint8_t uid[4])
{
	uint8_t mf_auth_buf[12] = {0};
	uint8_t com_irq = 0; 
	uint8_t status2 = 0; 

	MFRC522_flush_FIFO(me);
	MFRC522_clear_IRQ(me);
	//TODO: add to defines, this is for KEYA, KEYB=0x61
	//mf_auth_buf[0] = 0x60;
	//mf_auth_buf[1] = block_addr;
	MFRC522_write_reg(me, FIFODataReg, MFAUTH_KEYA);
	MFRC522_write_reg(me, FIFODataReg, block_addr);
	for(int i=0; i<6; i++){
		//mf_auth_buf[i+2] = sector_key[i];
		MFRC522_write_reg(me, FIFODataReg, sector_key[i]);
	}
	for(int i=0; i<4; i++){
		//mf_auth_buf[i+8] = uid[i];
		MFRC522_write_reg(me, FIFODataReg, uid[i]);
	}
	MFRC522_write_reg(me, CommandReg, MFAuthent);
	//start timer (Doesn't seem to do anything rn)
	set_reg_bits(me, ControlReg, Control_TStartNow);
	do{
		MFRC522_read_reg(me, ComIrqReg);
		com_irq = me->Rx_buf;
		MFRC522_read_reg(me, Status2Reg);
		status2 = me->Rx_buf;
	}while( !(status2 & Status2_MFCrypto1On) || (com_irq & (ComIrq_TimerIrq|ComIrq_IdleIrq) ) );
	if(status2 & Status2_MFCrypto1On)
		printf("MFAuthent connection Successful\n");
	if(com_irq & ComIrq_IdleIrq)
		printf("MFAuth completed and returned to IDLE\n");
	if(com_irq & ComIrq_TimerIrq)
		printf("MFAuth timed out\n");
	
}

/*
 * data_buf needs to include CMD to make it easier to represent all data for
 * a SEL command and so it's included in buffer when calculating CRC
 * TODO: Change `data_buf` to include CMD and just write to FIFO
 * One thing is to calculate CRC we need to write the same data to FIFO,
 * ideally we just write data to FIFO once, calc CRC
 */
void MFRC522_transeive(MFRC522_t *me, uint8_t cmd, uint8_t *data_buf, uint8_t data_buf_len)
{	
	uint8_t irq_val = 0;
	uint8_t tmp = 0;
	MFRC522_clear_IRQ(me);
	MFRC522_flush_FIFO(me);

	//Check if data is not null, then write to FIFO
	if(*data_buf){
		for(int i=0; i<data_buf_len; i++){
			MFRC522_write_reg(me, FIFODataReg, data_buf[i]);
		}
	}
	else{
		printf("Invalid `data_buf` for transeive function\n");
		me->status = MFRC522_ERROR; 
		me->error = PCD_TRANSEIVE_FAILURE;
		return;
	}
	//write transceive command 
	MFRC522_write_reg(me, CommandReg,Transceive);
	//write `start` bit within BitFramingReg to start sending data
	set_reg_bits(me, BitFramingReg, STARTSEND);
	//wait for transceive command to finish, indicated with either RxIRQ, IdleIrq or TimerIRQ
	do{
		MFRC522_read_reg(me, ComIrqReg);
		irq_val = me->Rx_buf;
	}while(!( (irq_val & (ComIrq_RxIrq|ComIrq_IdleIrq)) || (irq_val & ComIrq_TimerIrq) ));
	if(irq_val & ComIrq_RxIrq)
		printf("Rx IRQ received\n");
	if(irq_val & ComIrq_IdleIrq)
		printf("Idle IRQ received\n");
	if(irq_val & ComIrq_TimerIrq)
		printf("Timer timed out received\n");
	MFRC522_read_reg(me, RxModeReg);	
	printf("irq_val: %X, RxModeReg: %X\n", irq_val, me->Rx_buf);
}

void MFRC522_clear_IRQ(MFRC522_t *me)
{
	MFRC522_write_reg(me, ComIrqReg, CLEAR_IRQ_REG);
	MFRC522_write_reg(me, DivIrqReg, CLEAR_IRQ_REG);
}

void MFRC522_stop_encrypt_comm(MFRC522_t *me)
{
	clear_reg_bits(me, Status2Reg, Status2_MFCrypto1On);	
	printf("MFAuthent disconnected \n");
}

