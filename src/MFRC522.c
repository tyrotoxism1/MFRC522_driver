/*
 * Major TODO:
 * - Change MFRC522_read_reg to return read value instead of status for
 *   readability. print error instead of return  
 * - Add HALTA command 
 * - Add WUPA command
 * - Add PICC write functionality
 * - Add block access condition check function
 */

#include "MFRC522.h"
#include <stdint.h>

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
 *
 * Return:
 * -1 = Fail
 *  0 = Success 
 */
int MFRC522_SPI_init(MFRC522_t *me)
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

	if( HAL_SPI_Init(&(me->hspi)) != HAL_OK)
		return -1;
	else
		return 0;
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
	if(MFRC522_SPI_init(me) == -1)
		return -1;
	GPIO_init();
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
		printf("\n\nSuccessfully initialized MFRC522\n");
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


void print_picc_select_info(MFRC522_t *me){
	uint8_t picc_num_bytes = ((me->curr_picc.picc_uid_size)+1)*3+1;
	printf("PICC UID num bytes: %i\n", picc_num_bytes);
	if(me->curr_picc.picc_state == PICC_ACTIVE){
		printf("------------------- PICC SELECT INFO--------------------\n");
		if(me->curr_picc.iso_14443_compliant)
			printf("PICC IS ISO/IEC 14443 Compliant\n");	
		else
			printf("PICC IS NOT ISO/IEC 14443 Compliant\n");	
		for(int i=0; i<=picc_num_bytes; i++){
			printf("UID %i: %X\n", i, me->curr_picc.picc_uid[i]);
		}
	}
	else {
		printf("PICC not in active state\n");
	}
}

/* dump_sector_info() - Prints out all bytes (in hex) to serial output for
 * a single sector of PICC.
 *
 * Ensure PICC is selected and authentication has established encrypted
 * communication. 
 * `Sector` parameter is multiplied by 4 to get starting block address of
 * sector. 
 */
void dump_sector_info(MFRC522_t *me, uint8_t sector)
{
	if(me->curr_picc.picc_state != PICC_ACTIVE){
		printf("Cannot dump sector info, PICC is not in ACTIVE state\n");
		return;
	}
	uint8_t block_addr_strt = sector*4;
	uint8_t block_data[64] = {0};
	
	printf("Grabbing sector %i info\n", sector);
	for(int sctr_blck=0; sctr_blck<4; sctr_blck++){
		MFRC522_read_PICC(me, block_addr_strt+sctr_blck, block_data+(16*sctr_blck));	
	}
	printf("------------------Sector %i Data ------------------\n", sector);
	for(int block=0; block<4; block++){
		printf("\n---Block %i---\n", block);
		for(int byte=0; byte<16; byte++){
			if(byte==8)
				printf("\n");
			printf("%i:%X\t", byte, block_data[byte+(16*block)]);
			
		}
	}
	printf("\n\n");
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

void MFRC522_clear_FIFO(MFRC522_t *me)
{
	MFRC522_write_reg(me, FIFOLevelReg, FIFOLevel_FlushBuffer);
	for(int i=0; i<64; i++){
		MFRC522_write_reg(me, FIFODataReg, 0x00);
	}
	MFRC522_write_reg(me, FIFOLevelReg, FIFOLevel_FlushBuffer);
}

void MFRC522_FIFO_read_stream(MFRC522_t *me, uint8_t *buf, uint8_t buf_len, uint8_t print)
{
	MFRC522_read_reg(me, FIFOLevelReg);
	// Ensure buffer length doesn't exceed FIFO "water level"
	if(buf_len>me->Rx_buf)
		buf_len = me->Rx_buf;
	for(int i=0; i<buf_len; i++){
		MFRC522_read_reg(me, FIFODataReg);
		buf[i] = me->Rx_buf;
		if(print)
			printf("Index %i: %X\n", i, me->Rx_buf);
	}
}

void MFRC522_FIFO_write_stream(MFRC522_t *me, uint8_t *buf, uint8_t buf_len)
{
	for(int i=0; i<buf_len; i++){
		MFRC522_write_reg(me, FIFODataReg, buf[i]);
	}
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
 * Return: 
 * - -1 = failure
 * - 0 = success
 */

int MFRC522_self_test(MFRC522_t *me)
{
	uint8_t tmp_val = 0;
	uint8_t irq, cmd_reg;
	uint8_t FIFO_level = 0;
	uint8_t fifo_data_buf[25] = {0};
	if( !(is_initialized(me)) )
		return -1;
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
	MFRC522_FIFO_write_stream(me, fifo_data_buf, 25);

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


void init_picc(MFRC522_t *me, PICC_SIZE_t uid_size){
	me->curr_picc.picc_state = PICC_STATE_UNKNOWN;
	me->curr_picc.picc_uid_complete = 0;
	me->curr_picc.picc_uid_size = uid_size;
	me->curr_picc.iso_14443_compliant = PICC_IS_UKNOWN_ISOIEC1443_COMPLIANT; 
}


uint8_t MFRC522_REQA(MFRC522_t *me)
{
	uint8_t atqa[2] = {0};
	//Require var to send to `MFRC522_transceive`
	uint8_t reqa = REQA;
	MFRC522_write_reg(me, CollReg, Coll_ValuesAfterColl);
	//TODO: modularize, just here to test REQA command
	MFRC522_write_reg(me, BitFramingReg, 0x07);
	MFRC522_clear_IRQ(me);
	MFRC522_flush_FIFO(me);

	MFRC522_transeive(me, REQA, &reqa, 1);

	if(me->status !=  MFRC522_OK){
		//Clear first 3 bits to reset sending to 8 bits instead of 7 from REQA
		clear_reg_bits(me, BitFramingReg, 0x07);
		return NO_ATQA_RECIEVED;
	}
	else {
		printf("Received REQA! STATUS: %i\n", me->status);
		//Print to test for now
		MFRC522_read_reg(me, FIFOLevelReg);
		printf("Level Reg after REQA: %X\n", me->Rx_buf);

		MFRC522_read_reg(me, FIFODataReg);
		printf("ATQA 1: %X\n",me->Rx_buf);
		atqa[0] = me->Rx_buf;
		MFRC522_read_reg(me, FIFODataReg);
		printf("ATQA 2: %X\n",me->Rx_buf);
		atqa[1] = me->Rx_buf;
		//Clear first 3 bits to reset sending to 8 bits instead of 7 from REQA
		clear_reg_bits(me, BitFramingReg, 0x07);
		init_picc(me, (atqa[0])>>6);

		return ATQA_RECIEVED;
	}
}

uint8_t MFRC522_WUPA(MFRC522_t *me)
{
	uint8_t atqa[2] = {0};
	//Require var to send to `MFRC522_transceive`
	PCD_CMD_t wupa = WUPA;
	MFRC522_write_reg(me, CollReg, Coll_ValuesAfterColl);
	//TODO: modularize, just here to test wupa command
	MFRC522_write_reg(me, BitFramingReg, 0x07);
	MFRC522_clear_IRQ(me);
	MFRC522_flush_FIFO(me);

	MFRC522_transeive(me, WUPA, &wupa, 1);

	if(me->status !=  MFRC522_OK){
		//Clear first 3 bits to reset sending to 8 bits instead of 7 from wupa
		clear_reg_bits(me, BitFramingReg, 0x07);
		return NO_ATQA_RECIEVED;
	}
	else {
		printf("Received wupa! STATUS: %i\n", me->status);
		//Print to test for now
		MFRC522_read_reg(me, FIFOLevelReg);
		printf("Level Reg after wupa: %X\n", me->Rx_buf);

		MFRC522_read_reg(me, FIFODataReg);
		printf("ATQA 1: %X\n",me->Rx_buf);
		atqa[0] = me->Rx_buf;
		MFRC522_read_reg(me, FIFODataReg);
		printf("ATQA 2: %X\n",me->Rx_buf);
		atqa[1] = me->Rx_buf;
		//Clear first 3 bits to reset sending to 8 bits instead of 7 from wupa
		clear_reg_bits(me, BitFramingReg, 0x07);
		init_picc(me, (atqa[0])>>6);

		return ATQA_RECIEVED;
	}
}

void MFRC522_CL1(MFRC522_t *me, uint8_t *res_buf)
{
	uint8_t sel_cl1_buf[2] = {SELECT_CL1, 0x20};
	uint8_t fifo_datalevel = 0;
	//Ensure BitFramingReg is set for sending 8 bits
	MFRC522_read_reg(me, BitFramingReg);
	//Clear first 3 bits to reset sending to 8 bits instead of 7 from REQA
	clear_reg_bits(me, BitFramingReg, 0x07);
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

uint8_t valid_bcc(uint8_t data[4], uint8_t bcc)
{
	uint8_t actual_bcc = 0;
	for(int uid_byte=0; uid_byte<4; uid_byte++){
		actual_bcc ^= data[uid_byte];
	}
	return (actual_bcc == bcc);
}

uint8_t validate_select_picc_buf(MFRC522_t *me, uint8_t *data)
{
	if(valid_bcc(data+2, data[6])){
		printf("Received valid BCC, calculating CRC now\n");
		data[NVB_INDEX] = 0x70;
		MFRC522_calc_CRC(me, data, 7, data+7);
	}
	else{
		printf("BCC WASN'T VALID!!\n");
		me->status = MFRC522_ERROR;
		me->error = SELECT_PICC_BCC_FAILURE;
		return MFRC522_ERROR;
	}

	return me->status;
}
/**
 * MFRC522_select_PICC() - Handles selection of PICC via anticollision
 * algorithm defined in ISO/IEC14443-3.
 *
 * Anticollision has outer loop for Collision Level 'n' (CLn) and inner loop
 * for maximum of 32 collision within a CLn. The PICC initially send out
 * request to all PICCs in range, PICCs then respond with their UID bytes and
 * or Cascade Tag (CT). If the PICC response differs from anohter PICC response
 * at the same bit position, a collision occurs. Collisions are handled by
 * `MFRC522_collision_check`. 
 *
 * If a collision occurs, the function resends SEL command with Number of Valid
 * Bits (NVB). This continues until no collision occurs, which means the CLn
 * UID is complete.  
 *
 * Return:
 * - MFRC522_OK upon success
 * - MFRC522_ERROR upon failure
 *
 */
uint8_t MFRC522_select_PICC(MFRC522_t *me)
{
	uint8_t select_buf[9] = {0};
	uint8_t select_buf_len = 2;
	uint8_t CL_collision_count, valid_bits, valid_bytes, sel_buf_idx;
	CL_collision_count = valid_bits = valid_bytes = sel_buf_idx = 0;

	// CLn increments by 2 because difference of CL values(0x93,0x95,0x97)
	for(uint8_t CLn = SELECT_CL1; CLn<=SELECT_CL3; CLn+=2){
		select_buf[SEL_INDEX] = CLn;
		select_buf[NVB_INDEX] = 0x20;
		sel_buf_idx = 2;

		// Send out inital UID request, *could use `do while` here
		MFRC522_transeive(me, CLn, select_buf, select_buf_len);
		while(CL_collision_count<=32){
			if( MFRC522_collision_check(me, select_buf, &sel_buf_idx, &select_buf_len,&valid_bits, &valid_bytes) ){
				MFRC522_transeive(me, CLn, select_buf, select_buf_len);
				CL_collision_count++;
				// Reset BitFramingReg
				clear_reg_bits(me, BitFramingReg, 0x77);
			}
			// no coll occurred, thus should have complete UID for CLn, break
			// out of inner bit collision loop
			else{
				break;
			}
		}

		// Received complete UID for CL within max collisions
		if(CL_collision_count<32){
			printf("-------------CL%X Complete!------------\n", CLn);
			printf("received partial value: %X\n", select_buf[sel_buf_idx]);
			MFRC522_read_reg(me, FIFOLevelReg);
			uint8_t fifo_level = me->Rx_buf;
			printf("FIFO LEVEL: %X\n", fifo_level);
			// Transfer the rest of the UID from FIFO into buffer
			// First "byte" in buffer is rest of collision UID, so combine with
			// partial computered earlier
			// bits before valid_bits in FIFO are random and need to be cleared
			// with mask
			MFRC522_read_reg(me, FIFODataReg);
			if(CL_collision_count>0){
				me->Rx_buf |= create_valid_bitmask(0,valid_bits);
			}
			printf("sel_buf_idx: %i, Second part of partial byte: %X\n",sel_buf_idx, me->Rx_buf);
			select_buf[sel_buf_idx++] |= me->Rx_buf;

			// valid_bytes+3 skips already read bytes and SEL, NVB and just
			// read byte, FIFO level doesn't include SEL and NVB byte offset so
			// add 2 for fifo_level 
			for(int uid_byte=valid_bytes+3; uid_byte<fifo_level+2; uid_byte++){
				MFRC522_read_reg(me, FIFODataReg);
				select_buf[uid_byte] = me->Rx_buf;
			}
			if( validate_select_picc_buf(me, select_buf)!=0 ){
				printf("Selection failed :(\n");
				me->status = MFRC522_ERROR;
				me->error =  SELECT_PICC_FAILURE; 
				return MFRC522_ERROR;	

			}
			printf("Selection complete and valid!\n");
			for(int i=0; i<=8; i++){
				printf("SEL buf %i: %X\n",i,select_buf[i]);
			}
			MFRC522_transeive(me, SELECT_CL1, select_buf, 9);
			// First byte in fifo is SAK, 
			MFRC522_read_reg(me, FIFODataReg);
			// If SAK doesn't have `1` bit in pos 3 of byte, then UID is complete
			if( !((me->Rx_buf) & PICC_SAK_UID_INCOMPLETE) ){
				me->curr_picc.picc_uid_complete = TRUE;
				me->curr_picc.picc_state = PICC_ACTIVE;
				// Copy UID into picc struct
				// TODO: Make compatible with PICC UID size > 1
				// TODO: Verify CRC
				for(int i=0; i<5; i++)
					me->curr_picc.picc_uid[i] = select_buf[i+2];
				printf("SELECTION COMPLETE!!\n");
				return MFRC522_OK;
			}
		}
		else{
			printf("Exceeded max collisions");
			me->status = MFRC522_ERROR;
			me->error = SELECT_PICC_FAILURE;
			return MFRC522_ERROR;
		}
	}
	return MFRC522_ERROR;
}

/**
 * create_valid_bitmask() - helper function that places 1s within a byte
 * between the start and end bit position range provided in arguments.
 *
 * Bit positions are 0 based, for example if function is provided with 
 * `start_bit`=3 and `end_bit`=6 the result would be 0b01111000.
 * This function is used for reading partial bytes from FIFO when selecting
 * PICCs. So if a collision occurs, FIFO contains a partial byte and then next
 * incoming byte after resending selection is also partial. The bits outside of
 * the valid portion of partial byte are unknown and can mess up bitwise logical
 * operation.
 *
 * Return: resulting bitmask of desired bits from `start_bit` to `end_bit` 
 */
uint8_t create_valid_bitmask(uint8_t start_bit, uint8_t end_bit)
{
	uint8_t ret_val = 0;
	uint8_t valid_bits = end_bit-start_bit;
	for(int i=0; i<=valid_bits; i++){
		ret_val = (ret_val<<1)|1;
	}
	ret_val = ret_val<<start_bit;
	return ret_val;
}




/**
 * MFRC522_collision_check() - Checks ErrorReg for collision. If collission
 * occurs, set NVB within `select_picc_buf` and TxLastBits within
 * BitFramingReg for next anticollision loop.
 *
 * Return:
 *	0 = No collision
 *	1 = Collision Occurred
 */
uint8_t MFRC522_collision_check(MFRC522_t *me, uint8_t *select_picc_buf, uint8_t *sel_buf_idx, uint8_t* buf_len, uint8_t *valid_bits, uint8_t *valid_bytes)
{
	// Read err reg to determine if collision occurred
	MFRC522_read_reg(me, ErrorReg);
	uint8_t err_reg = me->Rx_buf;
	uint8_t valid_btmsk_start, valid_btmsk, num_new_bytes;
   	valid_btmsk_start = valid_btmsk = num_new_bytes = 0;
	uint8_t valid_btmsk_end = 7;
	if(err_reg & Error_CollErr){
		printf("Collision occurred\n");
		uint8_t prev_valid_bits = *valid_bits;
		uint8_t prev_valid_bytes = *valid_bytes;
		// Read FIFO to store values and determine num bytes
		// 0b00010000 (0d16), Divide by 8 -> 0b00000010 (0d2) so bit shift by 3
		MFRC522_read_reg(me, CollReg);
		printf("Collision reg: %X\n", me->Rx_buf);
		*valid_bytes= ((me->Rx_buf&Coll_CollPos_Msk) >> 3 );
		// Adding 3 for SEL, NVB, and last incomplete byte
		*buf_len = (*valid_bytes)+3;
		// Subtract 1 to make valid_bits 0 indexed bit position, not doing this
		// since it interferes with tracking valid bits and 
		*valid_bits = ((me->Rx_buf&Coll_CollPos_Msk)%8);
		printf("Prev valid_bytes: %i, Prev valid_bits: %i\n", prev_valid_bytes, prev_valid_bits);
		printf("Valid Bytes: %i, Valid Bits: %i\n", *valid_bytes, *valid_bits);

		// Determine number of new bytes between last coll and currest coll
		num_new_bytes = (*valid_bytes)-prev_valid_bytes;
		if(num_new_bytes==0){
			printf("Collision occurred within same byte\n");
			valid_btmsk = create_valid_bitmask(prev_valid_bits, *valid_bits-1);
			MFRC522_read_reg(me, FIFODataReg);
			// Apply bitmask to read FIFO Val
			me->Rx_buf = (me->Rx_buf)&valid_btmsk;
			// Append decision bit to read byte with coll from PICC 
			me->Rx_buf |= 1<<(*valid_bits-1);
			// combine incomplete byte with value from PICC along with decision
			// bit 
			printf("sel_buf_idx in collision check: %X\n", *sel_buf_idx);
			select_picc_buf[(*sel_buf_idx)] |= me->Rx_buf; 
			if(*valid_bits == 7){
				printf("Incrementing sel_buf_idx\n");
				sel_buf_idx++;
			}
			// With deciion bit appended we can incremment valid_bits
			//valid_bits++;

			// Add upper nibble of `valid_bytes` with 0x20, then add `valid_bits`
			// to lower nibble
			select_picc_buf[NVB_INDEX] = (0x20 + (*valid_bytes<<4)) | (*valid_bits);
			printf("anticollision NVB: %X\n", select_picc_buf[NVB_INDEX]);
			// Set TxLastBits to send valid bits of last byte
			set_reg_bits(me, BitFramingReg, *valid_bits);
			// Align to receive remaining bits of byte where coll occurred
			set_reg_bits(me, BitFramingReg, (*valid_bits<<BitFraming_RxAlign_Pos) );
			MFRC522_read_reg(me, BitFramingReg);
			printf("Bitframing Val: %X\n", me->Rx_buf);

			return COLL_OCCURRED;
		}
		else{
			printf("Collision occurred across multiple bytes\n");
			valid_btmsk = create_valid_bitmask(prev_valid_bits, 7);
			MFRC522_read_reg(me, FIFODataReg);
			select_picc_buf[(*sel_buf_idx)++] |= me->Rx_buf; 
			// Start at 1 because we read and completed first partial byte already
			for(int i=1; i<num_new_bytes; i++){
				MFRC522_read_reg(me, FIFODataReg);
				select_picc_buf[(*sel_buf_idx)++] = me->Rx_buf; 
			}
			valid_btmsk = create_valid_bitmask(0, *valid_bits);
			// Last byte contains collision
			MFRC522_read_reg(me, FIFODataReg);
			// Apply bitmask to get rid of unwanted bits outside of valid bits
			me->Rx_buf &= valid_btmsk; 
			// Append decision bit
			me->Rx_buf |= (1<<((*valid_bits)));
			select_picc_buf[(*sel_buf_idx)++] = me->Rx_buf; 
			// Increment valid_bits to reflect appended decision bit

			// Add upper nibble of `valid_bytes` with 0x20, then add `valid_bits`
			// to lower nibble
			select_picc_buf[NVB_INDEX] = (0x20 + (*valid_bytes<<4)) | (*valid_bits);
			printf("anticollision NVB: %X\n", select_picc_buf[NVB_INDEX]);
			// Set TxLastBits to send valid bits of last byte
			set_reg_bits(me, BitFramingReg, *valid_bits);
			// Align to receive remaining bits of byte where coll occurred
			set_reg_bits(me, BitFramingReg, (*valid_bits<<BitFraming_RxAlign_Pos) );
			MFRC522_read_reg(me, BitFramingReg);
			printf("Bitframing Val: %X\n", me->Rx_buf);

			return COLL_OCCURRED;
	
		}
		return COLL_OCCURRED;
	}
	else{
		printf("NO Collision occurred\n");
		return (!COLL_OCCURRED);
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

void MFRC522_read_PICC(MFRC522_t *me, uint8_t block_addr, uint8_t *result)
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
		result[i] = me->Rx_buf;
	}
}

uint8_t MFRC522_write_PICC(MFRC522_t *me, uint8_t block_addr, uint8_t data_buf[18])
{
	uint8_t write_picc_buf[4] = {WRITE_PICC, block_addr};
	uint8_t received_AK =0, received_num_bits = 0;
	MFRC522_calc_CRC(me, write_picc_buf, 2, write_picc_buf+2);

	// Send PICC we want to write to block
	MFRC522_transeive(me, WRITE_PICC, write_picc_buf, 4);
	MFRC522_read_reg(me, FIFODataReg);
	received_AK = me->Rx_buf;
	// Verify ControlReg[2:0] is 4, meaning last byte written in FIFO has 4 bits
	MFRC522_read_reg(me, ControlReg); 
	received_num_bits = me->Rx_buf&Control_RxLastBits_Msk;
	if( (received_AK!=PICC_WRITE_ACK) || (received_num_bits!=4) ){
		me->status = MFRC522_ERROR;
		me->error = WRITE_PICC_FAILURE;
		printf("NAK: Failed to REQUEST write to block. NAK: %X, rcved bits: %X \n", received_AK, received_num_bits);
		return MFRC522_ERROR;
	}
	// Calc CRC and append to data_buf 
	MFRC522_calc_CRC(me, data_buf, 16, data_buf+16);

	// Now send buf of data destined for `block_addr`	
	MFRC522_transeive(me, WRITE_PICC, data_buf, 18);
	// Verify ControlReg[2:0] is 4 meaning last byte written in FIFO has 4 bits
	MFRC522_read_reg(me, FIFODataReg);
	received_AK = me->Rx_buf;
	MFRC522_read_reg(me, ControlReg); 
	received_num_bits = me->Rx_buf&Control_RxLastBits_Msk;

	if( (received_AK!=PICC_WRITE_ACK) || (received_num_bits!=4) ){
		me->status = MFRC522_ERROR;
		printf("Failed to WRITE to block. NAK\n");
		me->error = WRITE_PICC_FAILURE;
		return MFRC522_ERROR;
	}
	return MFRC522_OK;
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
uint8_t MFRC522_auth_PICC(MFRC522_t *me, uint8_t block_addr, uint8_t sector_key[6])
{
	uint8_t mf_auth_buf[12] = {0};
	uint8_t com_irq = 0;
	uint8_t status2 = 0;

	if( !(me->curr_picc.picc_state == PICC_ACTIVE) ){
		printf("MFAuth failed! PICC has not been selected\n");
		return MFRC522_ERROR;
	}
	MFRC522_flush_FIFO(me);
	MFRC522_clear_IRQ(me);

	MFRC522_write_reg(me, FIFODataReg, MFAUTH_KEYA);
	MFRC522_write_reg(me, FIFODataReg, block_addr);
	for(int i=0; i<6; i++){
		MFRC522_write_reg(me, FIFODataReg, sector_key[i]);
	}
	for(int i=0; i<4; i++){
		MFRC522_write_reg(me, FIFODataReg, me->curr_picc.picc_uid[i]);
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
	if(status2 & Status2_MFCrypto1On){
		printf("MFAuthent connection Successful\n");
		return MFRC522_OK;
	}
	if(com_irq & ComIrq_IdleIrq){
		printf("MFAuth completed and returned to IDLE\n");
		return MFRC522_OK;
	}
	if(com_irq & ComIrq_TimerIrq){
		printf("MFAuth timed out\n");
		return MFRC522_ERROR;
	}
	return MFRC522_ERROR;
}

/*
 * data_buf needs to include CMD to make it easier to represent all data for
 * a SEL command and so it's included in buffer when calculating CRC
 * One thing is to calculate CRC we need to write the same data to FIFO,
 * ideally we just write data to FIFO once, calc CRC
 * TODO: Calc CRC here, have argument to determine if CRC is required 
 */
void MFRC522_transeive(MFRC522_t *me, PCD_CMD_t cmd, uint8_t *data_buf, uint8_t data_buf_len)
{
	uint8_t irq_val = 0;
	MFRC522_flush_FIFO(me);
	MFRC522_clear_IRQ(me);

	MFRC522_read_reg(me, FIFOLevelReg);
	//printf("Before Transceive, Fifo level: %i\n", me->Rx_buf);

	//Check if data is not null, then write to FIFO
	if(data_buf){
		printf("Transeive data:\n");
		for(int i=0; i<data_buf_len; i++){
			printf("%i: %X\n", i, data_buf[i]);
			MFRC522_write_reg(me, FIFODataReg, data_buf[i]);
		}
	}
	else{
		printf("Invalid `data_buf` for transeive function\n");
		me->status = MFRC522_ERROR;
		me->error = PCD_TRANSIEVE_FAILURE;
		return;
	}
	//write transceive command, don't set bits, that causes failure of transmission
	MFRC522_write_reg(me, CommandReg,Transceive);
	//write `start` bit within BitFramingReg to start sending data
	set_reg_bits(me, BitFramingReg, STARTSEND);
	//wait for transceive command to finish, indicated with either RxIRQ, IdleIrq or TimerIRQ
	do{
		MFRC522_read_reg(me, ComIrqReg);
		irq_val = me->Rx_buf;
	}while(!(irq_val & (ComIrq_RxIrq|ComIrq_IdleIrq|ComIrq_TimerIrq) ));

	if(irq_val & ComIrq_RxIrq)
		printf("Rx IRQ received\n");
	if(irq_val & ComIrq_IdleIrq)
		printf("Idle IRQ received\n");
	if(irq_val & ComIrq_TimerIrq){
		//printf("Timer timed out received\n");
		me->status = MFRC522_ERROR;
		me->error = PCD_TRANSIEVE_TIMEOUT;
		return;
	}
	// Don't want to return for error since collisions cause error IRQ from PCD
	if(irq_val & ComIrq_ErrIrq){
		MFRC522_read_reg(me, ErrorReg);
		printf("Error reg: %X\n", me->Rx_buf);
		me->status = MFRC522_ERROR;
		me->error = PCD_TRANSIEVE_ERROR;
	}
	else {
		me->status = MFRC522_OK;
	}
	MFRC522_read_reg(me, RxModeReg);
	printf("irq_val: %X, RxModeReg: %X\n", irq_val, me->Rx_buf);
	
	MFRC522_read_reg(me, FIFOLevelReg);
	printf("Completed Transceive, Fifo level: %X\n", me->Rx_buf);
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

