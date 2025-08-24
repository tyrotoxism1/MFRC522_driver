#include "MFRC522.h"
#include "SPI.h"
#include <stdint.h>
#include "printf.h"
#include "stm32f446xx.h"
 
volatile uint8_t timer7_done = 0;

void TIM7_IRQHandler()
{
	timer7_done = 1;
	GPIOA->ODR |= GPIO_ODR_OD5;
	TIM7->SR &= ~(TIM_SR_UIF);
}

/**
 * _timer7_init() - Configures basic timer 7 for one pulse mode for delay
 * timing use.
 *
 * Timer 7 is configured to count at 1MHz up to ARR(set in `delay()`), sending an
 * interrupt to update a global variable indicating timer no longer in use. The timer is shared
 * across MFRC522 objects, thus if timer is in use, another MFRC522 object
 * cannot use delay. Timer prescalar is set to 15 assuming 17MHz input clock
 * source to attain 1ms resolution of clock counter.   
 */
void _timer7_init()
{
	RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;	
	TIM7->CR1 |= TIM_CR1_OPM; 
	TIM7->DIER |= TIM_DIER_UIE;
	TIM7->PSC |= 15;
}


/**
 * is_initialized() - Helper function to check if `MFRC522_init` has been
 * called where status is set not UNKOWN state.
 *
 * Return: 0 if status is UNKOWN(0), 1 otherwise
 */
uint8_t is_initialized(MFRC522_t *me)
{
	if( !(me->status) ){
		me->status = MFRC522_ERROR;
		me->error = UNINITIALIZED;
		return 0;
	}
	return 1;
}

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

void MFRC522_init(MFRC522_t *me)
{
	SPI_init(0, 1, 0, 0);
	SPI_disable();
	me->status = IDLE;
	me->error = NO_ERROR;
	_EXTI0_init();
	_timer7_init();
	MFRC522_write_reg(me, ComIEnReg, MFRC522_TimerIEn);
	GPIOA->ODR &= ~(GPIO_ODR_OD5);

}



void MFRC522_deinit(MFRC522_t *me)
{
	SPI_deinit();
}

/**
 * addr_trans() - Helper function to translates register address to read address
 *
 * When reading from MFRC522 via SPI, the MSB defines the r/w mode and the LSB
 * is reserved. The register addresses only go up to 3Fh leaving 2 bits of room
 * for mode and reserved bit. Function shifts `addr` right one, then writes MSB
 * based on `rw_mode`.
 *
 * @addr: Desired MFRC522 register address 
 * @rw_mode: Value of 0 means read mode, otherwise write mode
 *
 * Return: Value to send over SPI to read desired base `addr` 
 */
uint8_t addr_trans(uint8_t addr, uint8_t rw_mode)
{
	addr = (addr<<1);
	if(rw_mode==0)
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
	uint8_t val = 0;
	if( !(is_initialized(me)) )
		return 0;

	SPI_enable();
	SPI_chip_select(0);
	SPI_single_transieve_poll(addr_trans(reg,READ));
	SPI_single_transieve_poll(addr_trans(0xFF ,READ));
	val = SPI_get_Rx_buf();
	SPI_chip_deselect();
	SPI_disable();

	return val;

}

void MFRC522_write_reg(MFRC522_t *me, PCD_reg reg, uint8_t data)
{
	if( !(is_initialized(me)) )
		return;
	
	SPI_enable();
	SPI_chip_select(0);
	SPI_single_transieve_poll(addr_trans(reg,WRITE));
	SPI_single_transieve_poll(data);

	SPI_chip_deselect();
	SPI_disable();
}

void MFRC522_write_cmd(MFRC522_t *me, MFRC522_cmd cmd)
{
	if( !(is_initialized(me)) )
		return;
	MFRC522_write_reg(me, CommandReg, cmd);
}	

void MFRC522_soft_reset(MFRC522_t *me)
{
	if( !(is_initialized(me)) )
		return;
	MFRC522_write_cmd(me, SoftReset);
	// Dummy delay for reset, not sure if timer should be used or not
	for(int i=0; i<155000; i++);
}

void MFRC522_flush_FIFO(MFRC522_t *me)
{
	if( !(is_initialized(me)) )
		return;
	MFRC522_write_reg(me, FIFOLevelReg, 0x80);
}

/**
 * MFRC522_delay() - Assigns `delay` to timer6 ARR, starts timer then waits
 * for timer_done global variable before returning to calling function 
 * 
 * Timer6 is configured to count at 1MHz so `delay` is directly assigned to
 * auto-reload register (ARR). 
 *
 * @delay: desired wait time in milliseconds
 *
 * Return: 0 on failure, 1 on success
 */
uint8_t MFRC522_delay(MFRC522_t *me, uint32_t delay)
{
	if( !(is_initialized(me)) )
		return 0;
	TIM7->ARR = delay;
	//TIM7->EGR |= TIM_EGR_UG;
	TIM7->CR1 |= TIM_CR1_CEN;
	while(!timer7_done);
	timer7_done = 0;

	return 1;	
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
	uint8_t irq; 
	uint8_t FIFO_level = 0;
	if( !(is_initialized(me)) )
		return 0;
	MFRC522_write_reg(me, ComIrqReg, 0);
	MFRC522_soft_reset(me);
	//TODO: Add timeout to break out of while loop
	do{
		irq = MFRC522_read_reg(me, ComIrqReg);
		printf("IRQ: %i\n", irq);
	} while( !(irq & MFRC522_IRQ_IDLE) );

	printf("Command: %i\n", tmp_val);
	MFRC522_flush_FIFO(me);
	for(int i=0; i<25; i++){
		MFRC522_write_reg(me, FIFODataReg, 0x00);
	}
	MFRC522_write_cmd(me, Mem);
	MFRC522_write_reg(me, AutoTestReg, 0x09);
	MFRC522_write_reg(me, FIFODataReg, 0x00);
	MFRC522_write_cmd(me, CalcCRC);
	while( MFRC522_read_reg(me, FIFOLevelReg) < 64 );
	for(int i=0; i<64; i++){
		if(i%8==0)
			printf("\n");
		tmp_val = MFRC522_read_reg(me, FIFODataReg);
		printf("%X, ", tmp_val);
	}
	printf("\n");
	MFRC522_write_cmd(me, Idle);
	MFRC522_write_reg(me, AutoTestReg, 0x0);


	return 0;
}
