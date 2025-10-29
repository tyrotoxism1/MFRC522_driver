#ifndef MFRC522_H
#define MFRC522_H
#include <stdint.h>

#include "Legacy/stm32_hal_legacy.h"
#include "printf.h"
#include "stm32f446xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_def.h"
#include "stm32f4xx_hal_spi.h"
#include "stm32f4xx_hal_gpio.h"


/*--------------------- MFRC522 Register definitions ---------------------*/
// CommandReg
#define CMDREG_RcvOff (1<<5)
#define CMDREG_PWRDWN (1<<4)

// Status1Reg
#define Status1Reg_LoAlert 1U 
#define Status1Reg_HiAlert (1<<1) 
#define Status1Reg_TRunning (1<<3) 
#define Status1Reg_Irq (1<<4) 
#define Status1Reg_CRCReady (1<<5) 
#define Status1Reg_CRCOk (1<<6) 

// Status2Reg
#define Status2_MFCrypto1On (1<<3)
#define Status2_I2CForceHS (1<<6)
#define Status2_TempSensClear (1<<7)

// TModeReg
#define TModeReg_TAutoRestart (1<<4) 
#define TModeReg_TAuto (1<<7) 

// ComIEnReg - reg to control enable/disable interrupt requests
#define ComIEnReg_TimerIEn 1U
#define ComIEnReg_ErrIEn (1<<1) 
#define ComIEnReg_LoAlertIEn (1<<2) 
#define ComIEnReg_HiAlertIEn (1<<3) 
#define ComIEnReg_IdleIEn (1<<4) 
#define ComIEnReg_RxIEn (1<<5) 
#define ComIEnReg_TxIEn (1<<6) 
#define ComIEnReg_IRqInv (1<<7) 

// ComIrqReg
#define ComIrq_TimerIrq 1U
#define ComIrq_ErrIrq (1<<1) 
#define ComIrq_LoAlertIrq (1<<2) 
#define ComIrq_HiAlertIrq (1<<3) 
#define ComIrq_IdleIrq (1<<4) 
#define ComIrq_RxIrq (1<<5) 
#define ComIrq_TxIrq (1<<6) 
#define ComIrq_Set1 (1<<7) 

//DivIrqReg
#define DivIrq_CRCIrq (1<<2)
#define DivIrq_MfinActIrq (1<<4)
#define DivIrq_Set2 (1<<7)

// ErrorReg
#define Error_ProtocolErr 1U
#define Error_ParityErr (1<<1)
#define Error_CRCErr (1<<2) 
#define Error_CollErr (1<<3) 
#define Error_BufferOvfl (1<<4) 
#define Error_TempErr (1<<6) 
#define Error_WrErr (1<<7) 


// ControlReg
#define Control_TStartNow (1<<6)
#define Control_TStopNow (1<<6)

// TxControlReg
#define TxControl_Tx1RFEn 1U
#define TxControl_Tx2RFEn (1<<1) 
#define TxControl_Tx2CW (1<<3) 
#define TxControl_InvTx1RFOff (1<<4) 
#define TxControl_InvTx2RFOff (1<<5) 
#define TxControl_InvTx1RFon (1<<6) 
#define TxControl_InvTx2RFon (1<<7) 

// FIFOLevelReg
#define FIFOLevel_FlushBuffer (1<<7)

// CollReg - Defines first bit collisions detected on RF interface 
#define Coll_CollPosNotValid (1<<5)
#define Coll_ValuesAfterColl (1<<7)
#define Coll_CollPos_Msk (0x1F)

// BitFramingReg
#define BitFraming_StartSend (1<<7)
#define BitFraming_TxLastBits_Msk (0x07)
#define BitFraming_RxAlign_Msk (0x70)
#define BitFraming_RxAlign_Pos (4)

/*--------------------- STM32 GPIO Definitions ---------------------*/
#define SCK_PIN GPIO_PIN_3
#define MISO_PIN GPIO_PIN_4
#define MOSI_PIN GPIO_PIN_5
#define CSS_PIN GPIO_PIN_8


/*--------------------- Collision Algo Definitions ---------------------*/
//Collision algo adjacent defines like select buffer for transmission 
#define SELECT_CL1 0x93
#define SELECT_CL2 0x95
#define SELECT_CL3 0x97
#define SEL_INDEX 0
#define NVB_INDEX 1
#define COLL_OCCURRED 1 


/*--------------------- MISC Definitions ---------------------*/
#define STARTSEND (1<<7)
#define READ 0
#define WRITE 1
#define SPI_TIMEOUT 1000U
#define MASTER_BOARD
#define SEL_NUM_BYTES 9
#define MFRC522_TimerPrescalar 2000U
#define MFRC522_TimerPreSc_HiNib (MFRC522_TimerPrescalar >> 8) 
#define MFRC522_TimerPreSc_Lo 208U 
#define CLEAR_IRQ_REG 0x7F




// MFRC522 registers defined in section 9 of MFRC522 datasheet
typedef enum {
	Reserved0			= 0x00,
	CommandReg			= 0x01,
	ComIEnReg			= 0x02,
	DivIEnReg			= 0x03,
	ComIrqReg			= 0x04,
	DivIrqReg			= 0x05,
	ErrorReg			= 0x06,
	Status1Reg			= 0x07,
	Status2Reg			= 0x08,
	FIFODataReg			= 0x09,
	FIFOLevelReg		= 0x0a,
	WaterLevelReg		= 0x0b,
	ControlReg			= 0x0c,
	BitFramingReg		= 0x0d,
	CollReg				= 0x0e,
	Reserved1			= 0x0f,
	Reserved2			= 0x10,
	ModeReg				= 0x11,
	TxModeReg			= 0x12,
	RxModeReg			= 0x13,
	TxControlReg		= 0x14,
	TxASKReg			= 0x15,
	TxSelReg			= 0x16,
	RxSelReg			= 0x17,
	RxThresholdReg		= 0x18,
	DemodReg			= 0x19,
	Reserved3			= 0x1a,
	Reserved4			= 0x1b,
	MfTxReg				= 0x1c,
	MfRxReg				= 0x1d,
	Reserved5			= 0x1e,
	SerilSpeedReg		= 0x1f,
	Reserved6			= 0x20,
	CRCResultMSBReg		= 0x21,
	CRCResultLSBReg		= 0x22,
	Reserved7			= 0x23,
	ModWidthReg			= 0x24,
	Reserved8			= 0x25,
	RFCfgReg			= 0x26,
	GsNReg				= 0x27,
	CWGsPReg			= 0x28,
	ModGsPReg			= 0x29,
	TModeReg			= 0x2a,
	TPrescalarReg		= 0x2b,
	TReloadHiReg		= 0x2c,
	TReloadLoReg		= 0x2d,
	TCounterValHiReg	= 0x2e,
	TCounterValLoReg	= 0x2f,
	Reserved9			= 0x30,
	TestSel1Reg			= 0x31,
	TestSel2Reg			= 0x32,
	TestPinEnReg		= 0x33,
	TestPinValueReg		= 0x34,
	TestBustReg			= 0x35,
	AutoTestReg			= 0x36,
	VersionReg			= 0x37,
	AnalogTestReg		= 0x38,
	TestDAC1Reg			= 0x39,
	TestDAC2Reg			= 0x3a,
	TestADCReg			= 0x3b,
	Reserved10			= 0x3c,
	Reserved11			= 0x3d,
	Reserved12			= 0x3e,
	Reserved13			= 0x3f
} PCD_reg;



typedef enum MFRC522_cmd{
	Idle = 0b0000,
	Mem = 0b0001,
	GenerateRandomID = 0b0010,
	CalcCRC = 0b0011,
	Transmit = 0b0100,
	NoCmdChange = 0b0111,
	Receive = 0b1000,
	Transceive = 0b1100,
	Reserved = 0b1101,
	MFAuthent = 0b1110,
	SoftReset = 0b1111
} MFRC522_cmd;

typedef enum {
	MFRC522_OK,
	UNKOWN,
	IDLE,
	MFRC522_ERROR,
} MFRC522_status;

typedef enum MFRC522_error {
	NO_ERROR,
	UNINITIALIZED,
	SELF_TEST_FAILED,
	TIMER_ALRDY_RUNNING,
	READ_REG_FAILURE,
	WRITE_REG_FAILURE,
	PCD_TRANSEIVE_FAILURE,
	FUNC_ASSERT_FAILURE,
} MFRC522_error;

typedef enum MFRC522_timer_t {
	TIMER_UNINITIALIZED,
	TIMER_INACTIVE,
	TIMER_ACTIVE,
	TIMER_DONE
} MFRC522_timer;

typedef enum PCD_CMD_t {
	REQA = 0x26,
	WUPA = 0x52,
	SELECT = 0x93,
	HLTA = 0x50, 
	RATS = 0xE0,
} PCD_CMD;

typedef enum PICC_CMD_t{
	READ_PICC = 0x30,
	MFAUTH_KEYA = 0x61,
	MFAUTH_KEYB = 0x62,
	WRITE_PICC = 0xA0,
	DECREMENT_PICC = 0xC0,
	INCREMENT_PICC = 0xC1,
	RESTORE_PICC = 0xC2,
	TRANSFER_PICC = 0xB0,
} PICC_CMD;


typedef struct MFRC522_t{
	MFRC522_status status;
	MFRC522_timer timer;
	MFRC522_error error;
	SPI_HandleTypeDef hspi;
	uint8_t Rx_buf;
	uint8_t Tx_buf;
} MFRC522_t;

void clear_reg_bits(MFRC522_t *me, PCD_reg reg, uint8_t clear_bitmask);
void set_reg_bits(MFRC522_t *me, PCD_reg reg, uint8_t set_bitmask);
void MFRC522_SPI_init(MFRC522_t *me);
int MFRC522_init(MFRC522_t *me);
void GPIO_init();
void MFRC522_deinit(MFRC522_t *me);
uint8_t MFRC522_read_reg(MFRC522_t *me, PCD_reg reg);
uint8_t MFRC522_write_reg(MFRC522_t *me, PCD_reg reg, uint8_t data);
uint8_t MFRC522_addr_trans(uint8_t addr, uint8_t rw_mode);
void MFRC522_soft_reset(MFRC522_t *me);
void MFRC522_flush_FIFO(MFRC522_t *me);
void MFRC522_clear_FIFO(MFRC522_t *me);
void MFRC522_fifo_read_stream(MFRC522_t *me, uint8_t *buf, uint8_t buf_len, uint8_t print);
void MFRC522_fifo_write_stream(MFRC522_t *me, uint8_t *buf, uint8_t buf_len);
void MFRC522_calc_CRC(MFRC522_t *me, uint8_t *data, uint8_t data_size, uint8_t *result);
uint8_t MFRC522_self_test(MFRC522_t *me);
void MFRC522_TxEnable(MFRC522_t *me);

void MFRC522_REQA(MFRC522_t *me);
void MFRC522_SEL(MFRC522_t *me, uint8_t *uid_buf );
void MFRC522_CL1(MFRC522_t *me, uint8_t *res_buf);
void MFRC522_read_PICC(MFRC522_t *me, uint8_t block_addr);
void MFRC522_auth_PICC(MFRC522_t *me, uint8_t block_addr, uint8_t sector_key[6], uint8_t serial_num[4]);
void MFRC522_transeive(MFRC522_t *me, uint8_t cmd, uint8_t *data_buf, uint8_t data_buf_len);

void MFRC522_clear_IRQ(MFRC522_t *me);
void MFRC522_stop_encrypt_comm(MFRC522_t *me);


uint8_t MFRC522_collision_check(MFRC522_t *me, uint8_t *select_picc_buf, uint8_t *sel_buf_idx, uint8_t* buf_len, uint8_t *valid_bits,uint8_t *valid_bytes);

void MFRC522_select_PICC(MFRC522_t *me);

#endif //MFRC522_H
