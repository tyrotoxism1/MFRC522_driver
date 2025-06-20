#ifndef MFRC522_H
#define MFRC522_H
#include <stdint.h>
#include "stm32f446xx.h"
#include "SPI.h"


// Defined in section 9 of MFRC522 datasheet
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
	Idle,
	Mem,
	GenerateRandomID,
	CalcCRC,
	Transmit,
	NoCmdChange,
	Receive,
	Transceive,
	Reserved,
	MFAuthent,
	SoftReset
} MFRC522_cmd;

typedef enum {
	UNKOWN,
	IDLE,
	MFRC522_ERROR,
} MFRC522_status;

typedef enum MFRC522_error {
	NO_ERROR,
	UNINITIALIZED,
	SELF_TEST_FAILED,
} MFRC522_error;

typedef struct MFRC522_t{
	MFRC522_status status;
	MFRC522_error error;
	
} MFRC522_t;

void MFRC522_init(MFRC522_t *me);
void MFRC522_deinit(MFRC522_t *me);
uint8_t MFRC522_read_reg(MFRC522_t *me, PCD_reg reg);
void MFRC522_write_reg(MFRC522_t *me, PCD_reg reg, uint8_t data);
uint8_t MFRC522_addr_trans(uint8_t addr, uint8_t rw_mode);
void MFRC522_soft_reset(MFRC522_t *me);
uint8_t MFRC522_self_test(MFRC522_t *me);

#endif //MFRC522_H
