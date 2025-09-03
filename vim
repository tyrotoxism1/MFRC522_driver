diff --git a/src/MFRC522.c b/src/MFRC522.c
index eaaf60d..a3098c0 100644
--- a/src/MFRC522.c
+++ b/src/MFRC522.c
@@ -1,8 +1,4 @@
 #include "MFRC522.h"
-#include "Legacy/stm32_hal_legacy.h"
-#include "stm32f4xx_hal_gpio.h"
-#include "stm32f4xx_hal_spi.h"
-#include <cstdint>
 
 /**
  * _EXTI0_init() - Sets up EXTI0 of STM32F446 to wait for rising edge of MFRC522 interrupts
@@ -30,14 +26,7 @@ void GPIO_init()
 	HAL_GPIO_Init(GPIOA, &led);
 }
 
-/**
- * MFRC522_init() - Initializes SPI, GPIO and basic status of MFRC522 struct
- *
- * Return: Status of initialization
- * 0 = Success
- * -1 = SPI failure  
- */
-int MFRC522_init(MFRC522_t *me)
+void MFRC522_SPI_init(MFRC522_t *me)
 {
 	me->hspi.Instance = SPI1;
  	me->hspi.Init.Mode = SPI_MODE_MASTER;
@@ -52,7 +41,18 @@ int MFRC522_init(MFRC522_t *me)
 	me->hspi.Init.TIMode = SPI_TIMODE_DISABLED; 
 	me->hspi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED; 
 	me->hspi.State = HAL_SPI_STATE_RESET;
+}
 
+/**
+ * MFRC522_init() - Initializes SPI, GPIO and basic status of MFRC522 struct
+ *
+ * Return: Status of initialization
+ * 0 = Success
+ * -1 = SPI failure  
+ */
+int MFRC522_init(MFRC522_t *me)
+{
+	MFRC522_SPI_init(me);
 	GPIO_init();
 	if( HAL_SPI_Init(&(me->hspi)) != HAL_OK)
 		return -1;
@@ -90,13 +90,13 @@ uint8_t is_initialized(MFRC522_t *me)
  *
  * When interacting with MFRC522 via SPI, the MSB defines the r/w mode and the LSB
  * is reserved. The register addresses only go up to 3Fh leaving 2 bits of room
- * for mode and reserved bit. Function shifts `addr` right one, then writes MSB
+ * for mode and reserved bit. Function shifts `addr` right 1, then writes MSB
  * based on `rw_mode`.
  *
  * @addr: Desired MFRC522 register address 
- * @rw_mode: Value of 0 means read mode, otherwise write mode
+ * @rw_mode: 0=read, 1=write
  *
- * Return: Value to send over SPI to read desired base `addr` 
+ * Return: Converted MFRC522 register address 
  */
 uint8_t addr_trans(uint8_t addr, uint8_t rw_mode)
 {
@@ -116,28 +116,37 @@ uint8_t addr_trans(uint8_t addr, uint8_t rw_mode)
  */
 uint8_t MFRC522_read_reg(MFRC522_t *me, PCD_reg reg)
 {
-	uint8_t rx[2];	
 	uint8_t tx[2];	
+	uint8_t rx[2];	
+
+	if( !(is_initialized(me)) )
+		return me->status;
 
 	tx[0] = addr_trans(reg, READ);
 	tx[1] = 0x00;
 
-	if( !(is_initialized(me)) )
-		return 0;
-
 	HAL_GPIO_WritePin(GPIOB, CSS_PIN, GPIO_PIN_RESET);
 	if(HAL_SPI_TransmitReceive( &(me->hspi), tx, rx, 2, SPI_TIMEOUT) != HAL_OK){
 		printf("read reg failed\n");
 		HAL_GPIO_WritePin(GPIOB, CSS_PIN, GPIO_PIN_SET);
-		return 0;
+		me->status = MFRC522_ERROR; 
+		me->error = READ_REG_FAILURE; 
+		return MFRC522_ERROR;
 	}
 
 	HAL_GPIO_WritePin(GPIOB, CSS_PIN, GPIO_PIN_SET);
-	if(me->hspi.State != HAL_SPI_STATE_ERROR)
-		return rx[1];
+	if(me->hspi.State != HAL_SPI_STATE_ERROR){
+		me->Rx_buf = rx[1];
+		me->status = MFRC522_OK;	
+		printf("Read reg val: %X\n", rx[1]);
+		return MFRC522_OK;
+	}
 	//Might want to error handle here and not return 0, as 0 is a valid value
-	else
-		return 0; 
+	else{
+		me->status = MFRC522_ERROR;
+		me->error = READ_REG_FAILURE;
+		return MFRC522_ERROR; 
+	}
 }
 
 uint8_t MFRC522_write_reg(MFRC522_t *me, PCD_reg reg, uint8_t data)
@@ -155,7 +164,7 @@ uint8_t MFRC522_write_reg(MFRC522_t *me, PCD_reg reg, uint8_t data)
 		return 0;
 	}
 	HAL_GPIO_WritePin(GPIOB, CSS_PIN, GPIO_PIN_SET);
-	return ;
+	return 0;
 }
 
 void MFRC522_write_cmd(MFRC522_t *me, MFRC522_cmd cmd)
@@ -234,3 +243,7 @@ uint8_t MFRC522_self_test(MFRC522_t *me)
 }
 
 
+uint8_t MFRC522_get_rx_buf(MFRC522_t *me)
+{
+	return me->Rx_buf; 
+}
