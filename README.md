# MFRC522_driver
MFRC522 wireless reader/writer peripheral driver written for STM32F4xx host to control and communicate via SPI

# Hardware pinout
| STM Pin  | MFRC Pin | Description           |
| -------- | -------- | --------------------- |
| PB8(D15) | SDA      | SPI CSS pin           |
| PB4(D5)  | MISO     | SPI MISO              |
| PB5(D4)  | MOSI     | SPI MOSI              |
| PB3(D3)  | SCK      | SPI clock             |
| 3V3      | 3.3V     | Power                 |
| GND      | GND      | Reference GND         |
| PA0(A0)  | IRQ      | EXTI0 interrupt line  |



# MFRC522 Timer
- Input clock speed of 13.56MHz
- 12-bit prescalar and 16-bit reload
    - presclar max = 4095, reload max = 65535 
- time delay equation is :
$$
t_{d1} = \frac{(TPrescalar\times2+1)\times(TReloadVal+1)}{(13.56MHz)}
$$
- So max delay is 39.59 seconds
- Smallest delay would be 14.7uS
	- Though due to SPI communication, would not be able to reliable poll and check timer being done in short amount of time
## Configuration
- Focusing on just delay for now, configuration includes:
	- TModeReg 
		- TAutoRestart
			- 1 -> Counter resets to 0 after reaching TReload value
			- 0 -> Counter decrements to 0 
		- TPreScalar_Hi
			- Defines the upper 4 bits of the prescalar
	- TPrescalarReg
		- Defines the lower 8 bits of the prescalar
	- TReloadReg
		- 2 registers (hi and low) that define the reload value
    - ControlReg
        - TStartNow, manually starts timer 
        - TStopNow, stops timer (crazy right)
- To create a delay the Autorestart should be set, then based on the value passed to the function, set the prescalar and reload value

## Calculating AutoReload and Prescalar
- The value passed to the delay function should be in milliseconds
- Uses provided delay formula divided by 1000 to convert from seconds to milliseconds
- By default the TPrescalarReg will be set to 2000
	- This keeps calculations simple with a set variable and just solving for the reload value
    - This gives a delay range of 295us to 19s
	- The reload register has wider range so it's easier to keep that changeable and have prescalar be constant
	- This gives the following equation:
$$
TReloadVal = \frac{t_{d1}*13.56\times10^6}{1000*(2000*2+1)} = t_{d1}*3.389-1
$$
- So for example if we want a delay of 25ms:
$$
TReloadVal = 25*3.389-1 = 83.73
$$
- Plugging into the original $t_{d1}$ equation 
$$
t_{d1} = \frac{(2000\times2+1)\times(83.73+1)}{(13.56MHz)} = 25\times10^{-3}
$$

