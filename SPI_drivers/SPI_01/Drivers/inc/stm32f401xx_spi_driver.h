
/*
 *  SPI drive features for this application
 *  1. SPI device mode
 *  2. SPI bus configuration
 *  3. SPI DFF
 *  4. SPI CPHA
 *  5. SPI CPOL
 *  6. SPI SSM
 *  7. SPI speed
 *
 *  The header file includes the structure for the features.
 */

#ifndef  INC_STM32F401XX_SPI_DRIVER_H_
#define  INC_STM32F401XX_SPI_DRIVER_H_

#include "stm32f401xx.h"
/*
 * Configuration structure for SPIx peripheral
 */
typedef struct
{
	uint8_t SPI_DeviceMode;              /* @GPIO_PIN_NUMBERS */
	uint8_t SPI_BusConfig;                /* @GPIO_PIN_MODES */
	uint8_t SPI_SclkSpeed;               /* @GPIO_PIN_SPEED */
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
}SPI_Config_t;

/*
 * Handle structure for SPIx peripheral
 */
typedef struct
{
	SPI_RegDef_t                  *pSPIx;  // This hold base address of SPIx
	SPI_Config_t                  SPIConfig;  // This hold SPIx
}SPI_Handle_t;


/*
 * Handle structure for SPIx peripheral
 */

/*
 * @SPI Device Mode
 * Ref: dm00096844, 20.3, 20.5.1
 */
#define SPI_DEVICE_MODE_MASTER	1
#define SPI_DEVICE_MODE_SLAVE	0

/*
 * @SPI Device Mode
 * Ref: dm00096844, 20.3.2, 20.3.5
 */
#define SPI_BUS_CONFIG_FD	1
#define SPI_BUS_CONFIG_HD   2
#define SPI_BUS_CONFIG_SIMPLEX_TXONLY	3
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY	4

/*
 * @SPI SclkSpeed
 * Ref: dm00096844, 20.5.1
 */
#define SPI_SCLK_SPEED_DIV2			0
#define SPI_SCLK_SPEED_DIV4			1
#define SPI_SCLK_SPEED_DIV8			2
#define SPI_SCLK_SPEED_DIV16		3
#define SPI_SCLK_SPEED_DIV32		4
#define SPI_SCLK_SPEED_DIV64		5
#define SPI_SCLK_SPEED_DIV128		6
#define SPI_SCLK_SPEED_DIV256		7

/*
 * @SPI DFF
 * Ref: dm00096844, 20.3.1, 20.3.2
 */
#define SPI_DFF_8BITS  	0
#define SPI_DFF_16BITS	1

/*
 * @SPI CPOL
 * Ref: dm00096844, 20.3.1
 */
#define SPI_CPOL_HIGH	1
#define SPI_CPOL_LOW	0

/*
 * @SPI CPHA
 * Ref: dm00096844, 20.3.1
 */
#define SPI_CPHA_HIGH	1
#define SPI_CPHA_LOW	0

/*
 * @SPI SSM
 * Ref: dm00096844, 20.3.1
 */
#define SPI_SSM_EN	1
#define SPI_SSM_DI	0

/*
 * @SPI CR2
 * Ref: dm00096844, 20.5.2
 */
#define SPI_CR2_RXDMAEN		0
#define SPI_CR2_TXDMAEN		1
#define SPI_CR2_SSOE		2
#define SPI_CR2_FRF			4
#define SPI_CR2_ERRIE		5
#define SPI_CR2_RXEIE		6
#define	SPI_CR2_TXEIE		7

/*
 * @SPI SR
 * Ref: dm00096844, 20.5.3
 */
#define SPI_SR_RXNE		0
#define SPI_SR_TXE		1
#define SPI_SR_CHIDE	2
#define SPI_SR_UDR		3
#define SPI_SR_CRCERR	4
#define SPI_SR_MODF		5
#define SPI_SR_OVR		6
#define SPI_SR_BSY		7
#define SPI_SR_FRE		8
/*
 * @SPI related status flags definitions
 * Ref: dm00096844,
 */

#define SPI_TXE_FLAG	(1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG	(1 << SPI_SR_RXNE)
#define SPI_BUSY_FLAG	(1 << SPI_SR_RXNE)

/* APIs supported by this driver
 * For more information about the APIs chek the function definitions
 */

/*
 * Peripheral Clock Setup
 */
void SPI_PeriClkCtrl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

/*
 * Init and De-init
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);



/*
 * Data Send and Receive
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Le);



/*
 * IRQ configuration and ISR handling
 */
void SPI_IRQITConfig(uint8_t IRQNumber,  uint8_t EnorDi);
void SPI_IRQHandling(SPI_Handle_t *pHandle);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);


/*
 * Other Peripheral Control APIs
 */
void SPI_PeripheralCtrl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);

#endif
