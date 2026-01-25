/*
 * stm32f401xx_gpio.c
 *
 *  Created on: Sep 4, 2025
 *      Author: flyingbean
 */

#include "stm32f401xx_spi_driver.h"
/*
 * Peripheral Clock setup
 */
/***********************************************************
 * @fn				- SPI_PeriClkCtrl
 *
 * @brief           - Enables or disables peripheral clock for the specified SPI port
 *                   (Required before SPI_Init can configure the peripheral)
 *
 * @param[in]       - pSPIx: Base address of the SPI peripheral (SPI1, SPI2, SPI3, or SPI4)
 * @param[in]       - EnorDi: ENABLE to turn on clock, DISABLE to turn off
 *
 * @return          - None
 *
 * @Note            - Must be called before SPI_Init()
 *                   Uses RCC (Reset and Clock Control) to enable SPI clock
 ***********************************************************/
void SPI_PeriClkCtrl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	// Enable the clock for the specified SPI peripheral
	if(EnorDi == ENABLE)
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_EN();      // Enable SPI1 clock
		}
		else if (pSPIx == SPI2)
		{
			SPI2_PCLK_EN();      // Enable SPI2 clock
		}
		else if (pSPIx == SPI3)
		{
			SPI3_PCLK_EN();      // Enable SPI3 clock
		}
		else if (pSPIx == SPI4)
		{
			SPI4_PCLK_EN();      // Enable SPI4 clock
		}
	}
	else
	{
		// TODO: Implement clock disable functionality

	}

}

/*
 * Init and De-init
 */
/***********************************************************
 * @fn				- SPI_Init
 *
 * @brief           - Initializes SPI peripheral with the specified configuration
 *                   Configures device mode, bus mode, clock speed, data format, clock polarity/phase, and SSM
 *
 * @param[in]       - pSPIHandle: Pointer to SPI_Handle_t containing port and configuration
 *
 * @return          - None
 *
 * @Note            - Reference: dm00096844, 20.5.1, SPI CR1 register
 *                   Must call SPI_PeriClkCtrl() first to enable the peripheral clock
 ***********************************************************/
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	uint32_t tempreg = 0;  // Temporary register for bit manipulation

	// Step 1: Enable peripheral clock for this SPI port
	SPI_PeriClkCtrl(pSPIHandle->pSPIx, ENABLE);

	// Step 2: Configure the SPI device mode
	// Selects whether this device is MASTER or SLAVE
	tempreg |=  pSPIHandle->SPIConfig.SPI_DeviceMode  << SPI_CP1_MSTR ;

	// Step 3: Configure the bus configuration
	// Determines unidirectional or bidirectional communication
	
    if(pSPIHandle->SPIConfig.SPI_BusConfig  == SPI_BUS_CONFIG_FD )
       {
    	   // Full-duplex mode: simultaneous TX and RX
    	   // BIDIMODE bit should be cleared
    	   tempreg &=  ~(1 << SPI_CP1_BIDIMODE) ;

       }else if ( pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD )
       {
    	   // Half-duplex mode: TX and RX on same line, not simultaneously
    	   // BIDIMODE bit should be set
    	   tempreg |= (1 << SPI_CP1_BIDIMODE) ;
       }else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
       {
    	   // Simplex mode: receive only
    	   // BIDIMODE bit should be cleared
    	   tempreg &=  ~(1 << SPI_CP1_BIDIMODE) ;
    	   // RXONLY bit must be set
    	   tempreg |= (1 << SPI_CP1_RXONLY) ;
       }

    // Step 4: Configure SPI SCLK speed (baud rate)
    // Determines how fast the clock runs (affects data transmission speed)
    tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CP1_BR;

    // Step 5: Configure SPI DFF (Data Frame Format)
    // Determines if data is 8-bit or 16-bit frames
    tempreg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CP1_DFF;

    // Step 6: Configure SPI CPOL (Clock Polarity)
    // Determines if clock idles HIGH or LOW
    tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CP1_CPOL;

    // Step 7: Configure SPI CPHA (Clock Phase)
    // Determines when data is sampled relative to clock edge
    tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CP1_CPHA;
    
    // Step 8: Configure SSM (Slave Select Management)
    // Software or hardware control of slave select signal
    tempreg |= pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM;

    // Write the configured value to CR1 (Control Register 1)
    pSPIHandle->pSPIx->CR1 = tempreg;



}

void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	// Reset the specified SPI peripheral to default state
	// Uses RCC reset registers to restore power-on configuration
	
	if(pSPIx == SPI1)
		{
		SPI1_REG_RESET();      // Reset SPI1 to default state
		}
	else if (pSPIx == SPI2)
		{
		SPI2_REG_RESET();      // Reset SPI2 to default state
		}
	else if (pSPIx == SPI3)
		{
		SPI3_REG_RESET();      // Reset SPI3 to default state
		}
	else if (pSPIx == SPI4)
		{
		SPI4_REG_RESET();      // Reset SPI4 to default state
		}

}



/*
 * FLAG STATUS CHECK
 */

/*
 * @brief           - Checks if a flag is set in the SPI Status Register (SR)
 *
 * @param[in]       - pSPIx: Base address of the SPI peripheral
 * @param[in]       - FlagName: The flag bit to check (e.g., SPI_TXE_FLAG, SPI_RXNE_FLAG, SPI_BUSY_FLAG)
 *
 * @return          - FLAG_SET (1) if flag is set, FLAG_RESET (0) if flag is not set
 *
 * @Note            - Useful for polling the SPI status before transmitting or receiving data
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	// Check if the specified flag bit is set in the SR register
	if(pSPIx->SR & FlagName)
	{
		return FLAG_SET;      // Flag is set
	}
	return FLAG_RESET;        // Flag is not set
}
/*
 * Data read and write
 */

/***********************************************************
 * @fn				- SPI_SendData
 *
 * @brief           - Transmits data over SPI bus (blocking call)
 *                   Waits for TXE flag before sending each byte/word
 *
 * @param[in]       - pSPIx: Base address of the SPI peripheral
 * @param[in]       - pTxBuffer: Pointer to data buffer to transmit
 * @param[in]       - Len: Number of bytes to transmit
 *
 * @return          - None
 *
 * @Note            - This is a BLOCKING function - waits until data is transmitted
 *                   Ref: dm00096844, 20.5.1 table
 *                   Handles both 8-bit and 16-bit data frames
 *                   Will hang if TXE flag never sets (potential hardware issue)
 ***********************************************************/
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
	while( Len > 0 )
	{
		// Step 1: Wait until TXE (Transmit Buffer Empty) flag is set
		// TXE=1 means DR is empty and ready to accept new data
		while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);  // Can cause hang if HW issue

		// Step 2: Check the DFF (Data Frame Format) bit in CR1
		if(pSPIx->CR1 & (1 << SPI_CP1_DFF))
		{
			// 16-bit data frame format
			// Load 16-bit data into the Data Register (DR)
			pSPIx->DR = *((uint16_t*)(pTxBuffer));
			
			Len--;    // Decrement by 2 bytes (one 16-bit word)
			Len--;
			(uint16_t*)pTxBuffer++;   // Increment pointer by 2 bytes
		}else
		{
			// 8-bit data frame format (default)
			// Load 8-bit data into the Data Register (DR)
			pSPIx->DR = *(pTxBuffer);
			
			Len--;    // Decrement by 1 byte
			pTxBuffer++;   // Increment pointer by 1 byte
		}
	}
}




/*
 * IRQ configuration and ISR handling: ARM doc DUI0553, 4.2,Nested Vectored Interrupt Controller
 */


/*********************************************************************
 * @fn      		  - SPI_PeripheralCtrl
 *
 * @brief             - Enables or disables the SPI peripheral
 *                     Must be called after SPI_Init() to activate the SPI
 *
 * @param[in]         - pSPIx: Base address of the SPI peripheral
 * @param[in]         - EnOrDi: ENABLE to turn on SPI, DISABLE to turn off
 *
 * @return            - None
 *
 * @Note              - Modifies the SPE (SPI Enable) bit in CR1 register
 *                     When disabled, all SPI operations stop

 */
void SPI_PeripheralCtrl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	// Enable or disable the SPI peripheral by setting/clearing SPE bit
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);    // Set SPE bit to enable SPI
	}else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);   // Clear SPE bit to disable SPI
	}
}

/*
 * @brief             - Enables or disables software slave select management
 *                     When SSM is enabled, NSS is controlled via software (SSI bit)
 *
 * @param[in]         - pSPIx: Base address of the SPI peripheral
 * @param[in]         - EnOrDi: ENABLE for software control, DISABLE for hardware control
 *
 * @return            - None
 *
 * @Note              - SSI (Internal Slave Select) works with SSM to control NSS in software mode
 *                     Must be enabled when in master mode with software NSS management
 */
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	// Enable or disable internal slave select (SSI) for software NSS management
	if(EnOrDi == ENABLE)
		{
			pSPIx->CR1 |= (1 << SPI_CR1_SSI);   // Set SSI bit to enable software NSS
		}else
		{
			pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);  // Clear SSI bit to disable software NSS
		}
}

/*
 * IRQ configuration and IRQ priority handling: ARM doc DUI0553, 4.2,Nested Vectored Interrupt Controller
 */

