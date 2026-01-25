/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: Jan 29, 2019
 *      Author: admin
 */


#include "stm32f401xx_gpio_driver.h"


/*********************************************************************
 * @fn      		  - GPIO_PeriClockControl
 *
 * @brief             - Enables or disables the peripheral clock for a GPIO port
 *                     (Required before configuring GPIO pins)
 *
 * @param[in]         - pGPIOx: Base address of the GPIO port (GPIOA through GPIOE)
 * @param[in]         - EnorDi: ENABLE to turn on clock, DISABLE to turn off
 *
 * @return            - None
 *
 * @Note              - Must be called before GPIO_Init() for the pin to function
 *                     Uses RCC (Reset and Clock Control) peripheral to enable clock

 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	// Enable the clock for the specified GPIO port
	if(EnorDi == ENABLE)
	{
		// Check which GPIO port and enable its peripheral clock
		// These macros set the appropriate bits in RCC_AHB1ENR register
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();      // Enable GPIOA clock
		}else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();      // Enable GPIOB clock
		}else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();      // Enable GPIOC clock
		}else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();      // Enable GPIOD clock
		}else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();      // Enable GPIOE clock
		}
		else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();      // Enable GPIOH clock
		}

	}
	else
	{
		// TODO: Implement clock disable functionality for each port
	}

}




/*********************************************************************
 * @fn      		  - GPIO_Init
 *
 * @brief             - Initializes a GPIO pin with the specified configuration
 *                     Configures pin mode, speed, pull-up/down, output type, and alternate function
 *
 * @param[in]         - pGPIOHandle: Pointer to GPIO_Handle_t containing port and pin configuration
 *
 * @return            - None
 *
 * @Note              - Must call GPIO_PeriClockControl() first to enable port clock
 *                     Handles both standard GPIO modes and interrupt modes

 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	 uint32_t temp=0; // Temporary register for bit manipulation

	 // Step 1: Enable the peripheral clock for this GPIO port
	 GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	// Step 2: Configure GPIO pin mode
	// Modes include: Input, Output, Alternate Function, Analog, or Interrupt modes
	
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		// Standard GPIO modes (Input, Output, Alternate Function, or Analog)
		// Each pin mode uses 2 bits in the MODER register
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber ) );
		pGPIOHandle->pGPIOx->MODER &= ~( 0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); // Clear the 2 bits
		pGPIOHandle->pGPIOx->MODER |= temp;                                                        // Set new value

	}else
	{
		// Interrupt mode configuration
		// Configure EXTI (External Interrupt) edge detection
		
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode ==GPIO_MODE_IT_FT )
		{
			// Falling edge interrupt (signal goes from high to low)
			EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);      // Enable falling edge
			EXTI->RTSR &= ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);     // Disable rising edge

		}else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode ==GPIO_MODE_IT_RT )
		{
			// Rising edge interrupt (signal goes from low to high)
			EXTI->RTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);      // Enable rising edge
			EXTI->FTSR &= ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);     // Disable falling edge

		}else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT )
		{
			// Both rising and falling edge interrupt
			EXTI->RTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);      // Enable rising edge
			EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);      // Enable falling edge
		}

		// Step 3: Configure which GPIO port triggers the interrupt
		// Each EXTICR register handles 4 pins (pins 0-3, 4-7, 8-11, 12-15)
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4 ;       // Which EXTICR register (0, 1, 2, 3)
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;        // Which field in that register
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);         // Convert port address to code (GPIOA=0, B=1, C=2, etc.)
		SYSCFG_PCLK_EN();                                                      // Enable SYSCFG peripheral clock
		SYSCFG->EXTICR[temp1] = portcode << ( temp2 * 4);                     // Set the port code in EXTICR

		// Step 4: Enable the interrupt request to be sent to NVIC
		EXTI->IMR |= 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;         // Unmask the interrupt
	}

	// Step 3: Configure output speed (OSPEEDR register)
	// Speed options: LOW, MEDIUM, FAST, or VERY_FAST
	// Higher speed may increase power consumption
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
	pGPIOHandle->pGPIOx->OSPEEDR &= ~( 0x3 << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); // Clear the 2 bits
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;                                                         // Set new speed value

	// Step 4: Configure pull-up/pull-down settings (PUPDR register)
	// Options: No pull, Pull-up, or Pull-down
	// Pull-up/down provides a default state when pin is not driven
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
	pGPIOHandle->pGPIOx->PUPDR &= ~( 0x3 << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); // Clear the 2 bits
	pGPIOHandle->pGPIOx->PUPDR |= temp;                                                          // Set new pull-up/down value


	// Step 5: Configure output type (OTYPER register)
	// Options: Push-pull (normal) or Open-drain (for I2C, etc.)
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
	pGPIOHandle->pGPIOx->OTYPER &= ~( 0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // Clear the 1 bit
	pGPIOHandle->pGPIOx->OTYPER |= temp;                                                   // Set new output type value

	// Step 6: Configure alternate function (AFR registers)
	// Each pin can be assigned to a specific peripheral (SPI, I2C, UART, etc.)
	// Only applies when GPIO_PinMode is set to GPIO_MODE_ALTFN
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		// AFR[0] handles pins 0-7, AFR[1] handles pins 8-15
		// Each pin's alternate function uses 4 bits in the AFR register
		uint8_t temp1, temp2;

		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;              // Which AFR register (0 or 1)
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber  % 8;             // Which field in that register
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << ( 4 * temp2 ) );        // Clear the 4 bits
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << ( 4 * temp2 ) );  // Set alternate function
	}

}


/*********************************************************************
 * @fn      		  - GPIO_DeInit
 *
 * @brief             - Resets the GPIO port to default state (power-on reset state)
 *                     Clears all configuration registers for the specified port
 *
 * @param[in]         - pGPIOx: Base address of the GPIO port to reset
 *
 * @return            - None
 *
 * @Note              - Uses RCC reset registers to restore default configuration
 *                     All pins on the port are reset

 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	// Reset the specified GPIO port using RCC reset registers
	// These macros set and clear the appropriate bits in RCC_AHB1RSTR register
	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();      // Reset GPIOA to default state
	}else if (pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();      // Reset GPIOB to default state
	}else if (pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();      // Reset GPIOC to default state
	}else if (pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();      // Reset GPIOD to default state
	}else if (pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();      // Reset GPIOE to default state

	}
	else if (pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();      // Reset GPIOH to default state
	}	

}



/*********************************************************************
 * @fn      		  - GPIO_ReadFromInputPin
 *
 * @brief             - Reads the digital value (0 or 1) from an input GPIO pin
 *
 * @param[in]         - pGPIOx: Base address of the GPIO port
 * @param[in]         - PinNumber: Pin number to read (0-15)
 *
 * @return            - uint8_t: 0 (LOW) or 1 (HIGH)
 *
 * @Note              - Reads from IDR (Input Data Register)
 *                     Pin must be configured in input mode first

 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
   uint8_t value;

   // Read the bit at PinNumber position from IDR register and isolate it
   // Right-shift to get the desired bit, then mask with 0x01 to get only that bit
   value = (uint8_t )((pGPIOx->IDR  >> PinNumber) & 0x00000001 ) ;

   return value;   // Return 0 if pin is LOW, 1 if pin is HIGH
}


/*********************************************************************
 * @fn      		  - GPIO_ReadFromInputPort
 *
 * @brief             - Reads all 16 bits from a GPIO port
 *
 * @param[in]         - pGPIOx: Base address of the GPIO port
 *
 * @return            - uint16_t: 16-bit value representing all pins (bit 0 = pin 0, bit 15 = pin 15)
 *
 * @Note              - Reads from IDR (Input Data Register)
 *                     Useful for reading multiple pins simultaneously

 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;

	// Read the entire 16-bit IDR register which contains status of all pins
	value = (uint16_t)pGPIOx->IDR;

	return value;   // Return entire port state as one 16-bit value
}


/*********************************************************************
 * @fn      		  - GPIO_WriteToOutputPin
 *
 * @brief             - Sets or clears an output GPIO pin (writes 0 or 1)
 *
 * @param[in]         - pGPIOx: Base address of the GPIO port
 * @param[in]         - PinNumber: Pin number to write (0-15)
 * @param[in]         - Value: GPIO_PIN_SET (1) to set pin HIGH, GPIO_PIN_CLEAR (0) to set LOW
 *
 * @return            - None
 *
 * @Note              - Writes to ODR (Output Data Register)
 *                     Pin must be configured in output mode first

 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	// Write a 1 or 0 to the specified pin in the output data register

	if(Value == GPIO_PIN_SET)
	{
		// Set the pin HIGH (logic 1)
		// Use OR operation to set only the desired bit without affecting others
		pGPIOx->ODR |= ( 1 << PinNumber);
	}else
	{
		// Set the pin LOW (logic 0)
		// Use AND with complement to clear only the desired bit
		pGPIOx->ODR &= ~( 1 << PinNumber);
	}
}


/*********************************************************************
 * @fn      		  - GPIO_WriteToOutputPort
 *
 * @brief             - Writes a 16-bit value to all GPIO pins on a port simultaneously
 *
 * @param[in]         - pGPIOx: Base address of the GPIO port
 * @param[in]         - Value: 16-bit value to write (bit 0 = pin 0, bit 15 = pin 15)
 *
 * @return            - None
 *
 * @Note              - Writes to entire ODR (Output Data Register) at once
 *                     Useful for simultaneous control of multiple pins

 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	// Replace the entire ODR register with the new value
	// All 16 pins change state according to the bits in Value
	pGPIOx->ODR  = Value;
}


/*********************************************************************
 * @fn      		  - GPIO_ToggleOutputPin
 *
 * @brief             - Toggles the output state of a GPIO pin (0→1 or 1→0)
 *
 * @param[in]         - pGPIOx: Base address of the GPIO port
 * @param[in]         - PinNumber: Pin number to toggle (0-15)
 *
 * @return            - None
 *
 * @Note              - Uses XOR operation to flip the bit
 *                     Pin must be configured in output mode first

 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	// XOR the pin bit with 1 to toggle it
	// If pin is 0, XOR with 1 gives 1 (set HIGH)
	// If pin is 1, XOR with 1 gives 0 (set LOW)
	pGPIOx->ODR  ^= ( 1 << PinNumber);
}



/*********************************************************************
 * @fn      		  - GPIO_IRQInterruptConfig
 *
 * @brief             - Enables or disables an external interrupt in the NVIC (Nested Vectored Interrupt Controller)
 *                     Must be called after GPIO_Init() for interrupt mode pins
 *
 * @param[in]         - IRQNumber: IRQ number of the interrupt (device-specific, check datasheet)
 * @param[in]         - EnorDi: ENABLE to activate interrupt, DISABLE to deactivate
 *
 * @return            - None
 *
 * @Note              - NVIC uses three 32-bit registers (ISER0, ISER1, ISER2) to enable up to 96 interrupts
 *                     ISER = Interrupt Set-Enable Register
 *                     ICER = Interrupt Clear-Enable Register

 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{

	if(EnorDi == ENABLE)
	{
		// Enable the interrupt in NVIC
		// Each ISER register handles 32 interrupts (bits 0-31)
		
		if(IRQNumber <= 31)
		{
			// IRQs 0-31: Use ISER0 register
			*NVIC_ISER0 |= ( 1 << IRQNumber );

		}else if(IRQNumber > 31 && IRQNumber < 64 ) //32 to 63
		{
			// IRQs 32-63: Use ISER1 register
			*NVIC_ISER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber >= 64 && IRQNumber < 96 )
		{
			// IRQs 64-95: Use ISER2 register
			*NVIC_ISER2 |= ( 1 << (IRQNumber % 64) );
		}
	}else
	{
		// Disable the interrupt in NVIC
		if(IRQNumber <= 31)
		{
			// IRQs 0-31: Use ICER0 register
			*NVIC_ICER0 |= ( 1 << IRQNumber );
		}else if(IRQNumber > 31 && IRQNumber < 64 )
		{
			// IRQs 32-63: Use ICER1 register
			*NVIC_ICER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber >= 64 && IRQNumber < 96 )
		{
			// IRQs 64-95: Use ICER2 register
			*NVIC_ICER2 |= ( 1 << (IRQNumber % 64) );
		}
	}

}



/*********************************************************************
 * @fn      		  - GPIO_IRQPriorityConfig
 *
 * @brief             - Sets the priority level for an interrupt (determines handling order)
 *                     Lower priority value = higher urgency
 *
 * @param[in]         - IRQNumber: IRQ number of the interrupt
 * @param[in]         - IRQPriority: Priority value (implementation-dependent on microcontroller)
 *
 * @return            - None
 *
 * @Note              - NVIC Priority Registers (IPR) each handle 4 interrupts
 *                     Each interrupt uses 8 bits, but typically only the upper bits are used
 *                     NO_PR_BITS_IMPLEMENTED defines how many bits are actually used

 */
void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
{
	// Priority registers store interrupt priorities
	// Each IPR register handles 4 interrupts, each using 1 byte (8 bits)
	
	uint8_t iprx = IRQNumber / 4;                              // Which IPR register (0, 1, 2, 3, etc.)
	uint8_t iprx_section  = IRQNumber % 4 ;                    // Which byte within that register (0-3)

	// Calculate the shift amount accounting for implemented priority bits
	// (8 - NO_PR_BITS_IMPLEMENTED) shifts priority to the correct upper bits
	uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - NO_PR_BITS_IMPLEMENTED) ;

	// Set the priority bits in the appropriate register
	*(  NVIC_PR_BASE_ADDR + iprx ) |=  ( IRQPriority << shift_amount );

}
/*********************************************************************
 * @fn      		  - GPIO_IRQHandling
 *
 * @brief             - Handles an interrupt by clearing the interrupt pending flag
 *                     This function should be called in the IRQ handler for the pin
 *
 * @param[in]         - PinNumber: Pin number that triggered the interrupt (0-15)
 *
 * @return            - None
 *
 * @Note              - Clears the pending request in EXTI PR (Pending Register)
 *                     Must be called to clear the interrupt flag, otherwise it may keep firing

 */
void GPIO_IRQHandling(uint8_t PinNumber)
{
	// Clear the interrupt pending flag
	// When an external interrupt occurs, a flag is set in the EXTI PR register
	// The handler must clear this flag to allow future interrupts on the same pin
	
	if(EXTI->PR & ( 1 << PinNumber))                // Check if the pending flag is set for this pin
	{
		//clear the pending flag by writing 1 to it
		// (Writing 1 to PR clears the bit; this is a "write 1 to clear" register)
		EXTI->PR |= ( 1 << PinNumber);
	}

}
