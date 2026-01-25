#ifndef  INC_STM32F401XX_GPIO_DRIVER_H_
#define  INC_STM32F401XX_GPIO_DRIVER_H_

#include "stm32f401xx.h"

typedef struct
{
	uint8_t GPIO_PinNumber;              /* @GPIO_PIN_NUMBERS */
	uint8_t GPIO_PinMode;                /* @GPIO_PIN_MODES */
	uint8_t GPIO_PinSpeed;               /* @GPIO_PIN_SPEED */
	uint8_t GPIO_PinPuPdControl;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;


typedef struct
{
	GPIO_RegDef_t                  *pGPIOx;
	GPIO_PinConfig_t               GPIO_PinConfig;  // This hold GPIO pin
}GPIO_Handle_t;


/*
 * @GPIO_PIN_NUMBERS
 * GPIO pin possible modes
 * Ref: dm00096844
 */
#define GPIO_PIN_NO_0       0
#define GPIO_PIN_NO_1       1
#define GPIO_PIN_NO_2       2
#define GPIO_PIN_NO_3       3
#define GPIO_PIN_NO_4       4
#define GPIO_PIN_NO_5       5
#define GPIO_PIN_NO_6       6
#define GPIO_PIN_NO_7       7
#define GPIO_PIN_NO_8       8
#define GPIO_PIN_NO_9       9
#define GPIO_PIN_NO_10      10
#define GPIO_PIN_NO_11      11
#define GPIO_PIN_NO_12      12
#define GPIO_PIN_NO_13      13
#define GPIO_PIN_NO_14      14
#define GPIO_PIN_NO_15      15


/*
 * @GPIO_PIN_MODES
 * GPIO pin possible modes
 * Ref: dm00096844, 8.4.1
 */
#define GPIO_MODE_INPUT    0
#define GPIO_MODE_OUTPUT   1
#define GPIO_MODE_ALTFN    2
#define GPIO_MODE_ANALOG   3
#define GPIO_MODE_IT_FT    4
#define GPIO_MODE_IT_RT    5
#define GPIO_MODE_IT_RFT   6




/*
 * @GPIO_PIN_MODES
 * GPIO pin possible output type
 * Ref: dm00096844, 8.4.2
*/
#define GPIO_OP_TYPE_PP      0
#define GPIO_OP_TYPE_OD      1

/*
 * @GPIO_PIN_SPEED
 * GPIO pin possible speed
 * Ref: dm00096844, 8.4.3
 */
#define GPIO_SPEED_LOW       0
#define GPIO_SPEED_MEDIUM    1
#define GPIO_SPEED_FAST      2
#define GPIO_SPEED_HIGH      3

/*
 * GPIO pin pull-up/pull-down configuration macros
 * Ref: dm00096844, 8.4.4
 */
#define GPIO_NO_PUPD       0
#define GPIO_PIN_PU        1
#define GPIO_PIN_PD        2


/* APIs supported by this driver
 */

/*
 * Peripheral Clock setup
 */
void GPIO_PeriClkCtrl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

/*
 * Init and De-init
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*
 * Data read and write
 */

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInput(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutport(GPIO_RegDef_t *pGPIOx, uint8_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*
 * IRQ configuration and ISR handling
 */
void GPIO_IRQITConfig(uint8_t IRQNumber,  uint8_t EnorDi);
void GPIO_IRQHandling(uint8_t PinNumber);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);

#endif
