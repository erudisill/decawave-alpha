#ifndef _CPH_BOARD_H
#define _CPH_BOARD_H

#include "compiler.h"
#include "system_sam4s.h"
#include "exceptions.h"
#include "pio.h"

/** Board oscillator settings */
#define BOARD_FREQ_SLCK_XTAL        (32768U)
#define BOARD_FREQ_SLCK_BYPASS      (32768U)
#define BOARD_FREQ_MAINCK_XTAL      (12000000U)
#define BOARD_FREQ_MAINCK_BYPASS    (12000000U)

/** Master clock frequency */
#define BOARD_MCK                   CHIP_FREQ_CPU_MAX

/** board main clock xtal statup time */
#define BOARD_OSC_STARTUP_US   15625

/** Name of the board */
#define BOARD_NAME "CPH-BOARD"
/** Board definition */
#define cphBoard
/** Family definition (already defined) */
#define sam4s
/** Core definition */
#define cortexm4

#define TASK_MONITOR_STACK_SIZE            (2048/sizeof(portSTACK_TYPE))
#define TASK_MONITOR_STACK_PRIORITY        (tskIDLE_PRIORITY)

#define TASK_LED_STACK_SIZE                (1024/sizeof(portSTACK_TYPE))
#define TASK_LED_STACK_PRIORITY            (tskIDLE_PRIORITY)

#define TASK_CLI_STACK_SIZE                (2048/sizeof(portSTACK_TYPE))
#define TASK_CLI_STACK_PRIORITY            (tskIDLE_PRIORITY)

#define TASK_DW_STACK_SIZE                 (2048/sizeof(portSTACK_TYPE))
#define TASK_DW_STACK_PRIORITY             (tskIDLE_PRIORITY)

#define DELAY_1S							(1000 / portTICK_PERIOD_MS)

/*----------------------------------------------------------------------------*/
/*	CONSOLE																	  */
/*----------------------------------------------------------------------------*/

//#define CONSOLE_UART        UART0
//#define CONSOLE_UART_ID     ID_UART0
//
//#define PINS_CONSOLE_PIO 		PIOA
//#define PINS_CONSOLE_TYPE 	PIO_PERIPH_A
//#define PINS_CONSOLE_MASK 	PIO_PA9A_URXD0|PIO_PA10A_UTXD0
//#define PINS_CONSOLE_ATTR 	PIO_DEFAULT

#define CONSOLE_UART        UART1
#define CONSOLE_UART_ID     ID_UART1

#define PINS_CONSOLE_PIO 	PIOB
#define PINS_CONSOLE_TYPE 	PIO_PERIPH_A
#define PINS_CONSOLE_MASK 	PIO_PB2A_URXD1|PIO_PB3A_UTXD1
#define PINS_CONSOLE_ATTR 	PIO_DEFAULT

/*----------------------------------------------------------------------------*/
/*	LEDS																	  */
/*----------------------------------------------------------------------------*/

#define LED_STATUS0_IDX		PIO_PC12_IDX

#define PINS_LED0_PIO		PIOC
#define PINS_LED0_TYPE		PIO_OUTPUT_0
#define PINS_LED0_MASK		PIO_PC12
#define PINS_LED0_ATTR		PIO_DEFAULT


#define LED_STATUS1_IDX		PIO_PC13_IDX

#define PINS_LED1_PIO		PIOC
#define PINS_LED1_TYPE		PIO_OUTPUT_0
#define PINS_LED1_MASK		PIO_PC13
#define PINS_LED1_ATTR		PIO_DEFAULT

#define LED_GROUP_PIO		PIOC
#define LED_GROUP_MASK		(PINS_LED0_MASK | PINS_LED1_MASK)



/*----------------------------------------------------------------------------*/
/*	DECAWAVE																  */
/*----------------------------------------------------------------------------*/


// Decawave SPI
#define DW_SPI_BAUD_FAST			5000000UL
#define DW_SPI_BAUD_SLOW			1000000UL
#define DW_SPI						SPI
#define DW_SPI_MAX_BLOCK_TIME 		(50 / portTICK_RATE_MS)
#define DW_CHIP_SELECT				1
#define DW_CHIP_SELECT_VALUE		0x0001
#define DW_NONE_CHIP_SELECT_VALUE   0x0f
//#define DW_DELAY_BEFORE				0x40
//#define DW_DELAY_BETWEEN			0x10
#define DW_DELAY_BEFORE				0x00
#define DW_DELAY_BETWEEN			0x10
#define DW_CLOCK_POLARITY			1
#define DW_CLOCK_PHASE				1
//#define DW_CLOCK_PHASE				0

// Decawave CSn (maybe not used??)
#define DW_CSn_PIO				PIOC
#define DW_CSn_PIO_IDX			PIO_PC9_IDX
#define DW_CSn_MASK				PIO_PC9
#define DW_CSn_TYPE				PIO_OUTPUT_1
#define DW_CSn_ATTR				(PIO_DEFAULT)


// Decawave RSTn - input, no pull up/down
#define DW_RSTn_PIO					PIOC
#define DW_RSTn_PIO_ID				ID_PIOC
#define DW_RSTn_PIO_IDX				PIO_PC10_IDX
#define DW_RSTn_MASK				PIO_PC10
#define DW_RSTn_TYPE				PIO_INPUT
#define DW_RSTn_ATTR				(PIO_IT_RISE_EDGE | PIO_DEFAULT)
#define DW_RSTn_FLAGS				(DW_RSTn_TYPE | DW_RSTn_ATTR)

// Decawave RSTSWn - tied to a P-CHAN MOSFET
#define DW_RSTSWn_PIO				PIOC
#define DW_RSTSWn_PIO_IDX			PIO_PC11_IDX
#define DW_RSTSWn_MASK				PIO_PC11
#define DW_RSTSWn_TYPE				PIO_OUTPUT_1
#define DW_RSTSWn_ATTR				(PIO_DEFAULT)

// Decawave IRQ
#define DW_IRQ_PIO					PIOC
#define DW_IRQ_PIO_ID				ID_PIOC
#define DW_IRQ_IDX					PIO_PC0_IDX
#define DW_IRQ_MASK					PIO_PC0
#define DW_IRQ_TYPE					PIO_INPUT
#define DW_IRQ_ATTR					(PIO_IT_RISE_EDGE | PIO_DEFAULT)
#define DW_IRQ_FLAGS				(DW_IRQ_TYPE | DW_IRQ_ATTR)


#endif  // _CPH_BOARD_H
