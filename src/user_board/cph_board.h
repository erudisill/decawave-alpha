
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
//#define PINS_UART0_PIO 		PIOA
//#define PINS_UART0_TYPE 	PIO_PERIPH_A
//#define PINS_UART0_MASK 	PIO_PA9A_URXD0|PIO_PA10A_UTXD0
//#define PINS_UART0_ATTR 	PIO_DEFAULT


#define CONSOLE_UART        UART1
#define CONSOLE_UART_ID     ID_UART1

#define PINS_CONSOLE_PIO 	PIOB
#define PINS_CONSOLE_TYPE 	PIO_PERIPH_A
#define PINS_CONSOLE_MASK 	PIO_PB2A_URXD1|PIO_PB3A_UTXD1
#define PINS_CONSOLE_ATTR 	PIO_DEFAULT

/*----------------------------------------------------------------------------*/
/*	LEDS																	  */
/*----------------------------------------------------------------------------*/

#define LED_STATUS_IDX		PIO_PC12_IDX

#define PINS_LED0_PIO		PIOC
#define PINS_LED0_TYPE		PIO_OUTPUT_0
#define PINS_LED0_MASK		PIO_PC12
#define PINS_LED0_ATTR		PIO_DEFAULT


/*----------------------------------------------------------------------------*/
/*	DECAWAVE																  */
/*----------------------------------------------------------------------------*/

#define DW_CS_IDX			PIO_PC9_IDX
#define DW_CS_PIO			PIOC
#define DW_CS_TYPE			PIO_OUTPUT_1
#define DW_CS_MASK			PIO_PC9
#define DW_CS_ATTR			PIO_DEFAULT





#endif  // _CPH_BOARD_H
