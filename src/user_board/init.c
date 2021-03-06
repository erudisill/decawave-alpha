/**
 * \file
 *
 * \brief SAM4S-EK board init.
 *
 * Copyright (c) 2011 - 2013 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */

#include "compiler.h"
#include "board.h"
#include "conf_board.h"

void board_init(void)
{
#ifndef CONF_BOARD_KEEP_WATCHDOG_AT_INIT
	/* Disable the watchdog */
	WDT->WDT_MR = WDT_MR_WDDIS;
#endif

#ifdef CONF_BOARD_LEDS
	pio_configure(PINS_LED0_PIO, PINS_LED0_TYPE, PINS_LED0_MASK, PINS_LED0_ATTR);
	pio_configure(PINS_LED1_PIO, PINS_LED1_TYPE, PINS_LED1_MASK, PINS_LED1_ATTR);
#endif

#ifdef CONF_BOARD_UART_CONSOLE
	pio_configure(PINS_CONSOLE_PIO, PINS_CONSOLE_TYPE, PINS_CONSOLE_MASK, PINS_CONSOLE_ATTR);
#endif

	pio_configure(DW_WAKEUP_PIO, DW_WAKEUP_TYPE, DW_WAKEUP_MASK, DW_WAKEUP_ATTR);
	pio_configure(DW_RSTn_PIO, DW_RSTn_TYPE, DW_RSTn_MASK, DW_RSTn_ATTR);
	pio_configure(DW_RSTSWn_PIO, DW_RSTSWn_TYPE, DW_RSTSWn_MASK, DW_RSTSWn_ATTR);

}
