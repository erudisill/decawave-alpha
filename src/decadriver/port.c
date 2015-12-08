/*! ----------------------------------------------------------------------------
 * @file	port.c
 * @brief	HW specific definitions and functions for portability
 *
 * @attention
 *
 * Copyright 2013 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author DecaWave
 */

#include <port.h>

int NVIC_DisableDECAIRQ(void) {
	pio_disable_interrupt(DW_IRQ_PIO, DW_IRQ_MASK);

	return 0;
}

int NVIC_Configuration(void) {

	pio_configure_pin(DW_IRQ_IDX, DW_IRQ_FLAGS);
	pio_pull_down(DW_IRQ_PIO, DW_IRQ_MASK, true);
	pio_handler_set(DW_IRQ_PIO, DW_IRQ_PIO_ID, DW_IRQ_MASK, DW_IRQ_ATTR, process_deca_irq);
	pio_enable_interrupt(DW_IRQ_PIO, DW_IRQ_MASK);
	// handler prototype:  void (*p_handler) (uint32_t, uint32_t)

//	NVIC_DisableIRQ(PIOC_IRQn);
//	NVIC_ClearPendingIRQ(PIOC_IRQn);
//	NVIC_SetPriority(PIOC_IRQn, configLIBRARY_LOWEST_INTERRUPT_PRIORITY); //TODO: Is this correct IRQ priority w/ FreeRTOS?
//	NVIC_EnableIRQ(PIOC_IRQn);

//	pio_handler_set_priority(DW_IRQ_PIO, PIOC_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
	pio_handler_set_priority(DW_IRQ_PIO, DW_IRQ_IRQ, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
//	pio_handler_set_priority(DW_IRQ_PIO, PIOC_IRQn, 0);

	pmc_enable_periph_clk(DW_IRQ_PIO_ID);

	return 0;
}

void SPI_ConfigFastRate(uint32_t baudrate) {
	spi_disable(DW_SPI);
	spi_set_baudrate_div(DW_SPI, DW_CHIP_SELECT, (sysclk_get_cpu_hz() / baudrate));
	spi_enable(DW_SPI);
}

void * SPI_Configuration(void) {

	freertos_spi_if spi_if;

	const freertos_peripheral_options_t driver_options = {
	/* No receive buffer pointer needed for SPI */
	NULL,

	/* No receive buffer size needed for SPI */
	0,

	/* Cortex-M4 priority */
	configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY,

	/* Operation mode - MASTER */
	SPI_MASTER,

	/* Blocking (not async) */
	(USE_TX_ACCESS_MUTEX | USE_RX_ACCESS_MUTEX | WAIT_TX_COMPLETE | WAIT_RX_COMPLETE) };

	spi_if = freertos_spi_master_init(DW_SPI, &driver_options);

	if (spi_if != NULL) {
		pio_configure_pin(PIO_PA12_IDX, PIO_PERIPH_A);	// MISO
		pio_configure_pin(PIO_PA13_IDX, PIO_PERIPH_A);	// MOSI
		pio_configure_pin(PIO_PA14_IDX, PIO_PERIPH_A);	// SPCK
//		pio_configure_pin(PIO_PC4_IDX, PIO_PERIPH_B);	// NPCS1
		pio_configure_pin(DW_CSn_PIO_IDX, DW_CSn_PIO_PERIPH);
		pmc_enable_periph_clk(ID_SPI);

		spi_disable(DW_SPI);
		spi_set_clock_polarity(DW_SPI, DW_CHIP_SELECT, DW_CLOCK_POLARITY);
		spi_set_clock_phase(DW_SPI, DW_CHIP_SELECT, DW_CLOCK_PHASE);
		spi_set_baudrate_div(DW_SPI, DW_CHIP_SELECT, (sysclk_get_cpu_hz() / DW_SPI_BAUD_SLOW));
		spi_set_transfer_delay(DW_SPI, DW_CHIP_SELECT, DW_DELAY_BEFORE,		DW_DELAY_BETWEEN);
//		spi_configure_cs_behavior(DW_SPI, DW_CHIP_SELECT, SPI_CS_KEEP_LOW);
		spi_configure_cs_behavior(DW_SPI, DW_CHIP_SELECT, SPI_CS_RISE_NO_TX);
		spi_set_peripheral_chip_select_value(DW_SPI, DW_CHIP_SELECT_VALUE);

		spi_enable(DW_SPI);
	}

	return spi_if;
}

void reset_DW1000(void) {
	// low gate - turn on MOSFET and drive to ground
	pio_set_pin_low(DW_RSTSWn_PIO_IDX);

	// sleep here??

	// high gate - turn off MOSFET and open drain
	pio_set_pin_high(DW_RSTSWn_PIO_IDX);

	// Why sleep here?  Left over from original code.
	Sleep(1);
}

void setup_DW1000RSTnIRQ(int enable) {
	if (enable) {
		// Listen for interrupt on RSTn line

		pio_configure_pin(DW_RSTn_PIO_IDX, DW_RSTn_FLAGS);
		pio_pull_down(DW_RSTn_PIO, DW_RSTn_MASK, true);
		pio_handler_set(DW_RSTn_PIO, DW_RSTn_PIO_ID, DW_RSTn_MASK, DW_RSTn_ATTR, process_dwRSTn_irq);
		pio_enable_interrupt(DW_RSTn_PIO, DW_RSTn_MASK);
		// handler prototype:  void (*p_handler) (uint32_t, uint32_t)

		NVIC_DisableIRQ(PIOC_IRQn);
		NVIC_ClearPendingIRQ(PIOC_IRQn);
		NVIC_SetPriority(PIOC_IRQn, configLIBRARY_LOWEST_INTERRUPT_PRIORITY);//TODO: Is this correct IRQ priority w/ FreeRTOS?
		NVIC_EnableIRQ(PIOC_IRQn);

		pmc_enable_periph_clk(DW_RSTn_PIO_ID);

	} else {
		// Back to normal input pin (tri-state, no pull up/down)
		pio_disable_interrupt(DW_RSTn_PIO, DW_RSTn_MASK);
		pio_configure(DW_RSTn_PIO, DW_RSTn_TYPE, DW_RSTn_MASK, DW_RSTn_ATTR);
	}
}

void led_off(led_t led) {
	switch (led) {
	case LED_PC6:
		pio_set_pin_high(LED_STATUS0_IDX);
		break;
	case LED_PC7:
		pio_set_pin_high(LED_STATUS1_IDX);
		break;
	case LED_ALL:
		pio_set_pin_group_high(LED_GROUP_PIO, LED_GROUP_MASK);
		break;
	default:
		// do nothing for undefined led number
		break;
	}
}

void led_on(led_t led) {
	switch (led) {
	case LED_PC6:
		pio_set_pin_low(LED_STATUS0_IDX);
		break;
	case LED_PC7:
		pio_set_pin_low(LED_STATUS1_IDX);
		break;
	case LED_ALL:
		pio_set_pin_group_low(LED_GROUP_PIO, LED_GROUP_MASK);
		break;
	default:
		// do nothing for undefined led number
		break;
	}
}


#if TESTING
int readfromspi_serial(uint16_t headerLength, const uint8_t *headerBuffer, uint32_t readlength, uint8_t *readBuffer) {
	status_code_t result = STATUS_OK;
	uint32_t d;
	int i;

	// CS is controlled by NCSx setting on SAM4
	// Use LASTXFER bit to signify last byte and let SAM4 release CS line

	pio_set_pin_low(DW_CSn_PIO_IDX);

	// write command
	for (i = 0; i < headerLength; i++) {
		d = SPI_TDR_TD(headerBuffer[i]);
		// wait till ready then transmit
		while ((DW_SPI->SPI_SR & SPI_SR_TDRE) == 0)
			;
		DW_SPI->SPI_TDR = d;
		// wait till the received is ready, then grab, but ignore
		while ((DW_SPI->SPI_SR & SPI_SR_RDRF) == 0)
			;
		d = DW_SPI->SPI_RDR & SPI_RDR_RD_Msk;
	}

	// read results
	for (i = 0; i < readlength; i++) {
		// wait till ready then transmit dummy byte
		while ((DW_SPI->SPI_SR & SPI_SR_TDRE) == 0)
			;
		if (i == (readlength - 1)) {
			DW_SPI->SPI_TDR = SPI_TDR_LASTXFER | 0xff;
		} else {
			DW_SPI->SPI_TDR = 0xff;
		}
		// wait till the received is ready, then grab the data
		while ((DW_SPI->SPI_SR & SPI_SR_RDRF) == 0)
			;
		readBuffer[i] = (uint8_t) (DW_SPI->SPI_RDR & 0xff);
	}

	pio_set_pin_high(DW_CSn_PIO_IDX);

	return result;
}

int writetospi_serial(uint16_t headerLength, const uint8_t *headerBuffer, uint32_t bodylength, const uint8_t *bodyBuffer) {
	status_code_t result = STATUS_OK;
	uint32_t d;
	int i;

	// CS is controlled by NCSx setting on SAM4
	// Use LASTXFER bit to signify last byte and let SAM4 release CS line

	pio_set_pin_low(DW_CSn_PIO_IDX);

	// write header
	for (i = 0; i < headerLength; i++) {
		d = SPI_TDR_TD(headerBuffer[i]);
		// wait till ready then transmit
		while ((DW_SPI->SPI_SR & SPI_SR_TDRE) == 0)
			;
		DW_SPI->SPI_TDR = d;
		// wait till the received is ready, then grab, but ignore
		while ((DW_SPI->SPI_SR & SPI_SR_RDRF) == 0)
			;
		d = DW_SPI->SPI_RDR & SPI_RDR_RD_Msk;
	}

	// write body
	for (i = 0; i < bodylength; i++) {
		d = SPI_TDR_TD(bodyBuffer[i]);
		// wait till ready then transmit
		while ((DW_SPI->SPI_SR & SPI_SR_TDRE) == 0)
			;
		if (i == (bodylength - 1)) {
			DW_SPI->SPI_TDR = SPI_TDR_LASTXFER | d;
		} else {
			DW_SPI->SPI_TDR = d;
		}
		// wait till the received is ready, then grab, but ignore
		while ((DW_SPI->SPI_SR & SPI_SR_RDRF) == 0)
			;
		d = DW_SPI->SPI_RDR & SPI_RDR_RD_Msk;
	}

	pio_set_pin_high(DW_CSn_PIO_IDX);

	return result;
}
#else

#include <string.h>
static uint8_t spi_buffer[1024];

int writetospi_serial(uint16_t headerLength, const uint8_t *headerBuffer, uint32_t bodylength, const uint8_t *bodyBuffer) {
	status_code_t result = STATUS_OK;

	memcpy(spi_buffer, headerBuffer, headerLength);
	memcpy(&spi_buffer[headerLength], bodyBuffer, bodylength);

	spi_set_peripheral_chip_select_value(DW_SPI, (~(1U << DW_CHIP_SELECT)));

	if (freertos_spi_write_packet(DW_SPI, spi_buffer, bodylength + headerLength, DW_SPI_MAX_BLOCK_TIME) != STATUS_OK) {
		result = STATUS_ERR_TIMEOUT;
	}

	spi_set_peripheral_chip_select_value(DW_SPI, DW_NONE_CHIP_SELECT_VALUE);
	spi_set_lastxfer(DW_SPI);

	if (result != STATUS_OK) {
		printf("writetospi_serial timeout\r\n");
		for (;;) {
		}
	}

	return result;
}

int readfromspi_serial(uint16_t headerLength, const uint8_t *headerBuffer, uint32_t readlength, uint8_t *readBuffer) {
	status_code_t result = STATUS_OK;

	memcpy(spi_buffer, headerBuffer, headerLength);
	memset(&spi_buffer[headerLength],0xff,readlength);

	spi_set_peripheral_chip_select_value(DW_SPI, (~(1U << DW_CHIP_SELECT)));

	//NOTE: Requires a change in ASF FreeRTOS SPI code freertos_spi_master.c line 423
	//      This is because by default, the ASF code was blasting the buffer with 0xff for the read,
	//      but in this case, we need the "dummy" bytes of the header to remain.  So I just commented out
	//      the memset() line in the ASF code.

	if (freertos_spi_read_packet(DW_SPI, spi_buffer, readlength + headerLength, DW_SPI_MAX_BLOCK_TIME) != STATUS_OK) {
		result = STATUS_ERR_TIMEOUT;
	}
	else {
		memcpy(readBuffer, &spi_buffer[headerLength], readlength);
	}

	spi_set_peripheral_chip_select_value(DW_SPI, DW_NONE_CHIP_SELECT_VALUE);
	spi_set_lastxfer(DW_SPI);

	if (result != STATUS_OK) {
		printf("readfromspi_serial timeout\r\n");
		for (;;) {
		}
	}

	return result;
}
#endif

void port_SPIx_clear_chip_select(void) {
	// Is this the best way to control the CS line?
	uint8_t dummy_buf[1];
	dummy_buf[0] = 0x00;
	spi_set_peripheral_chip_select_value(DW_SPI, (~(1U << DW_CHIP_SELECT)));
	freertos_spi_write_packet(DW_SPI, dummy_buf, 1, DW_SPI_MAX_BLOCK_TIME);
}

void port_SPIx_set_chip_select(void) {
	spi_set_peripheral_chip_select_value(DW_SPI, DW_NONE_CHIP_SELECT_VALUE);
	spi_set_lastxfer(DW_SPI);
}

decaIrqStatus_t decamutexon(void) {
	decaIrqStatus_t bitstatus = 0x00;

	bitstatus = pio_get_interrupt_mask(DW_IRQ_PIO) & DW_IRQ_MASK;
	if (bitstatus != 0x00) {
		port_DisableEXT_IRQ();
	}

	return bitstatus;
}

void decamutexoff(decaIrqStatus_t s) {

	if (s != 0x00) {
		port_EnableEXT_IRQ();
	}
}

int peripherals_init(void) {
	NVIC_Configuration();

	return 0;
}

void * spi_peripheral_init() {
	return SPI_Configuration();
}
