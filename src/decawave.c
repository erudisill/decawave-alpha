/*
 * decawave.c
 *
 *  Created on: Jun 30, 2015
 *      Author: ericrudisill
 */

#include <asf.h>

#define SPI_MAX_BLOCK_TIME 	(50 / portTICK_RATE_MS)
#define CHIP_SELECT			1
#define CHIP_SELECT_VALUE	0x0001
#define NONE_CHIP_SELECT_VALUE     0x0f
#define BAUD_RATE 		1000000
#define DELAY_BEFORE	0x40
#define DELAY_BETWEEN	0x10
#define CLOCK_POLARITY	1
#define CLOCK_PHASE		1

static freertos_spi_if freertos_spi;

static uint8_t transmit_buffer[3];
static uint8_t receive_buffer[20];

static freertos_spi_if prepare_spi_port(Spi *spi_base) {

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

	spi_if = freertos_spi_master_init(spi_base, &driver_options);

	if (spi_if != NULL) {
		pio_configure_pin(PIO_PA12_IDX, PIO_PERIPH_A);	// MISO
		pio_configure_pin(PIO_PA13_IDX, PIO_PERIPH_A);	// MOSI
		pio_configure_pin(PIO_PA14_IDX, PIO_PERIPH_A);	// SPCK
		pio_configure_pin(PIO_PC4_IDX, PIO_PERIPH_B);	// NPCS1
		pmc_enable_periph_clk(ID_SPI);

		spi_disable(spi_base);
		spi_set_clock_polarity(spi_base, CHIP_SELECT, CLOCK_POLARITY);
		spi_set_clock_phase(spi_base, CHIP_SELECT, CLOCK_PHASE);
		spi_set_baudrate_div(spi_base, CHIP_SELECT, (sysclk_get_cpu_hz() / BAUD_RATE));
		spi_set_transfer_delay(spi_base, CHIP_SELECT, DELAY_BEFORE, DELAY_BETWEEN);
		spi_configure_cs_behavior(spi_base, CHIP_SELECT, SPI_CS_KEEP_LOW);
		spi_set_peripheral_chip_select_value(spi_base, CHIP_SELECT_VALUE);
		spi_enable(spi_base);
	}

	return spi_if;
}

portTASK_FUNCTION(task_decawave, pvParameters) {
	UNUSED(pvParameters);

	// prepare spi port
	freertos_spi = prepare_spi_port(SPI);

	if (freertos_spi != NULL) {
		printf("FreeRTOS SPI master init success.\r\n");
	} else {
		printf("FreeRTOS SPI master init ERROR.\r\n");
		for (;;) {
		}
	}

	// enter polling loop
	for (;;) {

		// take a breath ...
		vTaskDelay(DELAY_1S);

		transmit_buffer[0] = 0x00;	// read device id
		receive_buffer[0] = 0xAA;
		receive_buffer[1] = 0xAA;	// init receive buffer
		receive_buffer[2] = 0xAA;
		receive_buffer[3] = 0xAA;

		spi_set_peripheral_chip_select_value(SPI, (~(1U << CHIP_SELECT)));

		if (freertos_spi_write_packet(freertos_spi, transmit_buffer, 1, SPI_MAX_BLOCK_TIME) == STATUS_OK) {
			// read the bytes
			if (freertos_spi_read_packet(freertos_spi, receive_buffer, 4, SPI_MAX_BLOCK_TIME) == STATUS_OK) {
				printf("SPI received data %02X %02X %02X %02X\r\n", receive_buffer[0], receive_buffer[1], receive_buffer[2], receive_buffer[3]);
			} else {
				printf("SPI RX timed out\r\n");
			}
		}
		else {
			printf("SPI TX timed out\r\n");
		}

		spi_set_peripheral_chip_select_value(SPI, NONE_CHIP_SELECT_VALUE);
		spi_set_lastxfer(SPI);

	}

}
