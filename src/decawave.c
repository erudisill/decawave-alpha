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

#define DW_WRITE_MASK	0x80

static freertos_spi_if freertos_spi;

static uint8_t cmd_buffer[3];
static uint8_t data_buffer[20];

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

static status_code_t send_command(freertos_spi_if spi, uint8_t *_cmd_buffer, uint32_t cmd_size, uint8_t *_data_buffer,
		uint32_t data_size) {

	status_code_t result = STATUS_OK;

	spi_set_peripheral_chip_select_value(spi, (~(1U << CHIP_SELECT)));

	// Send the command
	if (freertos_spi_write_packet(spi, _cmd_buffer, cmd_size, SPI_MAX_BLOCK_TIME) == STATUS_OK) {
		if (_cmd_buffer[0] & DW_WRITE_MASK) {
			if (freertos_spi_write_packet(spi, _data_buffer, data_size, SPI_MAX_BLOCK_TIME) != STATUS_OK) {
				result = STATUS_ERR_TIMEOUT;
			}
		} else {
			if (freertos_spi_read_packet(spi, _data_buffer, data_size, SPI_MAX_BLOCK_TIME) != STATUS_OK) {
				result = STATUS_ERR_TIMEOUT;
			}
		}
	} else {
		result = STATUS_ERR_TIMEOUT;
	}

	spi_set_peripheral_chip_select_value(spi, NONE_CHIP_SELECT_VALUE);
	spi_set_lastxfer(spi);

	return result;
}

static void dw_read_device_id(void) {
	cmd_buffer[0] = 0x00;	// read device id

	if (send_command(freertos_spi, cmd_buffer, 1, data_buffer, 4) == STATUS_OK) {
		printf("DW DEVICE ID: %02X %02X %02X %02X\r\n", data_buffer[0], data_buffer[1], data_buffer[2], data_buffer[3]);
	} else {
		printf("SPI TIMEOUT\r\n");
	}
}

static void dw_test_eui(void) {
	cmd_buffer[0] = DW_WRITE_MASK | 0x01;	// WRITE EUI

	data_buffer[0] = 0xAA;
	data_buffer[1] = 0xBB;
	data_buffer[2] = 0xCC;
	data_buffer[3] = 0xDD;
	data_buffer[4] = 0xEE;
	data_buffer[5] = 0x88;
	data_buffer[6] = 0x55;
	data_buffer[7] = 0x22;

	if (send_command(freertos_spi, cmd_buffer, 1, data_buffer, 8) == STATUS_OK) {
		printf("DW WRITE EUI SUCCESS\r\n");
		cmd_buffer[0] = 0x01;	// READ EUI
		data_buffer[0] = 0x00;
		data_buffer[1] = 0x00;
		data_buffer[2] = 0x00;
		data_buffer[3] = 0x00;
		data_buffer[4] = 0x00;
		data_buffer[5] = 0x00;
		data_buffer[6] = 0x00;
		data_buffer[7] = 0x00;
		if (send_command(freertos_spi, cmd_buffer, 1, data_buffer, 8) == STATUS_OK) {
			printf("DW EUI: %02X %02X %02X %02X %02X %02X %02X %02X\r\n", data_buffer[0], data_buffer[1], data_buffer[2],
					data_buffer[3], data_buffer[4], data_buffer[5], data_buffer[6], data_buffer[7]);
		} else {
			printf("SPI TIMEOUT\r\n");
		}
	} else {
		printf("SPI TIMEOUT\r\n");
	}
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

		vTaskDelay(DELAY_1S);

		dw_read_device_id();

		vTaskDelay(DELAY_1S);

		dw_test_eui();
	}

}
