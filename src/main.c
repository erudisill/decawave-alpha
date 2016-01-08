#include <asf.h>
#include "conf_board.h"
#include "decawave.h"


static volatile bool bMonitorPaused = true;

extern void vApplicationMallocFailedHook(void) {
	taskDISABLE_INTERRUPTS();
	for (;;)
		;
}

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName) {
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *) pcTaskName);
	for (;;)
		;
}

extern void vApplicationIdleHook(void) {
}

extern void vApplicationTickHook(void) {
}

static void task_monitor(void *pvParameters) {
	static portCHAR szList[256];
	UNUSED(pvParameters);

	for (;;) {
		if (bMonitorPaused == false) {
			printf("--- task ## %u\r\n", (unsigned int) uxTaskGetNumberOfTasks());
			vTaskList((signed portCHAR *) szList);
			printf(szList);
		}
		vTaskDelay(DELAY_1S);
	}
}

static void task_led(void *pvParameters) {
	UNUSED(pvParameters);
	for (;;) {
		pio_toggle_pin(LED_STATUS0_IDX);
		vTaskDelay(DELAY_1S);
	}
}

// CHEAP CLI task to exercise getchar()
// A better solution is to use FreeRTOS+CLI
static void task_cli(void *pvParameters) {
	uint8_t c;
	UNUSED(pvParameters);
	for (;;) {
		c = getchar();
		if (c == 'm' || c == 'M') {
			bMonitorPaused = (bMonitorPaused ? false : true);
			printf("bMonitorPaused is now %d\r\n", (int)bMonitorPaused);
		}
	}
}

static void configure_console(void) {
	const usart_serial_options_t uart_serial_options = { .baudrate = CONF_UART_BAUDRATE, .paritytype = CONF_UART_PARITY, };
	stdio_serial_init(CONF_UART, &uart_serial_options);
}

int main(void) {
	sysclk_init();
	board_init();

	configure_console();

	/* Output demo infomation. */
	printf("\r\n-- Decawave Tests --\n\r");
	printf("-- %s\n\r", BOARD_NAME);
	printf("-- Compiled: %s %s --\n\r", __DATE__, __TIME__);
	printf("-- Press M to toggle Task Monitor\r\n");

	/* Create task to monitor processor activity */
	if (xTaskCreate(task_monitor, "Monitor", TASK_MONITOR_STACK_SIZE, NULL,TASK_MONITOR_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create Monitor task\r\n");
	}

	/* Create task to make led blink */
	if (xTaskCreate(task_led, "Led", TASK_LED_STACK_SIZE, NULL, TASK_LED_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create test led task\r\n");
	}

	/* Create task for Decawave */
	if (xTaskCreate(task_decawave, "Decawave", TASK_DW_STACK_SIZE, NULL, TASK_DW_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create Decawave task\r\n");
	}

	/* Create task for Decawave Tests */
//	if (xTaskCreate(task_decawave_tests, "DW TESTS", TASK_DW_STACK_SIZE, NULL, TASK_DW_STACK_PRIORITY, NULL) != pdPASS) {
//		printf("Failed to create DW TESTS task\r\n");
//	}

	/* Create task for cheap CLI */
	if (xTaskCreate(task_cli, "CLI", TASK_CLI_STACK_SIZE, NULL, TASK_CLI_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create CLI task\r\n");
	}

	/* Start the scheduler. */
	vTaskStartScheduler();

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}



#if 0
int main(void) {
	sysclk_init();
	board_init();

	configure_console();

	/* Output demo infomation. */
	printf("\r\n-- Decawave SPI TEST --\n\r");

	spi_set_master_mode(DW_SPI);

	pio_configure_pin(PIO_PA12_IDX, PIO_PERIPH_A);	// MISO
	pio_configure_pin(PIO_PA13_IDX, PIO_PERIPH_A);	// MOSI
	pio_configure_pin(PIO_PA14_IDX, PIO_PERIPH_A);	// SPCK
	pio_configure_pin(DW_CSn_PIO_IDX, DW_CSn_PIO_PERIPH);
	pmc_enable_periph_clk(ID_SPI);

	spi_disable(DW_SPI);
	spi_set_clock_polarity(DW_SPI, DW_CHIP_SELECT, DW_CLOCK_POLARITY);
	spi_set_clock_phase(DW_SPI, DW_CHIP_SELECT, DW_CLOCK_PHASE);
	spi_set_baudrate_div(DW_SPI, DW_CHIP_SELECT, (sysclk_get_cpu_hz() / DW_SPI_BAUD_SLOW));
	spi_set_transfer_delay(DW_SPI, DW_CHIP_SELECT, DW_DELAY_BEFORE,		DW_DELAY_BETWEEN);
	spi_configure_cs_behavior(DW_SPI, DW_CHIP_SELECT, SPI_CS_RISE_NO_TX);
	spi_set_peripheral_chip_select_value(DW_SPI, DW_CHIP_SELECT_VALUE);

	spi_enable(DW_SPI);

	uint8_t pcs = DW_CHIP_SELECT;
	uint16_t data = 0;
	uint8_t buff[5];

	spi_set_peripheral_chip_select_value(DW_SPI, (~(1U << DW_CHIP_SELECT)));


	spi_write(DW_SPI, 0, pcs, 0);
	spi_read(DW_SPI, &data,&pcs);
	buff[0] = data & 0xFF;

	spi_write(DW_SPI, 0xFF, pcs, 0);
	spi_read(DW_SPI, &data,&pcs);
	buff[1] = data & 0xFF;

	spi_write(DW_SPI, 0xFF, pcs, 0);
	spi_read(DW_SPI, &data,&pcs);
	buff[2] = data & 0xFF;

	spi_write(DW_SPI, 0xFF, pcs, 0);
	spi_read(DW_SPI, &data,&pcs);
	buff[3] = data & 0xFF;

	spi_write(DW_SPI, 0xFF, pcs, 1);
	spi_read(DW_SPI, &data,&pcs);
	buff[4] = data & 0xFF;

	spi_set_peripheral_chip_select_value(DW_SPI, DW_NONE_CHIP_SELECT_VALUE);


	printf("returned %02X%02X%02X%02X\r\n", buff[1], buff[2], buff[3], buff[4]);

	while (1) ;

}
#endif
