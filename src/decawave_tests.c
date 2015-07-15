/*
 * decawave_tests.c
 *
 *  Created on: Jul 9, 2015
 *      Author: ericrudisill
 */

#if 1

#include <asf.h>
#include <string.h>
#include <inttypes.h>
#include <decawave.h>
#include <deca_regs.h>

static freertos_spi_if spi;
static uint8_t eui64[] = { 0x01, 0x00, 0x00, 0x00, 0x00, 0x02, 0xCA, 0xDE };
static uint8_t dr_mode = 0x02;	// Mode 3
static instanceConfig_t instConfig;

static uint8_t pll_cfg[] = { 0x08, 0x05, 0x40, 0x08, 0x26 };

typedef struct {
	uint8 channel;
	uint8 prf;
	uint8 datarate;
	uint8 preambleCode;
	uint8 preambleLength;
	uint8 pacSize;
	uint8 nsSFD;
	uint16 sfdTO;
} chConfig_t;

//Configuration for DecaRanging Modes (8 default use cases selectable by the switch S1 on EVK)
chConfig_t chConfig[8] = {
//mode 1 - S1: 7 off, 6 off, 5 off
		{ 2,              // channel
				DWT_PRF_16M,    // prf
				DWT_BR_110K,    // datarate
				3,             // preambleCode
				DWT_PLEN_1024,  // preambleLength
				DWT_PAC32,      // pacSize
				1,       // non-standard SFD
				(1025 + 64 - 32) //SFD timeout
		},
		//mode 2
		{ 2,              // channel
				DWT_PRF_16M,    // prf
				DWT_BR_6M8,    // datarate
				3,             // preambleCode
				DWT_PLEN_128,   // preambleLength
				DWT_PAC8,       // pacSize
				0,       // non-standard SFD
				(129 + 8 - 8) //SFD timeout
		},
		//mode 3
		{ 2,              // channel
				DWT_PRF_64M,    // prf
				DWT_BR_110K,    // datarate
				9,             // preambleCode
				DWT_PLEN_1024,  // preambleLength
				DWT_PAC32,      // pacSize
				1,       // non-standard SFD
				(1025 + 64 - 32) //SFD timeout
		},
		//mode 4
		{ 2,              // channel
				DWT_PRF_64M,    // prf
				DWT_BR_6M8,    // datarate
				9,             // preambleCode
				DWT_PLEN_128,   // preambleLength
				DWT_PAC8,       // pacSize
				0,       // non-standard SFD
				(129 + 8 - 8) //SFD timeout
		},
		//mode 5
		{ 5,              // channel
				DWT_PRF_16M,    // prf
				DWT_BR_110K,    // datarate
				3,             // preambleCode
				DWT_PLEN_1024,  // preambleLength
				DWT_PAC32,      // pacSize
				1,       // non-standard SFD
				(1025 + 64 - 32) //SFD timeout
		},
		//mode 6
		{ 5,              // channel
				DWT_PRF_16M,    // prf
				DWT_BR_6M8,    // datarate
				3,             // preambleCode
				DWT_PLEN_128,   // preambleLength
				DWT_PAC8,       // pacSize
				0,       // non-standard SFD
				(129 + 8 - 8) //SFD timeout
		},
		//mode 7
		{ 5,              // channel
				DWT_PRF_64M,    // prf
				DWT_BR_110K,    // datarate
				9,             // preambleCode
				DWT_PLEN_1024,  // preambleLength
				DWT_PAC32,      // pacSize
				1,       // non-standard SFD
				(1025 + 64 - 32) //SFD timeout
		},
		//mode 8
		{ 5,              // channel
				DWT_PRF_64M,    // prf
				DWT_BR_6M8,    // datarate
				9,             // preambleCode
				DWT_PLEN_128,   // preambleLength
				DWT_PAC8,       // pacSize
				0,       // non-standard SFD
				(129 + 8 - 8) //SFD timeout
		} };

static void print_eui(void) {
	dwt_geteui(eui64);
	printf("eui: ");
	printf("%02X ", eui64[7]);
	printf("%02X ", eui64[6]);
	printf("%02X ", eui64[5]);
	printf("%02X ", eui64[4]);
	printf("%02X ", eui64[3]);
	printf("%02X ", eui64[2]);
	printf("%02X ", eui64[1]);
	printf("%02X ", eui64[0]);
	printf("\r\n");
}

static void init_interrupts(void) {
	peripherals_init();
}


#if TESTING
static void init_spi(void) {

	pio_configure_pin(PIO_PA12_IDX, PIO_PERIPH_A);	// MISO
	pio_configure_pin(PIO_PA13_IDX, PIO_PERIPH_A);	// MOSI
	pio_configure_pin(PIO_PA14_IDX, PIO_PERIPH_A);	// SPCK
	pio_configure_pin(PIO_PC4_IDX, PIO_PERIPH_B);	// NPCS1
	pmc_enable_periph_clk(ID_SPI);

	spi_master_init(DW_SPI);

	spi_disable(DW_SPI);
	spi_set_clock_polarity(DW_SPI, DW_CHIP_SELECT, DW_CLOCK_POLARITY);
	spi_set_clock_phase(DW_SPI, DW_CHIP_SELECT, DW_CLOCK_PHASE);
	spi_set_baudrate_div(DW_SPI, DW_CHIP_SELECT, (sysclk_get_cpu_hz() / DW_SPI_BAUD_SLOW));
	spi_set_transfer_delay(DW_SPI, DW_CHIP_SELECT, DW_DELAY_BEFORE, DW_DELAY_BETWEEN);
	spi_configure_cs_behavior(DW_SPI, DW_CHIP_SELECT, SPI_CS_KEEP_LOW);
	spi_set_peripheral_chip_select_value(DW_SPI, DW_CHIP_SELECT_VALUE);
//	spi_set_variable_peripheral_select(DW_SPI);

	spi_enable(DW_SPI);
}
#else
static void init_spi(void) {
	spi = spi_peripheral_init();

	if (spi != NULL) {
		printf("FreeRTOS SPI master init success.\r\n");
	} else {
		printf("FreeRTOS SPI master init ERROR.\r\n");
		for (;;) {
		}
	}
}
#endif

static uint8_t init_dw(void) {
	dwt_config_t config;
	uint32_t dev_id;
	int result;

	dev_id = dwt_readdevid();
	printf("devID %08X\r\n", dev_id);

	//this is called here to wake up the device (i.e. if it was in sleep mode before the restart)
	//if the read of devide ID fails, the DW1000 could be asleep
	if (DWT_DEVICE_ID != dev_id) {
		port_SPIx_clear_chip_select();  //CS low
		Sleep(1); //200 us to wake up then waits 5ms for DW1000 XTAL to stabilise
		port_SPIx_set_chip_select();  //CS high
		Sleep(7);
		dev_id = dwt_readdevid();
		// SPI not working or Unsupported Device ID
		if (DWT_DEVICE_ID != dev_id) {
			return (-1);
		}
		//clear the sleep bit - so that after the hard reset below the DW does not go into sleep
		dwt_softreset();
	}

	//reset the DW1000 by driving the RSTn line low
	printf("resetting...");
	reset_DW1000();
	printf("done\r\n");

	result = dwt_initialise(DWT_LOADUCODE | DWT_LOADLDO | DWT_LOADTXCONFIG | DWT_LOADANTDLY | DWT_LOADXTALTRIM);
//	result = dwt_initialise(0);
	if (result == DWT_ERROR) {
		printf("dwt_initialise error\r\n");
		for (;;) {
		}
	}
	printf("dwt_initialise success\r\n");

	printf("configuring dw...");
    instConfig.channelNumber = chConfig[dr_mode].channel ;
    instConfig.preambleCode = chConfig[dr_mode].preambleCode ;
    instConfig.pulseRepFreq = chConfig[dr_mode].prf ;
    instConfig.pacSize = chConfig[dr_mode].pacSize ;
    instConfig.nsSFD = chConfig[dr_mode].nsSFD ;
    instConfig.sfdTO = chConfig[dr_mode].sfdTO ;
    instConfig.dataRate = chConfig[dr_mode].datarate ;
    instConfig.preambleLen = chConfig[dr_mode].preambleLength ;
    // Set operating channel etc
    instance_config(&instConfig) ;
    printf("done\r\n");

	dwt_enableframefilter(DWT_FF_NOTYPE_EN); //disable frame filtering
	dwt_seteui(eui64);
	dwt_setpanid(0xDECA);
	dwt_setrxaftertxdelay(0);
	dwt_setautorxreenable(0x01);
	dwt_setdblrxbuffmode(0x00);
	dwt_setrxtimeout(0);		// no timeout

	return 0;
}

void dw_test_config(void) {
	uint32_t dev_id;
	int result;

	// Potentially wakeup the device??
	dev_id = dwt_readdevid();
	printf("devID %08X\r\n", dev_id);

	//reset the DW1000 by driving the RSTn line low
	printf("resetting...");
	reset_DW1000();
	printf("done\r\n");

	dev_id = dwt_readdevid();
	printf("devID %08X\r\n", dev_id);

//	dwt_seteui(eui64);


//	dwt_writetodevice(FS_CTRL_ID, FS_PLLCFG_OFFSET, 5, &pll_cfg[0]);
//
//	// Defines for enable_clocks function
//#define FORCE_SYS_XTI  0
//#define ENABLE_ALL_SEQ 1
//#define FORCE_SYS_PLL  2
//#define READ_ACC_ON    7
//#define READ_ACC_OFF   8
//#define FORCE_OTP_ON   11
//#define FORCE_OTP_OFF  12
//#define FORCE_TX_PLL   13
//
//	_dwt_enableclocks(FORCE_SYS_PLL);
//	dwt_xtaltrim(0x1f);
}

void process_deca_irq(uint32_t id, uint32_t mask) {
	dwt_isr();
}

void process_dwRSTn_irq(uint32_t id, uint32_t mask) {
}

static uint8 buffer[40] = { 0, 0 };

#if TESTING
portTASK_FUNCTION(task_decawave_tests, pvParameters) {
	UNUSED(pvParameters);
	uint32 status = 0;
	uint32 status1 = 0;

	init_interrupts();

	init_spi();

	init_dw();

//	dw_test_config();

	print_eui();

	Sleep(2000);

	printf("Ready.\r\n");

//	port_EnableEXT_IRQ();
//
//	dwt_setinterrupt(DWT_INT_RFCG | DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RXPTO | DWT_INT_ARFE, 0x01);
//
//	dwt_xtaltrim(0x1f);

//	if (dwt_rxenable(0) == DWT_ERROR) {
//		printf("dwt_rxenable error\r\n");
//	}
//

	SPI_ConfigFastRate(DW_BAUD_RATE_FAST);

	for (;;) {

		Sleep(1000);

		//dwt_seteui(eui64);
		print_eui();

//		status = dwt_read32bitreg(SYS_STATUS_ID);
//		status1 = dwt_read32bitoffsetreg(SYS_STATUS_ID, 1);            // read status register
//		dwt_readfromdevice(RX_BUFFER_ID, 0, 2, buffer);
//		printf("MAIN status %02X%08X, byte 0 %02X%02X\r\n", status1 >> 24, status, buffer[0], buffer[1]);

//		memset(buffer, 0xaa, FS_CTRL_LEN);
//		dwt_readfromdevice(FS_CTRL_ID, 0, FS_CTRL_LEN, buffer);
//		printf("FS_PLLCFG %08X  FS_PLLTUNE %02X  FS_XTALT %02X\r\n", *((uint32_t*) &buffer[0x07]), (uint8_t) (buffer[0x0b]),
//				(uint8_t) (buffer[0x0e]));

//	    printf("FS_CTRL ");
//	    for (int i=0;i<FS_CTRL_LEN;i++) {
//	    	printf("%02X ", buffer[i]);
//	    }
//	    printf("\r\n");
	}

}
#else

portTASK_FUNCTION(task_decawave_tests, pvParameters) {
	UNUSED(pvParameters);
	uint32 status = 0;
	uint32 status1 = 0;
	uint32_t dev_id;

	init_interrupts();

	init_spi();

	init_dw();

//	dw_test_config();
//	dwt_seteui(eui64);
//	print_eui();

	Sleep(2000);

	printf("Ready.\r\n");

//	port_EnableEXT_IRQ();

//	dwt_setinterrupt(DWT_INT_RFCG | DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RXPTO | DWT_INT_ARFE, 0x01);

//	dwt_xtaltrim(0x1f);
//
	if (dwt_rxenable(0) == DWT_ERROR) {
		printf("dwt_rxenable error\r\n");
	}


	SPI_ConfigFastRate(DW_SPI_BAUD_FAST);

	for (;;) {

		Sleep(1000);

//		dev_id = dwt_readdevid();
//		dev_id = dwt_readdevid();
//		printf("devID %08X\r\n", dev_id);
////
////		dwt_seteui(eui64);
//		print_eui();

		dwt_readfromdevice(EUI_64_ID, 0x0, 8, eui64);
		dwt_readfromdevice(EUI_64_ID, 0x0, 8, eui64);
		print_eui();

		status = dwt_read32bitreg(SYS_STATUS_ID);
		status1 = dwt_read32bitoffsetreg(SYS_STATUS_ID, 1);            // read status register
		dwt_readfromdevice(RX_BUFFER_ID, 0, 2, buffer);
		printf("MAIN status %02X %08X, byte 0 %02X%02X\r\n", status1 >> 24, status, buffer[0], buffer[1]);

		memset(buffer, 0xaa, FS_CTRL_LEN);
		dwt_readfromdevice(FS_CTRL_ID, 0, FS_CTRL_LEN, buffer);
		printf("FS_PLLCFG %08X  FS_PLLTUNE %02X  FS_XTALT %02X\r\n", *((uint32_t*) &buffer[0x07]), (uint8_t) (buffer[0x0b]),
				(uint8_t) (buffer[0x0e]));

		status = dwt_read32bitoffsetreg(RF_CONF_ID, RF_STATUS_OFFSET);
		printf("RF_STATUS %08X\r\n", status);

//	    printf("FS_CTRL ");
//	    for (int i=0;i<FS_CTRL_LEN;i++) {
//	    	printf("%02X ", buffer[i]);
//	    }
//	    printf("\r\n");
	}

}
#endif


#endif
