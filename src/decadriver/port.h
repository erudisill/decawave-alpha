/*
 * port.h
 *
 *  Created on: Jul 1, 2015
 *      Author: ericrudisill
 */

#ifndef SRC_DECADRIVER_PORT_H_
#define SRC_DECADRIVER_PORT_H_

#include <asf.h>
#include "deca_device_api.h"

#define __weak              __attribute__((weak))

extern int writetospi_serial(uint16_t headerLength, const uint8_t *headerBuffer, uint32_t bodylength, const uint8_t *bodyBuffer);
extern int readfromspi_serial(uint16_t headerLength, const uint8_t *headerBuffer, uint32_t readlength, uint8_t *readBuffer);

#define writetospi		writetospi_serial
#define readfromspi		readfromspi_serial

#define Sleep(x)				vTaskDelay(x / portTICK_RATE_MS)
#define portGetTickCount()		(xTaskGetTickCount() / portTICK_RATE_MS)

#define SPI_BaudRatePrescaler_16	DW_SPI_BAUD_SLOW
#define SPI_BaudRatePrescaler_4		DW_SPI_BAUD_FAST

#define port_CheckEXT_IRQ()			pio_get_pin_value(DW_IRQ_IDX)
#define port_EnableEXT_IRQ()		pio_enable_interrupt(DW_IRQ_PIO, DW_IRQ_MASK)
#define port_DisableEXT_IRQ()		pio_disable_interrupt(DW_IRQ_PIO, DW_IRQ_MASK)

decaIrqStatus_t decamutexon(void);
void decamutexoff(decaIrqStatus_t s);

typedef enum
{
    LED_PC6,
    LED_PC7,
    LED_PC8,
    LED_PC9,
    LED_ALL,
    LEDn
} led_t;

void SPI_ConfigFastRate(uint32_t baudrate);
void setup_DW1000RSTnIRQ(int enable);
void reset_DW1000(void);

extern void process_deca_irq(uint32_t, uint32_t);
extern void process_dwRSTn_irq(uint32_t, uint32_t);

void port_SPIx_clear_chip_select(void);
void port_SPIx_set_chip_select(void);
int peripherals_init (void);
void * spi_peripheral_init(void);

void led_off(led_t led);
void led_on(led_t led);

#endif /* SRC_DECADRIVER_PORT_H_ */
