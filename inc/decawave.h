/*
 * decawave.h
 *
 *  Created on: Jun 30, 2015
 *      Author: ericrudisill
 */

#ifndef INC_DECAWAVE_H_
#define INC_DECAWAVE_H_


#include <asf.h>
#include "conf_board.h"
#include "conf_spi_master.h"
#include "port.h"
#include "deca_device_api.h"
#include "instance.h"

portTASK_FUNCTION_PROTO(task_decawave, pvParameters);
portTASK_FUNCTION_PROTO(task_decawave_tests, pvParameters);

#if DR_DISCOVERY == 0
extern void instancesetaddresses(instanceAddressConfig_t *plconfig);
#endif

#endif /* INC_DECAWAVE_H_ */
