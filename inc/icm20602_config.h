#ifndef _ICM20602_CONFIG_H
#define _ICM20602_CONFIG_H

/***** Includes *****/

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/***** Defines *****/

#define SLEEP(ms) vTaskDelay((ms) / portTICK_PERIOD_MS)

#define MUTEX_LOCK()

#define MUTEX_UNLOCK()

#endif
