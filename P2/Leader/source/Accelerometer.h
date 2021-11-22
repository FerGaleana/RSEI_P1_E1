/*
 * Accelerometer.h
 *
 *  Created on: 22 nov. 2021
 *      Author: Fernanda Galeana
 */

#ifndef ACCELEROMETER_H_
#define ACCELEROMETER_H_
#include "fsl_common.h"
#include "EmbeddedTypes.h"
#include "fsl_os_abstraction.h"

typedef struct
{
	int16_t xData;
	int16_t yData;
	int16_t zData;
}accel_data;

bool Accel_Init(void);
bool GetAccelData(accel_data* data_read);
#endif /* ACCELEROMETER_H_ */
