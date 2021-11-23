/*
 * Accelerometer.c
 *
 *  Created on: 22 nov. 2021
 *      Author: Fernanda Galeana
 */
#include "Accelerometer.h"

#include "fsl_debug_console.h"
#include "board.h"
#include "math.h"
#include "fsl_i2c.h"
#include "fsl_tpm.h"
#include "clock_config.h"
#include "pin_mux.h"
#include "fsl_gpio.h"
#include "fsl_port.h"
#include "fsl_fxos.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* The TPM instance/channel used for board */
#define BOARD_TIMER_BASEADDR TPM2
#define BOARD_FIRST_TIMER_CHANNEL 0U
#define BOARD_SECOND_TIMER_CHANNEL 1U
/* Get source clock for TPM driver */
#define BOARD_TIMER_SOURCE_CLOCK CLOCK_GetFreq(kCLOCK_Osc0ErClk)
#define TIMER_CLOCK_MODE 1U
/* I2C source clock */
#define ACCEL_I2C_CLK_SRC I2C1_CLK_SRC
#define I2C_BAUDRATE 100000U

#define I2C_RELEASE_SDA_PORT PORTC
#define I2C_RELEASE_SCL_PORT PORTC
#define I2C_RELEASE_SDA_GPIO GPIOC
#define I2C_RELEASE_SDA_PIN 3U
#define I2C_RELEASE_SCL_GPIO GPIOC
#define I2C_RELEASE_SCL_PIN 2U
#define I2C_RELEASE_BUS_COUNT 100U
/* Upper bound and lower bound angle values */
#define ANGLE_UPPER_BOUND 85U
#define ANGLE_LOWER_BOUND 5U
#define BOARD_ACCEL_I2C_BASEADDR I2C1
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void BOARD_I2C_ReleaseBus(void);

/*******************************************************************************
 * Variables
 ******************************************************************************/
i2c_master_handle_t g_MasterHandle;
/* FXOS device address */
const uint8_t g_accel_address[] = {0x1CU, 0x1DU, 0x1EU, 0x1FU};

fxos_handle_t g_fxosHandle;

/*******************************************************************************
 * Code
 ******************************************************************************/

static void i2c_release_bus_delay(void)
{
    uint32_t i = 0;
    for (i = 0; i < I2C_RELEASE_BUS_COUNT; i++)
    {
        __NOP();
    }
}

void BOARD_I2C_ReleaseBus(void)
{
    uint8_t i = 0;
    gpio_pin_config_t pin_config;
    port_pin_config_t i2c_pin_config = {0};

    /* Config pin mux as gpio */
    i2c_pin_config.pullSelect = kPORT_PullUp;
    i2c_pin_config.mux = kPORT_MuxAsGpio;

    pin_config.pinDirection = kGPIO_DigitalOutput;
    pin_config.outputLogic = 1U;
    CLOCK_EnableClock(kCLOCK_PortC);
    PORT_SetPinConfig(I2C_RELEASE_SCL_PORT, I2C_RELEASE_SCL_PIN, &i2c_pin_config);
    PORT_SetPinConfig(I2C_RELEASE_SDA_PORT, I2C_RELEASE_SDA_PIN, &i2c_pin_config);

    GPIO_PinInit(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, &pin_config);
    GPIO_PinInit(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, &pin_config);

    /* Drive SDA low first to simulate a start */
    GPIO_WritePinOutput(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, 0U);
    i2c_release_bus_delay();

    /* Send 9 pulses on SCL and keep SDA high */
    for (i = 0; i < 9; i++)
    {
        GPIO_WritePinOutput(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, 0U);
        i2c_release_bus_delay();

        GPIO_WritePinOutput(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, 1U);
        i2c_release_bus_delay();

        GPIO_WritePinOutput(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, 1U);
        i2c_release_bus_delay();
        i2c_release_bus_delay();
    }

    /* Send stop */
    GPIO_WritePinOutput(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, 0U);
    i2c_release_bus_delay();

    GPIO_WritePinOutput(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, 0U);
    i2c_release_bus_delay();

    GPIO_WritePinOutput(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, 1U);
    i2c_release_bus_delay();

    GPIO_WritePinOutput(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, 1U);
    i2c_release_bus_delay();
}
bool Accel_Init(void)
{
	    i2c_master_config_t i2cConfig;
	    uint32_t i2cSourceClock;
	    uint8_t i = 0;
	    uint8_t regResult = 0;
	    uint8_t array_addr_size = 0;
	    bool foundDevice = false;
	    bool status = true;
	    BOARD_BootClockRUN();
	    BOARD_I2C_ReleaseBus();
	    BOARD_I2C_ConfigurePins();

	    i2cSourceClock = CLOCK_GetFreq(ACCEL_I2C_CLK_SRC);
	    g_fxosHandle.base = BOARD_ACCEL_I2C_BASEADDR;
	    g_fxosHandle.i2cHandle = &g_MasterHandle;

	    I2C_MasterGetDefaultConfig(&i2cConfig);
	    I2C_MasterInit(BOARD_ACCEL_I2C_BASEADDR, &i2cConfig, i2cSourceClock);
	    I2C_MasterTransferCreateHandle(BOARD_ACCEL_I2C_BASEADDR, &g_MasterHandle, NULL, NULL);

	    /* Find sensor devices */
	    array_addr_size = sizeof(g_accel_address) / sizeof(g_accel_address[0]);
	    for (i = 0; i < array_addr_size; i++)
	    {
	        g_fxosHandle.xfer.slaveAddress = g_accel_address[i];
	        if (FXOS_ReadReg(&g_fxosHandle, WHO_AM_I_REG, &regResult, 1) == kStatus_Success)
	        {
	            foundDevice = true;
	            break;
	        }
	        if ((i == (array_addr_size - 1)) && (!foundDevice))
	        {
	            while (1)
	            {
	            };
	        }
	    }
	    /* Init accelerometer sensor */
	    if (FXOS_Init(&g_fxosHandle) != kStatus_Success)
	    {
	        status = false;
	    }
	    return(status);
}
bool GetAccelData(accel_data* data_read)
{
	fxos_data_t sensorData;
	int16_t x, y, z;
	bool status = true;
    /* Get new accelerometer data. */
    if (FXOS_ReadSensorData(&g_fxosHandle, &sensorData) != kStatus_Success)
    {
        data_read->xData = 0;
        data_read->yData = 0;
        data_read->zData = 0;
        status = false;
    }

    /* Get the X and Y data from the sensor data structure in 14 bit left format data*/
    data_read->xData = (int16_t)((uint16_t)((uint16_t)sensorData.accelXMSB << 8) | (uint16_t)sensorData.accelXLSB) / 4U;
    data_read->yData = (int16_t)((uint16_t)((uint16_t)sensorData.accelYMSB << 8) | (uint16_t)sensorData.accelYLSB) / 4U;
    data_read->zData = (int16_t)((uint16_t)((uint16_t)sensorData.accelZMSB << 8) | (uint16_t)sensorData.accelZLSB) / 4U;
    return(status);
}

