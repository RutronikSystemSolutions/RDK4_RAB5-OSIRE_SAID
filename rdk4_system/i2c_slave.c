/*
 * i2c_slave.c
 *
 *  Created on: 2024-06-25
 *      Author: GDR
 */

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "i2c_slave.h"

/* Declare variables */
cyhal_i2c_t i2c_slave_obj;

/* Define address */
uint16_t I2C_SLAVE_ADDRESS = 0x08u;

/* Define frequency */
uint32_t I2C_SLAVE_FREQUENCY = 100000u;

uint8_t i2c_write_buffer[4] = {0x00, 0x00, 0x00, 0x00};
uint8_t i2c_read_buffer[4] = {0xCA, 0xFE, 0xBA, 0xBE};

void handle_i2c_events(void *callback_arg, cyhal_i2c_event_t event)
{
    /* To remove unused variable warning */
    (void) callback_arg;
    /* Check write complete event */
    if (0UL != (CYHAL_I2C_SLAVE_WR_CMPLT_EVENT & event))
    {
        /* Perform the required functions */
        cyhal_i2c_slave_config_write_buffer(&i2c_slave_obj, i2c_write_buffer,4);
    }
    /* Check read complete event */
    if (0UL != (CYHAL_I2C_SLAVE_RD_CMPLT_EVENT & event))
    {
        /* Perform the required functions */
    	cyhal_i2c_slave_config_read_buffer(&i2c_slave_obj, i2c_read_buffer,4);
    }
    /* Check for errors */
    if (0UL == (CYHAL_I2C_SLAVE_ERR_EVENT & event))
    {
        /* Perform the required function */
    }
}

cy_rslt_t rdk4_i2c_slave_init(void)
{
	cy_rslt_t rslt;

	/* Define the slave configuration structure */
	cyhal_i2c_cfg_t i2c_slave_config = {CYHAL_I2C_MODE_SLAVE, I2C_SLAVE_ADDRESS, I2C_SLAVE_FREQUENCY};

	/* Initialize I2C slave, set the SDA and SCL pins and assign a new clock */
	rslt = cyhal_i2c_init(&i2c_slave_obj, CYBSP_I2C_SDA, CYBSP_I2C_SCL, NULL);
	if(rslt != CY_RSLT_SUCCESS)
	{
		return rslt;
	}

	/* Configure the I2C resource to be slave */
	rslt = cyhal_i2c_configure(&i2c_slave_obj, &i2c_slave_config);

    /* Configure the read buffer on an I2C Slave */
    cyhal_i2c_slave_config_read_buffer(&i2c_slave_obj, i2c_read_buffer, 4);
    /* Configure the write buffer on an I2C Slave */
    cyhal_i2c_slave_config_write_buffer(&i2c_slave_obj, i2c_write_buffer, 4);

    /* Register I2C slave event callback */
    cyhal_i2c_register_callback(&i2c_slave_obj, (cyhal_i2c_event_callback_t) handle_i2c_events, NULL);

    /* Enable I2C Events */
    cyhal_i2c_enable_event(&i2c_slave_obj, (cyhal_i2c_event_t)   		\
                                 (CYHAL_I2C_SLAVE_WR_CMPLT_EVENT     	\
                                | CYHAL_I2C_SLAVE_RD_CMPLT_EVENT 		\
                                | CYHAL_I2C_SLAVE_ERR_EVENT),    		\
                                3u , true);

	return rslt;
}
