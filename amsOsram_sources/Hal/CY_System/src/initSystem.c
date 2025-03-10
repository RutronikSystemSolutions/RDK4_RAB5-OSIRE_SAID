/*****************************************************************************
 * Copyright 2022 by ams OSRAM AG                                            *
 * All rights are reserved.                                                  *
 *                                                                           *
 * IMPORTANT - PLEASE READ CAREFULLY BEFORE COPYING, INSTALLING OR USING     *
 * THE SOFTWARE.                                                             *
 *                                                                           *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       *
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT         *
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS         *
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT  *
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,     *
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT          *
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,     *
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY     *
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT       *
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE     *
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.      *
 *****************************************************************************/

/*****************************************************************************/
/*****************************************************************************/
#include <amsOsram_sources/Hal/CY_Flash_EEPROM/inc/flash.h>
#include <amsOsram_sources/Hal/CY_Gpios/inc/pin.h>
#include <amsOsram_sources/Hal/CY_SPI/inc/SpiMaster.h>
#include <amsOsram_sources/Hal/CY_SPI/inc/SpiSlave.h>
#include <amsOsram_sources/Hal/CY_System/inc/initSystem.h>
#include <amsOsram_sources/Hal/CY_Uart/inc/uart.h>
#include <amsOsram_sources/Osp/inc/genericDevice.h>
#include <amsOsram_sources/SwTimer/inc/swTimer.h>
#include "i2c_slave.h"
#include "sys_timer.h"
#include "nonBlock_spi_timer.h"
#include "sbc_rab5_osire.h"
#include "cy_retarget_io.h"

cyhal_crc_t crc_obj;

/*****************************************************************************/
/*****************************************************************************/
void handle_error(uint32_t status);

void hal_init_clk(void)
{

}

/*****************************************************************************/
/*****************************************************************************/
void reset_wdt(void)
{

}

/*****************************************************************************/
/*****************************************************************************/
void lptmrISR(void)
{

}

/*****************************************************************************/
/*****************************************************************************/
void init_sys(void)
{
	cy_rslt_t result;

	/* Initialise the device and board peripherals */
	result = cybsp_init();
	if (result != CY_RSLT_SUCCESS)
	{
		CY_ASSERT(0);
	}

	/*Initialise the System Basis Chip*/
	sbc_rab5_osire_init();

	/* Initialise the CRC Generator */
	result = cyhal_crc_init(&crc_obj);
	if (result != CY_RSLT_SUCCESS)
	{
		CY_ASSERT(0);
	}

    /*Enable debug output via KitProg UART*/
    result = cy_retarget_io_init( KITPROG_TX, KITPROG_RX, 115200);
    if (result != CY_RSLT_SUCCESS)
    {
    	CY_ASSERT(0);
    }

	if (!sys_timer_init())
	{
		CY_ASSERT(0);
	}

    /*Initialize RDK2 I2C Slave Device*/
    result =  rdk4_i2c_slave_init();
    if (result != CY_RSLT_SUCCESS)
    {
    	CY_ASSERT(0);
    }


	/*amsOsram stack initialisations*/
	hal_init_pin();
	set_led_blue(0); //set LED to OFF or else they will light up
	set_led_red(0);
	set_led_green(0);
	handle_error(sw_Timer_100ms_init());
	CY_init_SPI_Master();
	CY_init_SPI_Slave();
	hal_init_flash();

	__enable_irq();

	/*RESET*/
	Cy_SysLib_Delay(10);
	osp_reset(0x00);
	Cy_SysLib_Delay(10);

	/*Initialise the timer for non-blocking SPI*/
	result = one_shot_spi_timer_init();
	if (result != CY_RSLT_SUCCESS)
	{
		CY_ASSERT(0);
	}
}

void handle_error(uint32_t status)
{
	if (status != CY_RSLT_SUCCESS)
	{
		CY_ASSERT(0);
	}
}
