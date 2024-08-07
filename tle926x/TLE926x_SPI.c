/*********************************************************************************************************************
 * Copyright (c) 2021, Infineon Technologies AG
 *
 *
 * Distributed under the Boost Software License, Version 1.0.
 *
 *
 * Boost Software License - Version 1.0 - August 17th, 2003
 *
 * Permission is hereby granted, free of charge, to any person or organization
 * obtaining a copy of the software and accompanying documentation covered by
 * this license (the "Software") to use, reproduce, display, distribute,
 * execute, and transmit the Software, and to prepare derivative works of the
 * Software, and to permit third-parties to whom the Software is furnished to
 * do so, all subject to the following:
 *
 * The copyright notices in the Software and this entire statement, including
 * the above license grant, this restriction and the following disclaimer,
 * must be included in all copies of the Software, in whole or in part, and
 * all derivative works of the Software, unless such copies or derivative
 * works are solely in the form of machine-executable object code generated by
 * a source language processor.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT
 * SHALL THE COPYRIGHT HOLDERS OR ANYONE DISTRIBUTING THE SOFTWARE BE LIABLE
 * FOR ANY DAMAGES OR OTHER LIABILITY, WHETHER IN CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 *********************************************************************************************************************/

/********************************************************************************************************
 * @file        TLE926x_SPI.c
 *
 * @brief       Implementation of all SPI related functions
 *
 * @version     V1.0.0
 * @date
 * @author      Fedy Farhat
 * @author      Michael Schaffarczyk
 ********************************************************************************************************/

/* ================================================================================ */
/* ============================   HEADER FILES     ================================ */
/* ================================================================================ */

#include <tle926x/TLE926x_SPI.h>

/* ================================================================================ */
/* =======================   SPI communication functions     ====================== */
/* ================================================================================ */

/*RDK4 Platform Global Variables*/
cy_stc_scb_spi_context_t scbSPI_context;

const cy_stc_scb_spi_config_t scbSPI_config =
{
    .spiMode = CY_SCB_SPI_MASTER,
    .subMode = CY_SCB_SPI_MOTOROLA,
    .sclkMode = CY_SCB_SPI_CPHA1_CPOL0,
    .parity = CY_SCB_SPI_PARITY_NONE,
    .dropOnParityError = false,
    .oversample = 10,
    .rxDataWidth = 8UL,
    .txDataWidth = 8UL,
    .enableMsbFirst = false,
    .enableInputFilter = false,
    .enableFreeRunSclk = false,
    .enableMisoLateSample = true,
    .enableTransferSeperation = false,
    .ssPolarity = ((CY_SCB_SPI_ACTIVE_LOW << CY_SCB_SPI_SLAVE_SELECT0) | \
                                         (CY_SCB_SPI_ACTIVE_LOW << CY_SCB_SPI_SLAVE_SELECT1) | \
                                         (CY_SCB_SPI_ACTIVE_LOW << CY_SCB_SPI_SLAVE_SELECT2) | \
                                         (CY_SCB_SPI_ACTIVE_LOW << CY_SCB_SPI_SLAVE_SELECT3)),
    .ssSetupDelay = CY_SCB_SPI_SS_SETUP_DELAY_0_75_CYCLES,
    .ssHoldDelay = CY_SCB_SPI_SS_HOLD_DELAY_0_75_CYCLES,
    .ssInterDataframeDelay = CY_SCB_SPI_SS_INTERFRAME_DELAY_1_5_CYCLES,
    .enableWakeFromSleep = false,
    .rxFifoTriggerLevel = 7UL,
    .rxFifoIntEnableMask = 0UL,
    .txFifoTriggerLevel = 15UL,
    .txFifoIntEnableMask = 0UL,
    .masterSlaveIntEnableMask = 0UL,
};

void scbSPI_isr_callback(uint32_t event)
{
	/*The Non-Blocking transfer with a delay is finished here*/
	if(event == CY_SCB_SPI_TRANSFER_CMPLT_EVENT  )
	{

	}
}

void scbSPI_Interrupt(void)
{
    Cy_SCB_SPI_Interrupt(mSPI_HW, &scbSPI_context);
}

/* Initialize the SPI Interface */
uint8_t sbc_spi_init(void)
{
    cy_en_scb_spi_status_t result;
    cy_en_sysint_status_t sysSpistatus;

    /* Configure the SPI block */

    result = Cy_SCB_SPI_Init(mSPI_HW, &scbSPI_config, &scbSPI_context);
    if( result != CY_SCB_SPI_SUCCESS)
    {
        return 1;
    }

    /* Set active slave select to line 0 */
    Cy_SCB_SPI_SetActiveSlaveSelect(mSPI_HW, CY_SCB_SPI_SLAVE_SELECT0);

    const cy_stc_sysint_t scbSPI_SCB_IRQ_cfg =
    {
            .intrSrc      = mSPI_IRQ,
            .intrPriority = 3U
    };

    /* Hook interrupt service routine and enable interrupt */
    sysSpistatus = Cy_SysInt_Init(&scbSPI_SCB_IRQ_cfg, &scbSPI_Interrupt);
    if(sysSpistatus != CY_SYSINT_SUCCESS)
    {
        return 1;
    }

    /*Register the SPI Interrupt Callback*/
    Cy_SCB_SPI_RegisterCallback(mSPI_HW, scbSPI_isr_callback, &scbSPI_context);

    /* Enable interrupt in NVIC */
    NVIC_EnableIRQ(mSPI_IRQ);

    /* Enable the SPI Master block */
    Cy_SCB_SPI_Enable(mSPI_HW);

	return 0;
}

/* Initialize the SPI Interface */
void sbc_spi_deinit(void)
{
	/*SPI De-initialization*/
    NVIC_DisableIRQ(mSPI_IRQ);
    Cy_SCB_SPI_DeInit(mSPI_HW);
}

/*The SPI transfer function*/
uint16_t SBC_SPI_TRANSFER16(uint8_t Upper, uint8_t Lower)
{
	uint8_t spi_rx[2] = {0};
	uint8_t spi_tx[2];

	spi_tx[0] = Upper;
	spi_tx[1] = Lower;

	Cy_SCB_SPI_Transfer(mSPI_HW, spi_tx, spi_rx, 2, &scbSPI_context);

	/* Blocking wait for transfer completion */
	while (0UL != (CY_SCB_SPI_TRANSFER_ACTIVE & Cy_SCB_SPI_GetTransferStatus(mSPI_HW, &scbSPI_context)))
	{
	}

	return (uint16_t)(spi_rx[0]<<8 | spi_rx[1]);
}
