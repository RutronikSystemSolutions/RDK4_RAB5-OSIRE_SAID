/*
 * said.c
 *
 *  Created on: 2024-06-21
 *      Author: GDR
 */

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "stdio.h"
#include "said.h"

#include <amsOsram_sources/Selectives/osp2/inc/osp2.h>
#include <amsOsram_sources/Hal/CY_Gpios/inc/pin.h>

// Tests several of the basic SAID features
void said(void)
{
  osp2_error_t      err;
  uint16_t          last;
  uint8_t           temp;
  uint8_t           stat;
  uint8_t           flags;
  uint32_t          id;
  uint16_t          red;
  uint16_t          green;
  uint16_t          blue;
  uint8_t           rcur;
  uint8_t           gcur;
  uint8_t           bcur;
  uint8_t           daytimes;
  uint8_t           com;
  uint8_t           buf[8];

  err = 0;

  printf("\n\rASSUMPTION: MCU-OSIRE-OSIRE-SAID(RGB,RGB,I2C)-OSIRE-OSIRE\n\r");
  printf("\n\rINIT\n\r");

  err|=osp2_exec_reset();
  if(err)
  {
	  printf("reset command fail.\n\r");
	  goto exit_with_error;
  }
  err|=osp2_send_initbidir(1,&last,&temp,&stat);
  if(err)
  {
	  printf("initbidir command fail.\n\r");
	  goto exit_with_error;
  }

  /*Making sure we have RAB5-OSIRE mounted*/
  if(last < 5)
  {
	  printf("RAB5-OSIRE adapter board not detected.\n\r");
	  goto exit_with_error;
  }
  else
  {
	  osp2_send_identify(3,&id);
	  if(id != 0x40)
	  {
		  printf("RAB5-OSIRE adapter board not detected.\n\r");
		  goto exit_with_error;
	  }
  }

  printf("\n\rSETUP (enable CRC)\n\r");
  err|=osp2_send_setsetup(1, OSP2_SETUP_FLAGS_OSIRE_DFLT | OSP2_SETUP_FLAGS_CRCEN );
  err|=osp2_send_setsetup(2, OSP2_SETUP_FLAGS_OSIRE_DFLT | OSP2_SETUP_FLAGS_CRCEN );
  err|=osp2_send_setsetup(3, OSP2_SETUP_FLAGS_SAID_DFLT  | OSP2_SETUP_FLAGS_CRCEN );
  err|=osp2_send_setsetup(4, OSP2_SETUP_FLAGS_OSIRE_DFLT | OSP2_SETUP_FLAGS_CRCEN );
  err|=osp2_send_setsetup(5, OSP2_SETUP_FLAGS_OSIRE_DFLT | OSP2_SETUP_FLAGS_CRCEN );
  err|=osp2_send_readsetup(1,&flags );
  printf("Device 1 setup flags: 0x%2X\n\r", flags);
  err|=osp2_send_readsetup(2,&flags );
  printf("Device 2 setup flags: 0x%2X\n\r", flags);
  err|=osp2_send_readsetup(3,&flags );
  printf("Device 3 setup flags: 0x%2X\n\r", flags);
  err|=osp2_send_readsetup(4,&flags );
  printf("Device 4 setup flags: 0x%2X\n\r", flags);
  err|=osp2_send_readsetup(5,&flags );
  printf("Device 5 setup flags: 0x%2X\n\r", flags);

  printf("\n\rSTATUS CHECK\n\r");
  err|=osp2_send_clrerror(0); // all SAIDs have the V flag (overvoltage) after reset, clear those
  err|=osp2_send_readtempstat(3, &temp, &stat);

  printf("\n\rID'S\n\r");
  err|=osp2_send_readotp(3,0x00,buf,3);
  err|=osp2_send_identify(1,&id);
  printf("Device 1 ID: 0x%2X\n\r", (unsigned int)id);
  err|=osp2_send_identify(2,&id);
  printf("Device 2 ID: 0x%2X\n\r", (unsigned int)id);
  err|=osp2_send_identify(3,&id);
  printf("Device 3 ID: 0x%2X\n\r", (unsigned int)id);
  err|=osp2_send_identify(4,&id);
  printf("Device 4 ID: 0x%2X\n\r", (unsigned int)id);
  err|=osp2_send_identify(5,&id);
  printf("Device 5 ID: 0x%2X\n\r", (unsigned int)id);

  printf("\n\rPWM ON\n\r");
  err|=osp2_send_setpwm   (1,  0x0200,0x0002,0x0020,0b000);
  err|=osp2_send_setpwm   (2,  0x0200,0x0002,0x0020,0b000);
  err|=osp2_send_setpwmchn(3,0,0x3333,0x2222,0x1111);
  err|=osp2_send_setpwmchn(3,1,0x3333,0x2222,0x1111);
  err|=osp2_send_setpwm   (4,  0x0200,0x0002,0x0020,0b000);
  err|=osp2_send_setpwm   (5,  0x0200,0x0002,0x0020,0b000);

  err|=osp2_send_readpwm(1,&red, &green, &blue, &daytimes );
  err|=osp2_send_readpwm(2,&red, &green, &blue, &daytimes );
  err|=osp2_send_readpwmchn(3,0, &red, &green, &blue );
  err|=osp2_send_readpwmchn(3,1, &red, &green, &blue );
  err|=osp2_send_readpwm(4,&red, &green, &blue, &daytimes );
  err|=osp2_send_readpwm(5,&red, &green, &blue, &daytimes );

  err|=osp2_send_goactive(0);

  printf("\n\rSYNC\n\r");
  err|=osp2_send_readcurchn(3,1, &flags, &rcur, &gcur, &bcur);
  err|=osp2_send_setcurchn(3,1, OSP2_CURCHN_FLAGS_SYNCEN, 0, 0, 0);
  err|=osp2_send_setpwmchn(3,1, 0x3333,0x2222,0x1111); // swap blue to green
  err|=osp2_send_sync(0);

  printf("\n\rTEMP\n\r");
  err|=osp2_send_readtemp(1, &temp);
  printf("Device 1 Temperature Register: %d\n\r", temp);
  err|=osp2_send_readtemp(2, &temp);
  printf("Device 2 Temperature Register: %d\n\r", temp);
  err|=osp2_send_readtemp(3, &temp);
  printf("Device 3 Temperature Register: %d\n\r", temp);
  err|=osp2_send_readtemp(4, &temp);
  printf("Device 4 Temperature Register: %d\n\r", temp);
  err|=osp2_send_readtemp(5, &temp);
  printf("Device 5 Temperature Register: %d\n\r", temp);

  printf("\n\rCOM STATUS\n\r");
  err|=osp2_send_readcomst(1, &com);
  printf("Device 1 com status: %d\n\r", com);
  err|=osp2_send_readcomst(2, &com);
  printf("Device 2 com status: %d\n\r", com);
  err|=osp2_send_readcomst(3, &com);
  printf("Device 3 com status: %d\n\r", com);
  err|=osp2_send_readcomst(4, &com);
  printf("Device 4 com status: %d\n\r", com);
  err|=osp2_send_readcomst(5, &com);
  printf("Device 5 com status: %d\n\r", com);

  printf("\n\rSTATUS CHECK\n\r");
  err|=osp2_send_readtempstat(1, &temp, &stat);
  if( stat & OSP2_SETUP_FLAGS_OSIRE_ERRORS ) { printf("ERROR status\n\r"); err|=OSP2_ERROR_STATUSFLAGS; }
  err|=osp2_send_readtempstat(2, &temp, &stat);
  if( stat & OSP2_SETUP_FLAGS_OSIRE_ERRORS ) { printf("ERROR status\n\r"); err|=OSP2_ERROR_STATUSFLAGS; }
  err|=osp2_send_readtempstat(3, &temp, &stat);
  if( stat & OSP2_SETUP_FLAGS_SAID_ERRORS ) { printf("ERROR status\n\r"); err|=OSP2_ERROR_STATUSFLAGS; }
  err|=osp2_send_readtempstat(4, &temp, &stat);
  if( stat & OSP2_SETUP_FLAGS_OSIRE_ERRORS ) { printf("ERROR status\n\r"); err|=OSP2_ERROR_STATUSFLAGS; }
  err|=osp2_send_readtempstat(5, &temp, &stat);
  if( stat & OSP2_SETUP_FLAGS_OSIRE_ERRORS ) { printf("ERROR status\n\r"); err|=OSP2_ERROR_STATUSFLAGS; }

  printf("\n\rFORCE ARG ERROR\n\r");
  err|=osp2_send_setsetup(3, OSP2_SETUP_FLAGS_SAID_DFLT | OSP2_SETUP_FLAGS_CRCEN | OSP2_SETUP_FLAGS_CE );
  err|=osp2_send_readsetup(3,&flags );
  printf("## setpwm to SAID is a comm error, that will switch to SLEEP and thus LEDs off\n\r");
  err|=osp2_send_setpwm(3,0x3333,0x2222,0x1111,0b000);
  err|=osp2_send_readstat(3, &stat);
  err|=osp2_send_readstat(3, &stat);
  printf("## error cleared, switch SAID to active\n\r");
  err|=osp2_send_goactive(3);

  printf("\n\rOTP TEST\n\r");
  osp2_exec_i2cenable_set(3,1);
  osp2_exec_otpdump(3);

  if( err==OSP2_ERROR_NONE )
  {
    printf("\n\rSAID tests: PASS\n\r");
  }
  else
  {
	exit_with_error:
    printf("\n\rSAID tests: FAILED\n\r");
  }
}

/* A simple animation on RAB5-OSIRE SAID */
void sidelookers_animate(void)
{
	static _Bool init = false;
	static int chn=0;
	static int col=0;
	uint8_t flags;
	uint8_t rcur;
	uint8_t gcur;
	uint8_t bcur;
	uint16_t last;
	uint8_t stat,temp;

  if(!init)
  {
	  osp2_exec_reset();
	  osp2_send_initbidir(1,&last,&temp,&stat);

	  // cur   0    1    2    3    4
	  // chn0  3mA  6mA 12mA 24mA 48mA
	  // chn1 1.5mA 3mA  6mA 12mA 24mA

	  /* Set LED4 current (channel 0) */
	  flags = 0;
	  rcur = 2;
	  gcur = 2;
	  bcur = 2;
	  osp2_send_setcurchn(3, 0, flags, rcur, gcur, bcur);

	  /*Set LED3 current (channel 1)*/
	  flags = 0;
	  rcur = 3;
	  gcur = 3;
	  bcur = 3;
	  osp2_send_setcurchn(3, 1, flags, rcur, gcur, bcur);

	  /* Every SAID boots with over voltage, clear that flag */
	  osp2_send_clrerror(0);
	  osp2_send_readstat(3, &stat);

	  /*Activate the SAID*/
	  osp2_send_goactive(3);
	  init = true;
  }

  /*Animation*/
  if(chn<2)
  {
	  if(col<3)
	  {
	        uint16_t red   = col==0 ? 0x3FFF : 0;
	        uint16_t green = col==1 ? 0x3FFF : 0;
	        uint16_t blue  = col==2 ? 0x3FFF : 0;
	        osp2_send_setpwmchn(3,chn,red,green,blue);
	        col++;
	  }
	  else
	  {
		  col=0;
		  osp2_send_setpwmchn(3,chn,0,0,0);
		  chn++;
	  }
  }
  else
  {
	  chn=0;
  }
}


// This tests the difference between authenticated and test mode
void testmode(void)
{
  osp2_error_t err = OSP2_ERROR_NONE;
  uint16_t last;
  uint8_t stat,temp;
  uint8_t buf[8];

  printf("\n\rRESET, INIT, STATUS\n\r");
  err|=osp2_exec_reset();
  err|=osp2_send_initbidir(1,&last,&temp,&stat);
  //err|=osp2_send_initloop(1,&last,&temp,&stat);
  err|= osp2_send_readstat(3, &stat); // Clear UV then check status (testmode bit)
  err|= osp2_send_readstat(3, &stat);   printf("testmode %s\n",stat & OSP2_STATUS_FLAGS_TESTMODE ? "yes":"no");


  printf("\n\rEnable TESTMODE, STATUS\n\r");
  err|= osp2_send_testpw(3,0x4247525F4143ULL); // Enable authenticated mode
  err|= osp2_send_readstat(3, &stat);   printf("testmode %s\n",stat & OSP2_STATUS_FLAGS_TESTMODE ? "yes":"no");
  err|= osp2_send_settestdata(3,0b00001); // Enable test mode
  err|= osp2_send_readstat(3, &stat);   printf("testmode %s\n",stat & OSP2_STATUS_FLAGS_TESTMODE ? "yes":"no");
  err|= osp2_exec_otpdump(3);

//  err|= osp2_send_setcurchn(2, 0, 0, 4, 4, 4);
//  err|= osp2_send_setcurchn(2, 1, 0, 4, 4, 4);
  err|= osp2_send_setotp(2,0x0D,buf,7);
  err|= osp2_exec_otpdump(2);
  err|= osp2_send_readtempstat(2,&temp, &stat);

  printf("\n\rDisable TESTMODE, STATUS\n\r");
  err|= osp2_send_settestdata(3,0b11111); // Disable test mode
  err|= osp2_send_readstat(3, &stat);   printf("testmode %s\n",stat & OSP2_STATUS_FLAGS_TESTMODE ? "yes":"no");
  err|= osp2_send_testpw(3,0); // Disable authenticated mode
  err|= osp2_send_readstat(2, &stat);   printf("testmode %s\n",stat & OSP2_STATUS_FLAGS_TESTMODE ? "yes":"no");

  if( err==OSP2_ERROR_NONE ) {
    printf("\n\rdone-with-success\n\r");
    set_led_green(1);
  } else {
    printf("\n\rDONE-WITH-ERROR\n\r");
    set_led_red(1);
  }
}


/* Reads the data from RDK4 I2C slave */
void said_i2c_test(uint8_t addr)
{
#define on_error_gotoexit()   do { if( err!=OSP2_ERROR_NONE ) goto exit; } while(0)

	uint8_t rdk4_addr = 0x08; // I2C address of RDK4 Slave I2C Device
	osp2_error_t err = OSP2_ERROR_NONE;
	uint16_t last;
	uint8_t temp;
	uint8_t stat;
	int enable;
	uint32_t id;
	uint8_t buf[4];

	printf("\n\rI2C test in progress...\n\r");

	/* Configure  SAID for I2C (with several checks) */
	osp2_exec_reset();
	err = osp2_send_initbidir(1, &last, &temp, &stat);
	on_error_gotoexit();
	if (addr > last)
	{
		err = OSP2_ERROR_ADDR;
		goto exit;
	}
	err = osp2_send_clrerror(0);
	on_error_gotoexit();
	err = osp2_send_setsetup(addr,OSP2_SETUP_FLAGS_SAID_DFLT | OSP2_SETUP_FLAGS_CRCEN);
	on_error_gotoexit();
	err = osp2_send_identify(addr, &id);
	on_error_gotoexit();
	if (id != 0x00000040)
	{
		err = OSP2_ERROR_ID;
		goto exit;
	}
	osp2_exec_i2cenable_get(addr, &enable);
	on_error_gotoexit();
	if (!enable)
	{
		err = OSP2_ERROR_MISSI2CBRIDGE;
		goto exit;
	}
	err = osp2_send_setcurchn(addr, /*chan*/2, /*flags*/0, 4, 4, 4);
	on_error_gotoexit();
	err = osp2_send_seti2ccfg(addr, OSP2_I2CCFG_FLAGS_DEFAULT,
			OSP2_I2CCFG_SPEED_DEFAULT);
	on_error_gotoexit();
	err = osp2_exec_i2cread8(addr, rdk4_addr, 0x00, buf, 4);
	if (err != OSP2_ERROR_I2CNACK && err != OSP2_ERROR_I2CTIMEOUT && err != OSP2_ERROR_NONE)
	{
		on_error_gotoexit();
	}

	printf("The I2C data from RDK4 slave: 0x%2X%2X%2X%2X\n\r",
			(unsigned int) buf[0], (unsigned int) buf[1],
			(unsigned int) buf[2], (unsigned int) buf[3]);

	err = OSP2_ERROR_NONE;

	exit: if (err == OSP2_ERROR_NONE)
	{
		printf("I2C test successful\n\r");
	}
	else
	{
		printf("I2C test failure\n\r");
	}
}

void parallel (void)
{
	  uint16_t          last;
	  uint8_t           temp;
	  uint8_t           stat;
//	  uint32_t          id;
//	  uint8_t           com;


	  osp2_exec_reset();
	  osp2_send_initbidir(1,&last,&temp,&stat);
	  osp2_send_initbidir(0x080,&last,&temp,&stat);
	  osp2_send_initbidir(0x100,&last,&temp,&stat);
//	  osp2_send_identify(1,&id);
//	  osp2_send_identify(0x080,&id);
	  //osp2_send_identify(0x100,&id);
//	  osp2_send_identify(0x101,&id);
//	  osp2_send_identify(2,&id);
	  osp2_send_clrerror(0);
	  osp2_send_goactive(0);
//	  osp2_send_readcomst(1, &com);
	  //osp2_send_readcomst(0x080, &com);
	  //osp2_send_readcomst(0x100, &com);
//	  osp2_send_setpwmchn(0x100,1,0x1FFF,0x1FFF,0x1FFF);
//	  osp2_send_setpwmchn(0x100,2,0x1FFF,0x1FFF,0x1FFF);
	  //osp2_send_setpwmchn(0x100,0,0x1FFF,0x1FFF,0x1FFF);
//	  osp2_send_setpwmchn(0x101,1,0x1FFF,0x1FFF,0x1FFF);
//	  osp2_send_setpwmchn(0x101,2,0x1FFF,0x1FFF,0x1FFF);
	  osp2_send_setpwmchn(0,0,0x1FFF,0x1FFF,0x1FFF);
	  //osp2_send_setpwmchn(0x100,0,0x1FFF,0x1FFF,0x1FFF);


	  osp2_send_readstat(1, &stat);
	  osp2_send_readstat(2, &stat);
	  osp2_send_readstat(0x080, &stat);
	  osp2_send_readstat(0x100, &stat);
	  osp2_send_readstat(0x101, &stat);




}

/* Sets THE PWM of both E5515 LEDs*/
void PWM(void)
{
	  uint16_t          last;
	  uint8_t           temp;
	  uint8_t           stat;
	  uint8_t           flags;
	  uint16_t			pwm = 0x3FFF;
	  uint8_t           com;

	  osp2_exec_reset();
	  osp2_send_initbidir(1,&last,&temp,&stat);
	  osp2_send_clrerror(0);
	  osp2_send_goactive(0);
	  osp2_send_setsetup(RAB5_SAID_ADDR, OSP2_SETUP_FLAGS_OSIRE_DFLT | OSP2_SETUP_FLAGS_CRCEN | OSP2_SETUP_FLAGS_LOS);
	  osp2_send_readsetup(RAB5_SAID_ADDR,&flags );
	  osp2_send_setpwm   (RAB5_SAID_ADDR,  0x0080,0x0002,0x0020,0b000);
	  osp2_send_setpwmchn(RAB5_SAID_ADDR,0,pwm,pwm,pwm);
	  osp2_send_setpwmchn(RAB5_SAID_ADDR,1,pwm,pwm,pwm);
	  osp2_send_readstat(RAB5_SAID_ADDR, &stat);
	  osp2_send_readcomst(RAB5_SAID_ADDR, &com);
}




