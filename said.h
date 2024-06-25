/*
 * said.h
 *
 *  Created on: 2024-06-21
 *      Author: GDR
 */

#ifndef SAID_H_
#define SAID_H_

#define RAB5_SAID_ADDR		(3U)

// Tests several of the basic SAID features
void said(void);

// Tests several SAID I2C features
void i2c(void);

// A simple animation on SAID at addr 2
void anim(void);

// This tests the difference between authenticated and test mode
void testmode(void);

// This scans the I2C bus (on SAID `addr`) for devices
void i2cscan( uint8_t addr);

// This tests the IO expander on SAID `addr`
void iox( uint8_t addr);

void parallel (void);

void PWM(void);

#endif /* SAID_H_ */
