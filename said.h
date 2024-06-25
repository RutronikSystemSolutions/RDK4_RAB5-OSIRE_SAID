/*
 * said.h
 *
 *  Created on: 2024-06-21
 *      Author: GDR
 */

#ifndef SAID_H_
#define SAID_H_

#define RAB5_SAID_ADDR		(3U)

/* Tests several of the basic SAID features and initiates the I2C */
void said(void);

// A simple animation on SAID at addr 2
void sidelookers_animate(void);

// This tests the difference between authenticated and test mode
void testmode(void);

/*Tests the SAID I2C by reading the data from the RDK4 I2C Slave */
void said_i2c_test( uint8_t addr);

void parallel (void);

void PWM(void);

#endif /* SAID_H_ */
