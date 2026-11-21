/*
 * soft_i2c
 *      This is a basic software implementation of I2C protocol
 *      using WiringPi library GPIO.
 *
 *      Reinoso G. 09/02/2017
 */

#ifndef SOFT_I2C_H
#define SOFT_I2C_H
#include <stdint.h>
#ifndef __cplusplus
#include <stdbool.h>
#endif
#ifdef __cplusplus
extern "C" {
#endif
#define I2C_READ   1
#define I2C_WRITE  0
#define I2C_ACK    0
#define I2C_NACK   1

#ifndef I2C_FREQ
#define I2C_FREQ 400000U
#endif
#define I2C_OK 0
#define I2C_BUS_ERROR (-1)

typedef unsigned char byte;

typedef struct {
	int scl;
	int sda;
} i2c_t;

typedef struct {
	int capacity;
	int size;
	int *container;
} vector_int;

int setupI2C(void);

i2c_t *getBus(void);

/* Check after start, read, or stop. A new start or reset clears the error. */
int i2c_status(void);

/* Pull: drives the line to level LOW */
void _i2c_pull(int pin);

/* Release: releases the line */
int _i2c_release(int pin);

/* Release_wait: releases the line and wait until it goes up*/
void _i2c_release_wait(int pin);

/* Initializes software emulated i2c */
i2c_t i2c_init(int scl, int sda);

/* Start: pull SDA while SCL is up*/
/* Best practice is test the bus before start */
void i2c_start(i2c_t port);

/* Stop: release SDA while SCL is up */
void i2c_stop(i2c_t port);

/* Reset: software reset bus sequence */
void i2c_reset(i2c_t port);

/* Sends 0 or 1:
 * Clock down, send bit, clock up, wait, clock down again */
void i2c_send_bit(i2c_t port, int bit);

/* Reads a bit from sda */
int i2c_read_bit(i2c_t port);

/* Sends 8 bit in a row, MSB first and reads ACK.
 * Returns I2C_ACK, I2C_NACK, or I2C_BUS_ERROR */
int i2c_send_byte(i2c_t port, uint8_t byte);

/* Reads a byte, MSB first */
uint8_t i2c_read_byte(i2c_t port);

bool i2c_test_slave(uint8_t address);

/* Scan inclusive 7-bit device addresses. Caller frees container. */
/* On invalid input, bus failure or out of memory, returns an empty vector. */
vector_int i2c_search(int from, int to);

#ifdef __cplusplus
}
#endif
#endif
