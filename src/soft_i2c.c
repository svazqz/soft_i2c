/*
 * Software I2C over WiringPi GPIO.
 * Original implementation by Reinoso G., 09/02/2017.
 */
#include <soft_i2c.h>
#include <stdlib.h>
#include <wiringPi.h>

enum { CLOCK_POLL_US = 100, CLOCK_POLLS = 250 };
#define HALF_PERIOD_US ((500000U + I2C_FREQ - 1U) / I2C_FREQ)

static i2c_t i2cbus = { -1, -1 };
static int bus_status;

int setupI2C(void)
{
    i2cbus = (i2c_t){ -1, -1 };
    if (wiringPiSetup() == -1) {
        bus_status = I2C_BUS_ERROR;
        return 1;
    }
    i2cbus = i2c_init(9, 8);
    delayMicroseconds(5000);
    return bus_status != I2C_OK;
}

i2c_t *getBus(void)
{
    return &i2cbus;
}

int i2c_status(void)
{
    return bus_status;
}

static bool valid_port(i2c_t port)
{
    if (port.scl >= 0 && port.sda >= 0 && port.scl != port.sda)
        return true;
    bus_status = I2C_BUS_ERROR;
    return false;
}

void _i2c_pull(int pin)
{
    digitalWrite(pin, LOW);
    pinMode(pin, OUTPUT);
    delayMicroseconds(HALF_PERIOD_US);
}

int _i2c_release(int pin)
{
    pinMode(pin, INPUT);
    delayMicroseconds(HALF_PERIOD_US);
    return digitalRead(pin);
}

void _i2c_release_wait(int pin)
{
    pinMode(pin, INPUT);
    delayMicroseconds(HALF_PERIOD_US);
    for (unsigned int attempt = 0; attempt < CLOCK_POLLS; ++attempt) {
        if (digitalRead(pin)) {
            delayMicroseconds(HALF_PERIOD_US);
            return;
        }
        delayMicroseconds(CLOCK_POLL_US);
    }
    bus_status = I2C_BUS_ERROR;
}

i2c_t i2c_init(int scl, int sda)
{
    i2c_t port = { scl, sda };
    if (!valid_port(port))
        return port;

    pinMode(scl, INPUT);
    pinMode(sda, INPUT);
    pullUpDnControl(scl, PUD_UP);
    pullUpDnControl(sda, PUD_UP);
    i2c_reset(port);
    return port;
}

void i2c_start(i2c_t port)
{
    bus_status = I2C_OK;
    if (!valid_port(port))
        return;

    if (!_i2c_release(port.sda))
        i2c_reset(port);
    if (bus_status != I2C_OK)
        return;

    _i2c_release_wait(port.scl);
    if (bus_status == I2C_OK) {
        _i2c_pull(port.sda);
        _i2c_pull(port.scl);
    }
}

void i2c_stop(i2c_t port)
{
    if (!valid_port(port))
        return;
    if (bus_status == I2C_OK) {
        _i2c_pull(port.scl);
        _i2c_pull(port.sda);
        _i2c_release_wait(port.scl);
    }
    if (!_i2c_release(port.sda))
        bus_status = I2C_BUS_ERROR;
    pinMode(port.scl, INPUT);
}

void i2c_reset(i2c_t port)
{
    bus_status = I2C_OK;
    if (!valid_port(port))
        return;

    pinMode(port.sda, INPUT);
    for (unsigned int pulse = 0; pulse < 9; ++pulse) {
        _i2c_pull(port.scl);
        _i2c_release_wait(port.scl);
        if (bus_status != I2C_OK)
            break;
    }
    i2c_stop(port);
}

void i2c_send_bit(i2c_t port, int bit)
{
    if (bus_status != I2C_OK || !valid_port(port))
        return;

    if (bit) {
        pinMode(port.sda, INPUT);
        delayMicroseconds(HALF_PERIOD_US);
    } else {
        _i2c_pull(port.sda);
    }
    _i2c_release_wait(port.scl);
    _i2c_pull(port.scl);
    _i2c_pull(port.sda);
}

int i2c_read_bit(i2c_t port)
{
    if (bus_status != I2C_OK || !valid_port(port))
        return I2C_BUS_ERROR;

    pinMode(port.sda, INPUT);
    delayMicroseconds(HALF_PERIOD_US);
    _i2c_release_wait(port.scl);
    int bit = bus_status == I2C_OK ? !!digitalRead(port.sda) : I2C_BUS_ERROR;
    _i2c_pull(port.scl);
    _i2c_pull(port.sda);
    return bit;
}

int i2c_send_byte(i2c_t port, uint8_t value)
{
    for (uint8_t mask = 0x80; mask != 0 && bus_status == I2C_OK; mask >>= 1)
        i2c_send_bit(port, value & mask);

    return i2c_read_bit(port);
}

uint8_t i2c_read_byte(i2c_t port)
{
    uint8_t value = 0;
    for (unsigned int index = 0; index < 8; ++index) {
        int bit = i2c_read_bit(port);
        if (bit == I2C_BUS_ERROR)
            return 0;
        value = (uint8_t)((value << 1) | bit);
    }
    return value;
}

bool i2c_test_slave(uint8_t address)
{
    if (address < 0x08 || address > 0x77)
        return false;

    i2c_t port = *getBus();
    i2c_start(port);
    int reply = i2c_send_byte(port, (uint8_t)((address << 1) | I2C_WRITE));
    i2c_stop(port);
    return reply == I2C_ACK && bus_status == I2C_OK;
}

vector_int i2c_search(int from, int to)
{
    if (from < 0x08 || to > 0x77 || from > to)
        return (vector_int){ 0, 0, NULL };

    vector_int found = { to - from + 1, 0, NULL };
    found.container = malloc((size_t)found.capacity * sizeof(*found.container));
    if (found.container == NULL)
        return (vector_int){ 0, 0, NULL };

    for (int address = from; address <= to; ++address) {
        if (i2c_test_slave((uint8_t)address))
            found.container[found.size++] = address;
        if (bus_status != I2C_OK) {
            free(found.container);
            return (vector_int){ 0, 0, NULL };
        }
    }
    return found;
}
