/*
 * soft_i2c
 *      This is a basic software implementation of I2C protocol
 *      using WiringPi library GPIO.
 *
 *      Reinoso G. 09/02/2017
 */
#include<soft_i2c.h>
#ifdef __cplusplus
extern "C" {
#endif
static i2c_t i2cbus;

int setupI2C() {
	/* You have to select i2c Freq. in soft_12c.h #define I2C_FREQ 400000
		(I use 400K). The frecuency has to be the same as you selected in OS.
		Run raspi-config and activate i2c. Test it running $i2cdetect -y 1
		Edit /boot/config.txt and add at the end: dtparam=i2c_baudrate=400000
		see: https://projects.drogon.net/raspberry-pi/wiringpi/i2c-library/
	*/
	if (wiringPiSetup () == -1) return 1;
	i2cbus = i2c_init(9, 8);
	i2c_reset(i2cbus);
	usleep(5000);
	return 0;
}

i2c_t* getBus() {
	return &i2cbus;
}

/* Pull: drives the line to level LOW */
void _i2c_pull(int pin) {
	pinMode(pin, OUTPUT);
	digitalWrite(pin, LOW);
	delayMicroseconds((1e6/I2C_FREQ)/2);
}

/* Release: releases the line and return line status */
int _i2c_release(int pin) {
	pinMode(pin, INPUT);
	delayMicroseconds((1e6/I2C_FREQ)/2);
	return digitalRead(pin);
}

/* In case of clock stretching or busy bus we must wait */
void _i2c_release_wait(int pin) {
	pinMode(pin, INPUT);
	delayMicroseconds((1e6/I2C_FREQ)/2);
	while (!digitalRead(pin))
		delayMicroseconds(100);
	delayMicroseconds((1e6/I2C_FREQ)/2);
}

/* Initializes software emulated i2c */
i2c_t i2c_init(int scl, int sda) {
	i2c_t port;

	port.scl = scl;
	port.sda = sda;
	pinMode(scl, INPUT);
	pinMode(sda, INPUT);
	pullUpDnControl(scl, PUD_UP);
	pullUpDnControl(sda, PUD_UP);
	i2c_reset(port);
	return port;
}

/* Start: pull SDA while SCL is up*/
/* Best practice is to ensure the bus is not busy before start */
void i2c_start(i2c_t port) {
	if (!_i2c_release(port.sda))
		i2c_reset(port);
		_i2c_release_wait(port.scl);

	_i2c_pull(port.sda);
	_i2c_pull(port.scl);
}

/* Stop: release SDA while SCL is up */
void i2c_stop(i2c_t port) {
	_i2c_release_wait(port.scl);
	if (!_i2c_release(port.sda))
		i2c_reset(port);
}

/* Reset bus sequence */
void i2c_reset(i2c_t port) {
	int i;
	_i2c_release(port.sda);
	do {
		for (i = 0; i < 9; i++) {
			_i2c_pull(port.scl);
			_i2c_release(port.scl);
		}
	} while (!digitalRead(port.sda));
	_i2c_pull(port.scl);
	_i2c_pull(port.sda);
	i2c_stop(port);
}

/* Sends 0 or 1:
 * Clock down, send bit, clock up, wait, clock down again
 * In clock stretching, slave holds the clock line down in order
 * to force master to wait before send more data */
void i2c_send_bit(i2c_t port, int bit) {
	if (bit)
		_i2c_release(port.sda);
	else
		_i2c_pull(port.sda);

	_i2c_release_wait(port.scl);
	_i2c_pull(port.scl);

	_i2c_pull(port.sda);
}

/* Reads a bit from sda */
int i2c_read_bit(i2c_t port) {
	int s;

	_i2c_release(port.sda);
	_i2c_release(port.scl);
	s = digitalRead(port.sda);
	_i2c_pull(port.scl);
	_i2c_pull(port.sda);

	return s;
}

/* Sends 8 bit in a row, MSB first and reads ACK.
 * Returns I2C_ACK if device ack'ed */
int i2c_send_byte(i2c_t port, uint8_t byte) {
	int i;

	for (i = 0; i < 8; i++) {
		i2c_send_bit(port, byte & 0x80);
		byte = byte << 1;
	}

	return i2c_read_bit(port);
}

/* Reads a byte, MSB first */
uint8_t i2c_read_byte(i2c_t port) {
	int _byte = 0x00;
	int i;

	for (i=0; i<8; i++)
		_byte = (_byte << 1) | i2c_read_bit(port);

	return _byte;
}


bool i2c_test_slave(uint8_t address) {
  i2c_start(*getBus());
  int r = i2c_send_byte(*getBus(), address << 1 | I2C_WRITE);
  i2c_send_byte(*getBus(), 0x00);
  i2c_stop(*getBus());
  return r == 0;
}

vector_int i2c_search(int from, int to) {
	vector_int found = {1, 0, (int*)malloc(1 * sizeof(int))};

  for (; from <= to; from++) {
    if (i2c_test_slave(from)) {
      // add to addressVector
			if(found.size > found.capacity){
				realloc(found.container, sizeof(found.container) * 2);
				found.capacity = sizeof(found.container) * 2;
			}
			found.container[found.size] = from;
			found.size = found.size + 1;
    }
  }
  return found;
}


#ifdef __cplusplus
}
#endif