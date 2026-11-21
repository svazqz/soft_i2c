# soft_i2c

A software I²C controller for Raspberry Pi GPIO using WiringPi. The default bus uses WiringPi pins 9 (SCL) and 8 (SDA). Call setupI2C before using the default bus and serialize access to the GPIO and I²C transaction state.

- Install the WiringPi development headers, then run make to build build/libsoft_i2c.a.
- Run make test to exercise the driver with simulated GPIO on a development machine.
- Link applications with libsoft_i2c.a and -lwiringPi. Set WIRINGPI_CFLAGS if the headers are outside the default include paths.

i2c_status reports a bus error after an operation, including a stuck clock or data line. i2c_send_byte distinguishes ACK, NACK, and bus error. A new start or reset clears the status. The scan accepts an inclusive address range from 0x08 through 0x77. Free the returned vector's container after use.
