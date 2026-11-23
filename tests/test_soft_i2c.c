#include <soft_i2c.h>
#include <wiringPi.h>
#include <assert.h>
#include <stddef.h>
#include <stdlib.h>

enum { SCL = 9, SDA = 8 };
static const i2c_t port = { SCL, SDA };
static int setup_result;
static int modes[10];
static int clock_stuck;
static int data_stuck;
static unsigned int stretched_reads;
static unsigned int clock_reads;
static unsigned int pullups;
static int input_bits[512];
static size_t input_size, input_index;
static int clocked_data[2048];
static size_t clocked_count;

int wiringPiSetup(void) { return setup_result; }

void pinMode(int pin, int mode)
{
    assert(pin == SDA || pin == SCL);
    if (pin == SCL && modes[SCL] == OUTPUT && mode == INPUT) {
        assert(clocked_count < sizeof(clocked_data) / sizeof(clocked_data[0]));
        clocked_data[clocked_count++] = modes[SDA] == INPUT;
    }
    modes[pin] = mode;
}

void pullUpDnControl(int pin, int pull)
{
    assert((pin == SDA || pin == SCL) && pull == PUD_UP);
    ++pullups;
}

int digitalRead(int pin)
{
    assert(pin == SDA || pin == SCL);
    if (pin == SCL) {
        ++clock_reads;
        if (stretched_reads != 0) {
            --stretched_reads;
            return LOW;
        }
        return !clock_stuck && modes[SCL] == INPUT;
    }
    if (data_stuck || modes[SDA] == OUTPUT)
        return LOW;
    return input_index < input_size ? input_bits[input_index++] : 1;
}

void digitalWrite(int pin, int value)
{
    assert(pin == SDA || pin == SCL);
    assert(value == LOW);
}

void delayMicroseconds(unsigned int how_long) { assert(how_long > 0); }

static void enqueue_bit(int bit)
{
    assert(input_size < sizeof(input_bits) / sizeof(input_bits[0]));
    input_bits[input_size++] = bit;
}

static void reset_mock(void)
{
    setup_result = clock_stuck = data_stuck = 0;
    stretched_reads = clock_reads = pullups = 0;
    modes[SCL] = modes[SDA] = INPUT;
    input_size = input_index = clocked_count = 0;
}

static void setup_and_bus_recovery(void)
{
    reset_mock();
    setup_result = -1;
    assert(setupI2C() == 1 && i2c_status() == I2C_BUS_ERROR);
    assert(getBus()->scl == -1 && getBus()->sda == -1);
    assert(!i2c_test_slave(0x44));

    setup_result = 0;
    assert(setupI2C() == 0 && i2c_status() == I2C_OK);
    assert(getBus()->scl == SCL && getBus()->sda == SDA);
    assert(pullups == 2 && clocked_count == 10);
    assert(modes[SCL] == INPUT && modes[SDA] == INPUT);

    i2c_t invalid = i2c_init(SDA, SDA);
    assert(invalid.scl == invalid.sda && i2c_status() == I2C_BUS_ERROR);
    stretched_reads = 3;
    i2c_reset(port);
    assert(i2c_status() == I2C_OK && stretched_reads == 0);

    data_stuck = 1;
    size_t before = clocked_count;
    i2c_start(port);
    assert(i2c_status() == I2C_BUS_ERROR && clocked_count - before == 10);
    assert(modes[SCL] == INPUT && modes[SDA] == INPUT);
}

static void reset_stops_on_a_stuck_clock(void)
{
    reset_mock();
    clock_stuck = 1;

    i2c_reset(port);

    assert(i2c_status() == I2C_BUS_ERROR);
    assert(clocked_count == 1);
    assert(modes[SCL] == INPUT && modes[SDA] == INPUT);

    clock_stuck = 0;
    i2c_reset(port);
    assert(i2c_status() == I2C_OK && clocked_count == 11);
}

static void bits_and_bytes(void)
{
    reset_mock();
    i2c_start(port);
    size_t first_bit = clocked_count;
    enqueue_bit(I2C_ACK);
    assert(i2c_send_byte(port, 0xa5) == I2C_ACK);
    for (unsigned int bit = 0; bit < 8; ++bit)
        assert(clocked_data[first_bit + bit] == ((0xa5 >> (7 - bit)) & 1));

    for (unsigned int bit = 0; bit < 8; ++bit)
        enqueue_bit((0xe7 >> (7 - bit)) & 1);
    assert(i2c_read_byte(port) == 0xe7);

    i2c_send_bit(port, I2C_NACK);
    assert(clocked_data[clocked_count - 1] == 1);
    i2c_stop(port);
    assert(i2c_status() == I2C_OK);
    assert(_i2c_release(SDA) == 1);

    _i2c_pull(SDA);
    assert(modes[SDA] == OUTPUT);
    i2c_start(port);
    clock_stuck = 1;
    assert(i2c_read_bit(port) == I2C_BUS_ERROR);
    assert(i2c_send_byte(port, 0) == I2C_BUS_ERROR);
    assert(i2c_read_byte(port) == 0);
    assert(clock_reads < 300);
    i2c_stop(port);
    assert(modes[SCL] == INPUT && modes[SDA] == INPUT);
}

static void scan_inclusive_range(void)
{
    reset_mock();
    for (int address = 0x08; address <= 0x77; ++address) {
        enqueue_bit(1);
        enqueue_bit(address == 0x32 ? I2C_NACK : I2C_ACK);
        enqueue_bit(1);
    }
    vector_int found = i2c_search(0x08, 0x77);
    assert(found.capacity == 112 && found.size == 111);
    assert(found.container != NULL && found.container[0] == 0x08);
    assert(found.container[0x32 - 0x08] == 0x33);
    assert(found.container[found.size - 1] == 0x77);
    assert(input_index == input_size);
    free(found.container);

    reset_mock();
    assert(i2c_search(0x21, 0x20).container == NULL);
    assert(i2c_search(-1, 0x20).container == NULL);
    assert(i2c_search(0x20, 0x80).container == NULL);
    assert(!i2c_test_slave(0x07) && !i2c_test_slave(0x78));
    assert(clocked_count == 0);
    clock_stuck = 1;
    assert(i2c_search(0x20, 0x20).container == NULL);
}

int main(void)
{
    setup_and_bus_recovery();
    reset_stops_on_a_stuck_clock();
    bits_and_bytes();
    scan_inclusive_range();
    return 0;
}
