#ifndef TEST_WIRING_PI_H
#define TEST_WIRING_PI_H

enum { INPUT = 0, OUTPUT = 1, LOW = 0, PUD_UP = 2 };
int wiringPiSetup(void);
void pinMode(int pin, int mode);
void pullUpDnControl(int pin, int pull);
int digitalRead(int pin);
void digitalWrite(int pin, int value);
void delayMicroseconds(unsigned int how_long);

#endif
