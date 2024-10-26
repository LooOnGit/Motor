#ifndef TM1637_H
#define TM1637_H

#include "stm32f1xx_hal.h"
#include "stdio.h"
#include "stdbool.h"

#define TM1637_I2C_COMM1    0x40
#define TM1637_I2C_COMM2    0xC0
#define TM1637_I2C_COMM3    0x80

extern const uint8_t digitToSegment[];

typedef struct {
    GPIO_TypeDef *clk_port;
    uint16_t clk_pin;
    GPIO_TypeDef *dio_port;
    uint16_t dio_pin;
    uint8_t brightness;
    uint32_t bit_delay;
} TM1637Display;

void TM1637_Init(TM1637Display *display, GPIO_TypeDef *clk_port, uint16_t clk_pin, GPIO_TypeDef *dio_port, uint16_t dio_pin, uint32_t bit_delay);
void TM1637_SetBrightness(TM1637Display *display, uint8_t brightness, uint8_t on);
void TM1637_SetSegments(TM1637Display *display, const uint8_t segments[], uint8_t length, uint8_t pos);
void TM1637_Clear(TM1637Display *display);
void TM1637_ShowNumberDec(TM1637Display *display, int num, bool leading_zero, uint8_t length, uint8_t pos);
void TM1637_ShowNumberHexEx(TM1637Display *display, uint16_t num, uint8_t dots, bool leading_zero, uint8_t length, uint8_t pos);

#endif // TM1637_H
