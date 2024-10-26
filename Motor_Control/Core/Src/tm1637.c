#include "tm1637.h"


const uint8_t digitToSegment[] = {
    0b00111111, 0b00000110, 0b01011011, 0b01001111, 0b01100110,
    0b01101101, 0b01111101, 0b00000111, 0b01111111, 0b01101111,
    0b01110111, 0b01111100, 0b00111001, 0b01011110, 0b01111001,
    0b01110001
};

void TM1637_Start(TM1637Display *display);
uint8_t TM1637_WriteByte(TM1637Display *display, uint8_t data);
void TM1637_Stop(TM1637Display *display);
uint8_t TM1637_WriteByte(TM1637Display *display, uint8_t data);
void TM1637_ShowNumberDecEx(TM1637Display *display, int num, uint8_t dots, bool leading_zero, uint8_t length, uint8_t pos);
void TM1637_ShowNumberBaseEx(TM1637Display *display, int8_t base, uint16_t num, uint8_t dots, bool leading_zero, uint8_t length, uint8_t pos);
void TM1637_ShowDots(uint8_t dots, uint8_t* digits);
uint8_t TM1637_EncodeDigit(uint8_t digit);

static const uint8_t minusSegments = 0b01000000;

void TM1637_Init(TM1637Display *display, GPIO_TypeDef *clk_port, uint16_t clk_pin, GPIO_TypeDef *dio_port, uint16_t dio_pin, uint32_t bit_delay) {
    display->clk_port = clk_port;
    display->clk_pin = clk_pin;
    display->dio_port = dio_port;
    display->dio_pin = dio_pin;
    display->bit_delay = bit_delay;

    HAL_GPIO_WritePin(display->clk_port, display->clk_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(display->dio_port, display->dio_pin, GPIO_PIN_RESET);
}

void TM1637_SetBrightness(TM1637Display *display, uint8_t brightness, uint8_t on) {
    display->brightness = (brightness & 0x7) | (on ? 0x08 : 0x00);
}

void TM1637_SetSegments(TM1637Display *display, const uint8_t segments[], uint8_t length, uint8_t pos) {

	// Write COMM1
    TM1637_Start(display);
    TM1637_WriteByte(display, TM1637_I2C_COMM1);
    TM1637_Stop(display);

    // Write COMM2 + first digit address
    TM1637_Start(display);
    TM1637_WriteByte(display, TM1637_I2C_COMM2 + (pos & 0x03));

    // Write the data bytes
    for (uint8_t k = 0; k < length; k++) {
        TM1637_WriteByte(display, segments[k]);
    }

    TM1637_Stop(display);

    // Write COMM3 + brightness
    TM1637_Start(display);
    TM1637_WriteByte(display, TM1637_I2C_COMM3 + (display->brightness & 0x0f));
    TM1637_Stop(display);
}

void TM1637_Clear(TM1637Display *display) {
    uint8_t data[] = { 0, 0, 0, 0 };
    TM1637_SetSegments(display, data, 4, 0);
}

void TM1637_ShowNumberDec(TM1637Display *display, int num, bool leading_zero, uint8_t length, uint8_t pos) {
    TM1637_ShowNumberDecEx(display, num, 0, leading_zero, length, pos);
}

void TM1637_ShowNumberDecEx(TM1637Display *display, int num, uint8_t dots, bool leading_zero, uint8_t length, uint8_t pos) {
    TM1637_ShowNumberBaseEx(display, (num < 0) ? -10 : 10, (num < 0) ? -num : num, dots, leading_zero, length, pos);
}

void TM1637_ShowNumberHexEx(TM1637Display *display, uint16_t num, uint8_t dots, bool leading_zero, uint8_t length, uint8_t pos) {
    TM1637_ShowNumberBaseEx(display, 16, num, dots, leading_zero, length, pos);
}

void TM1637_ShowNumberBaseEx(TM1637Display *display, int8_t base, uint16_t num, uint8_t dots, bool leading_zero, uint8_t length, uint8_t pos) {
    bool negative = false;
    if (base < 0) {
        base = -base;
        negative = true;
    }

    uint8_t digits[4] = {0};

    if (num == 0 && !leading_zero) {
        // Singular case - take care separately
        for (uint8_t i = 0; i < (length - 1); i++)
            digits[i] = 0;
        digits[length - 1] = TM1637_EncodeDigit(0);
    } else {
        for (int i = length - 1; i >= 0; --i) {
            uint8_t digit = num % base;

            if (digit == 0 && num == 0 && !leading_zero)
                // Leading zero is blank
                digits[i] = 0;
            else
                digits[i] = TM1637_EncodeDigit(digit);

            if (digit == 0 && num == 0 && negative) {
                digits[i] = minusSegments;
                negative = false;
            }

            num /= base;
        }
    }

    if (dots != 0) {
        TM1637_ShowDots(dots, digits);
    }

    TM1637_SetSegments(display, digits, length, pos);
}

void TM1637_Start(TM1637Display *display) {
    HAL_GPIO_WritePin(display->dio_port, display->dio_pin, GPIO_PIN_SET);
    HAL_Delay(display->bit_delay);
    HAL_GPIO_WritePin(display->clk_port, display->clk_pin, GPIO_PIN_SET);
    HAL_Delay(display->bit_delay);
    HAL_GPIO_WritePin(display->dio_port, display->dio_pin, GPIO_PIN_RESET);
    HAL_Delay(display->bit_delay);
}

void TM1637_Stop(TM1637Display *display) {
    HAL_GPIO_WritePin(display->clk_port, display->clk_pin, GPIO_PIN_RESET);
    HAL_Delay(display->bit_delay);
    HAL_GPIO_WritePin(display->dio_port, display->dio_pin, GPIO_PIN_RESET);
    HAL_Delay(display->bit_delay);
    HAL_GPIO_WritePin(display->clk_port, display->clk_pin, GPIO_PIN_SET);
    HAL_Delay(display->bit_delay);
    HAL_GPIO_WritePin(display->dio_port, display->dio_pin, GPIO_PIN_SET);
}

uint8_t TM1637_WriteByte(TM1637Display *display, uint8_t data) {

	 // 8 Data Bits
    for (uint8_t i = 0; i < 8; i++) {

        // CLK low
        HAL_GPIO_WritePin(display->clk_port, display->clk_pin, GPIO_PIN_RESET);
        HAL_Delay(display->bit_delay);

        // Set data bit
        if (data & 0x01) {
            HAL_GPIO_WritePin(display->dio_port, display->dio_pin, GPIO_PIN_SET);
        } else {
            HAL_GPIO_WritePin(display->dio_port, display->dio_pin, GPIO_PIN_RESET);
        }
        data >>= 1;
        // CLK high
        HAL_GPIO_WritePin(display->clk_port, display->clk_pin, GPIO_PIN_SET);
        HAL_Delay(display->bit_delay);
    }

    HAL_GPIO_WritePin(display->clk_port, display->clk_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(display->dio_port, display->dio_pin, GPIO_PIN_SET);
    HAL_Delay(display->bit_delay);

    HAL_GPIO_WritePin(display->clk_port, display->clk_pin, GPIO_PIN_SET);
    HAL_Delay(display->bit_delay);

    uint8_t ack = HAL_GPIO_ReadPin(display->dio_port, display->dio_pin);
    HAL_GPIO_WritePin(display->clk_port, display->clk_pin, GPIO_PIN_RESET);

    return ack;
}

void TM1637_ShowDots(uint8_t dots, uint8_t* digits) {
    for (int i = 0; i < 4; ++i) {
        digits[i] |= (dots & 0x80);
        dots <<= 1;
    }
}

uint8_t TM1637_EncodeDigit(uint8_t digit) {
    return digitToSegment[digit & 0x0F];
}

