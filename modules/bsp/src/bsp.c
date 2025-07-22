/************************************************************************************************
Copyright (c) 2022-2024, Laboratorio de Microprocesadores
Facultad de Ciencias Exactas y Tecnología, Universidad Nacional de Tucumán
https://www.microprocesadores.unt.edu.ar/

Copyright (c) 2022-2024, Esteban Volentini <evolentini@herrera.unt.edu.ar>

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES
OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

SPDX-License-Identifier: MIT
*************************************************************************************************/

/** \brief Longan NANO board configuration implementation
 **
 ** \addtogroup board Board support
 ** \brief Board agnostic configuration module
 ** @{ */

/* === Headers files inclusions =============================================================== */

#include "bsp.h"
#include "pantalla.h"

/* === Macros definitions ====================================================================== */

// Definiciones de los recursos asociados a los DIGITs de la pantalla
#define DIGIT_1_BIT    8
#define DIGIT_1_MASK   (1 << DIGIT_1_BIT)

#define DIGIT_2_BIT    9
#define DIGIT_2_MASK   (1 << DIGIT_2_BIT)

#define DIGIT_3_BIT    10
#define DIGIT_3_MASK   (1 << DIGIT_3_BIT)

#define DIGIT_4_BIT    11
#define DIGIT_4_MASK   (1 << DIGIT_4_BIT)

#define DIGITS_GPIO    GPIOC
#define DIGITS_CLK     RCU_GPIOC
#define DIGITS_MASK    (DIGIT_1_MASK | DIGIT_2_MASK | DIGIT_3_MASK | DIGIT_4_MASK)

// Definiciones de los recursos asociados a los SEGMENTs de la pantalla

#define SEGMENT_A_BIT  0
#define SEGMENT_A_MASK (1 << SEGMENT_A_BIT)

#define SEGMENT_B_BIT  1
#define SEGMENT_B_MASK (1 << SEGMENT_B_BIT)

#define SEGMENT_C_BIT  2
#define SEGMENT_C_MASK (1 << SEGMENT_C_BIT)

#define SEGMENT_D_BIT  3
#define SEGMENT_D_MASK (1 << SEGMENT_D_BIT)

#define SEGMENT_E_BIT  4
#define SEGMENT_E_MASK (1 << SEGMENT_E_BIT)

#define SEGMENT_F_BIT  5
#define SEGMENT_F_MASK (1 << SEGMENT_F_BIT)

#define SEGMENT_G_BIT  6
#define SEGMENT_G_MASK (1 << SEGMENT_G_BIT)

#define SEGMENTS_GPIO  GPIOC
#define SEGMENTS_CLK   RCU_GPIOC
#define SEGMENTS_MASK                                                                                                  \
    (SEGMENT_A_MASK | SEGMENT_B_MASK | SEGMENT_C_MASK | SEGMENT_D_MASK | SEGMENT_E_MASK | SEGMENT_F_MASK |             \
     SEGMENT_G_MASK)

#define SEGMENT_DOT_BIT  13
#define SEGMENT_DOT_MASK (1 << SEGMENT_DOT_BIT)
#define SEGMENT_DOT_GPIO GPIOC
#define SEGMENT_DOT_CLK  RCU_GPIOC

/* --- GPIO Direct Functions Keyboard ----------------------------------------------------------- */

#define KEY_ENC_BIT      5
#define KEY_ENC_MASK     (1 << KEY_ENC_BIT)

#define KEY_ACEPT_BIT    10
#define KEY_ACEPT_MASK   (1 << KEY_ACEPT_BIT)

#define KEY_CANCEL_BIT   11
#define KEY_CANCEL_MASK  (1 << KEY_CANCEL_BIT)

#define KEY_F1_BIT       12
#define KEY_F1_MASK      (1 << KEY_F1_BIT)

#define KEY_F2_BIT       13
#define KEY_F2_MASK      (1 << KEY_F2_BIT)

#define KEY_F3_BIT       14
#define KEY_F3_MASK      (1 << KEY_F3_BIT)

#define KEY_F4_BIT       15
#define KEY_F4_MASK      (1 << KEY_F4_BIT)

#define KEYS_GPIO        GPIOB
#define KEYS_CLK         RCU_GPIOB
#define KEYS_MASK                                                                                                      \
    (KEY_ENC_MASK | KEY_ACEPT_MASK | KEY_CANCEL_MASK | KEY_F1_MASK | KEY_F2_MASK | KEY_F3_MASK | KEY_F4_MASK)

/* --- Matrix Numbers Keyboard ----------------------------------------------------------------- */

#define NUMS_R1_BIT    9
#define NUMS_R1_MASK   (1 << NUMS_R1_BIT)

#define NUMS_R2_BIT    10
#define NUMS_R2_MASK   (1 << NUMS_R2_BIT)

#define NUMS_R3_BIT    11
#define NUMS_R3_MASK   (1 << NUMS_R3_BIT)

#define NUMS_R4_BIT    12
#define NUMS_R4_MASK   (1 << NUMS_R4_BIT)

#define NUMS_ROWS_GPIO GPIOA
#define NUMS_ROWS_CLK  RCU_GPIOA
#define NUMS_ROWS_MASK (NUMS_R1_MASK | NUMS_R2_MASK | NUMS_R3_MASK | NUMS_R4_MASK)

#define NUMS_C1_BIT    DIGIT_1_BIT
#define NUMS_C1_MASK   DIGIT_1_MASK

#define NUMS_C2_BIT    DIGIT_2_BIT
#define NUMS_C2_MASK   DIGIT_2_MASK

#define NUMS_C3_BIT    DIGIT_3_BIT
#define NUMS_C3_MASK   DIGIT_3_MASK

#define NUMS_C4_BIT    DIGIT_4_BIT
#define NUMS_C4_MASK   DIGIT_4_MASK

#define NUMS_COLS_GPIO DIGITS_GPIO
#define NUMS_COLS_MASK (NUMS_C1_MASK | NUMS_C2_MASK | NUMS_C3_MASK | NUMS_C4_MASK)

/* === Private data type declarations ========================================================== */

/* === Private variable declarations =========================================================== */

static const char NumericKeboardMap[] = {
    '1', '4', '7', '*', '2', '5', '8', '0', '3', '6', '9', '#', 'A', 'B', 'C', 'D',
};

/* === Private function declarations =========================================================== */

static void DigitsInit(void);

static void SegmentsInit(void);

static void ScreenTurnOff(void);

static void SegmentsTurnOn(uint8_t segments);

static void DigitTurnOn(uint8_t digit);

static uint16_t NumericKeyboardScan(uint8_t column, uint16_t previous_state);

static void FunctionKeyboardInit(void);

static uint32_t FunctionKeyboardScan(uint8_t key_count);

static void NumericKeyboardInit(void);

/* === Public variable definitions ============================================================= */

/* === Private variable definitions ============================================================ */

static uint8_t last_digit = 0;

static uint16_t current_state = 0;

static struct board_s self[1];

/* === Private function implementation ========================================================= */

static void DigitsInit(void) {
    /* enable the led clock */
    rcu_periph_clock_enable(DIGITS_CLK);

    /* configure led GPIO port */
    GPIO_BC(DIGITS_GPIO) = DIGITS_MASK;
    gpio_init(DIGITS_GPIO, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, DIGITS_MASK);
}

static void SegmentsInit(void) {
    /* enable the led clock */
    rcu_periph_clock_enable(SEGMENTS_CLK);
    rcu_periph_clock_enable(SEGMENT_DOT_CLK);

    /* configure led GPIO port */
    GPIO_BC(SEGMENTS_GPIO) = SEGMENTS_MASK;
    GPIO_BC(SEGMENT_DOT_GPIO) = SEGMENT_DOT_MASK;
    gpio_init(SEGMENTS_GPIO, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, SEGMENTS_MASK);
    gpio_init(SEGMENT_DOT_GPIO, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, SEGMENT_DOT_MASK);
}

static void ScreenTurnOff(void) {
    GPIO_BC(SEGMENTS_GPIO) = SEGMENTS_MASK;
    GPIO_BC(SEGMENT_DOT_GPIO) = SEGMENT_DOT_MASK;

    if (last_digit == 0) {
        KeyboardUpdate(self->numeric, current_state);
    }
    current_state = NumericKeyboardScan(last_digit, current_state);

    GPIO_BC(DIGITS_GPIO) = DIGITS_MASK;
}

static void SegmentsTurnOn(uint8_t segments) {
    GPIO_BC(SEGMENTS_GPIO) = SEGMENTS_MASK;
    GPIO_BC(SEGMENT_DOT_GPIO) = SEGMENT_DOT_MASK;

    GPIO_BOP(SEGMENTS_GPIO) = (segments << SEGMENT_A_BIT) & SEGMENTS_MASK;
    if (segments & SEGMENT_P) {
        GPIO_BOP(SEGMENT_DOT_GPIO) = SEGMENT_DOT_MASK;
    }
}

static void DigitTurnOn(uint8_t digit) {
    last_digit = digit;

    GPIO_BC(DIGITS_GPIO) = DIGITS_MASK;
    GPIO_BOP(DIGITS_GPIO) = ((1 << digit) << DIGIT_1_BIT) & DIGITS_MASK;
}

static void NumericKeyboardInit(void) {
    rcu_periph_clock_enable(NUMS_ROWS_CLK);
    gpio_init(NUMS_ROWS_GPIO, GPIO_MODE_IPD, GPIO_OSPEED_50MHZ, NUMS_ROWS_MASK);
}

static uint16_t NumericKeyboardScan(uint8_t column, uint16_t previous_state) {
    uint16_t result = previous_state;
    uint8_t value;

    value = (GPIO_ISTAT(NUMS_ROWS_GPIO) & NUMS_ROWS_MASK) >> NUMS_R1_BIT;

    result &= ~(0x000F << (column * 4));
    result |= (value << (column * 4));

    return result;
}

static void FunctionKeyboardInit(void) {
    rcu_periph_clock_enable(KEYS_CLK);
    gpio_init(KEYS_GPIO, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, KEYS_MASK);
}

static uint32_t FunctionKeyboardScan(uint8_t key_count) {
    uint32_t result = 0;

    result = (GPIO_ISTAT(KEYS_GPIO) & KEYS_MASK);
    result = (result >> KEY_ACEPT_BIT) | (((~result) & KEY_ENC_MASK) << 1);
    return result;
}

static void ConsoleInit(void) {
    /* enable GPIO clock */
    rcu_periph_clock_enable(RCU_GPIOC);
    rcu_periph_clock_enable(RCU_GPIOD);

    /* enable USART clock */
    rcu_periph_clock_enable(RCU_UART4);

    /* connect port to USARTx_Tx */
    gpio_init(GPIOC, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_12);

    /* connect port to USARTx_Rx */
    gpio_init(GPIOD, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_2);

    usart_deinit(UART4);
    usart_baudrate_set(UART4, 115200U);
    usart_word_length_set(UART4, USART_WL_8BIT);
    usart_stop_bit_set(UART4, USART_STB_1BIT);
    usart_parity_config(UART4, USART_PM_NONE);
    usart_receive_config(UART4, USART_RECEIVE_ENABLE);
    usart_transmit_config(UART4, USART_TRANSMIT_ENABLE);
    usart_enable(UART4);
}

/* === Public function implementation ========================================================== */

board_t BoardCreate(void) {
    BoardSetup();
    ConsoleInit();
    SonidoInit();
    DigitsInit();
    SegmentsInit();
    LedRgbInit(true);
    FunctionKeyboardInit();
    NumericKeyboardInit();
    AnalogInit();
    EncoderInit();

    self->display = DisplayCreate(4, &(struct display_driver_s){
                                         .ScreenTurnOff = ScreenTurnOff,
                                         .SegmentsTurnOn = SegmentsTurnOn,
                                         .DigitTurnOn = DigitTurnOn,
                                     });
    self->functions = KeyboardCreate(6, FunctionKeyboardScan);
    self->numeric = KeyboardCreate(16, NULL);

    return self;
}

char TranslateNumericKey(uint16_t value) {
    return (value < sizeof(NumericKeboardMap)) ? NumericKeboardMap[value] : 0;
}

/* retarget the C library printf function to the USART */
int _put_char(int value) {
    usart_data_transmit(UART4, (uint8_t)value);
    while (usart_flag_get(UART4, USART_FLAG_TBE) == RESET);

    return value;
}

/* === End of documentation ==================================================================== */

/** @} End of module definition for doxygen */
