/************************************************************************************************
Copyright (c) 2022-2023, Laboratorio de Microprocesadores
Facultad de Ciencias Exactas y Tecnología, Universidad Nacional de Tucumán
https://www.microprocesadores.unt.edu.ar/

Copyright (c) 2022-2023, Esteban Volentini <evolentini@herrera.unt.edu.ar>

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

/** \brief Hello World sample application
 **
 ** \addtogroup samples Samples
 ** \brief Samples applications with MUJU Framwork
 ** @{ */

/* === Headers files inclusions =============================================================== */

#include "board.h"
#include "rgb.h"
#include "teclas.h"
#include "pantalla.h"
#include "encoder.h"
#include "sonido.h"
#include <stddef.h>

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
#define KEYS_MASK        (KEY_ACEPT_MASK | KEY_CANCEL_MASK | KEY_F1_MASK | KEY_F2_MASK | KEY_F3_MASK | KEY_F4_MASK)

/* --- Matrix Numbers Keyboard ----------------------------------------------------------------- */

#define NUMS_R1_BIT      9
#define NUMS_R1_MASK     (1 << NUMS_R1_BIT)

#define NUMS_R2_BIT      10
#define NUMS_R2_MASK     (1 << NUMS_R2_BIT)

#define NUMS_R3_BIT      11
#define NUMS_R3_MASK     (1 << NUMS_R3_BIT)

#define NUMS_R4_BIT      12
#define NUMS_R4_MASK     (1 << NUMS_R4_BIT)

#define NUMS_ROWS_GPIO   GPIOA
#define NUMS_ROWS_CLK    RCU_GPIOA
#define NUMS_ROWS_MASK   (NUMS_R1_MASK | NUMS_R2_MASK | NUMS_R3_MASK | NUMS_R4_MASK)

#define NUMS_C1_BIT      DIGIT_1_BIT
#define NUMS_C1_MASK     DIGIT_1_MASK

#define NUMS_C2_BIT      DIGIT_2_BIT
#define NUMS_C2_MASK     DIGIT_2_MASK

#define NUMS_C3_BIT      DIGIT_3_BIT
#define NUMS_C3_MASK     DIGIT_3_MASK

#define NUMS_C4_BIT      DIGIT_4_BIT
#define NUMS_C4_MASK     DIGIT_4_MASK

#define NUMS_COLS_GPIO   DIGITS_GPIO
#define NUMS_COLS_MASK   (NUMS_C1_MASK | NUMS_C2_MASK | NUMS_C3_MASK | NUMS_C4_MASK)

#define FILTER_LENGTH    16

/* === Private data type declarations ========================================================== */

/* === Private variable declarations =========================================================== */

/* === Private function declarations =========================================================== */

/*!
    \brief      delay a time in milliseconds
    \param[in]  count: count in milliseconds
    \param[out] none
    \retval     none
*/
void delay_1ms(uint32_t count);

/* === Public variable definitions ============================================================= */

/* === Private variable definitions ============================================================ */

static keyboard_t numeric = NULL;

static uint8_t last_digit = 0;

static uint16_t current_state = 0;

/* === Private function implementation ========================================================= */

void delay_1ms(uint32_t count) {
    volatile uint64_t start_mtime, delta_mtime;

    volatile uint64_t tmp = SysTimer_GetLoadValue();
    do {
        start_mtime = SysTimer_GetLoadValue();
    } while (start_mtime == tmp);

    uint64_t delay_ticks = SystemCoreClock / 4; // 1 second
    delay_ticks = delay_ticks * count / 1000;

    do {
        delta_mtime = SysTimer_GetLoadValue() - start_mtime;
    } while (delta_mtime < delay_ticks);
}

uint16_t NumericKeyboardScan(uint8_t column, uint16_t previous_state) {
    uint16_t result = previous_state;
    uint8_t value;

    // gpio_init(NUMS_ROWS_GPIO, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, NUMS_ROWS_MASK);
    // gpio_init(NUMS_COLS_GPIO, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, NUMS_COLS_MASK);

    // switch (column) {
    // case 1:
    //     gpio_init(NUMS_COLS_GPIO, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, NUMS_C2_MASK);
    //     GPIO_BC(NUMS_COLS_GPIO) = NUMS_C2_MASK;
    //     break;
    // case 2:
    //     gpio_init(NUMS_COLS_GPIO, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, NUMS_C3_MASK);
    //     GPIO_BC(NUMS_COLS_GPIO) = NUMS_C3_MASK;
    //     break;
    // case 3:
    //     gpio_init(NUMS_COLS_GPIO, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, NUMS_C4_MASK);
    //     GPIO_BC(NUMS_COLS_GPIO) = NUMS_C4_MASK;
    //     break;
    // default:
    //     gpio_init(NUMS_COLS_GPIO, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, NUMS_C1_MASK);
    //     GPIO_BC(NUMS_COLS_GPIO) = NUMS_C1_MASK;
    //     break;
    // }
    // // delay_1ms(1);
    // for (int delay = 0; delay < 1000; delay++) {
    //     __asm__("nop");
    // }

    // value = (~GPIO_ISTAT(NUMS_ROWS_GPIO) & NUMS_ROWS_MASK) >> NUMS_R1_BIT;

    value = (GPIO_ISTAT(NUMS_ROWS_GPIO) & NUMS_ROWS_MASK) >> NUMS_R1_BIT;

    result &= ~(0x000F << (column * 4));
    result |= (value << (column * 4));
    // KeyboardUpdate(numeric, result);

    // gpio_init(NUMS_ROWS_GPIO, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, NUMS_ROWS_MASK);
    // gpio_init(NUMS_COLS_GPIO, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, NUMS_COLS_MASK);

    return result;
}

void DigitsInit(void) {
    /* enable the led clock */
    rcu_periph_clock_enable(DIGITS_CLK);

    /* configure led GPIO port */
    GPIO_BC(DIGITS_GPIO) = DIGITS_MASK;
    gpio_init(DIGITS_GPIO, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, DIGITS_MASK);
}

void SegmentsInit(void) {
    /* enable the led clock */
    rcu_periph_clock_enable(SEGMENTS_CLK);
    rcu_periph_clock_enable(SEGMENT_DOT_CLK);

    /* configure led GPIO port */
    GPIO_BC(SEGMENTS_GPIO) = SEGMENTS_MASK;
    GPIO_BC(SEGMENT_DOT_GPIO) = SEGMENT_DOT_MASK;
    gpio_init(SEGMENTS_GPIO, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, SEGMENTS_MASK);
    gpio_init(SEGMENT_DOT_GPIO, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, SEGMENT_DOT_MASK);
}

void ScreenTurnOff(void) {
    GPIO_BC(SEGMENTS_GPIO) = SEGMENTS_MASK;
    GPIO_BC(SEGMENT_DOT_GPIO) = SEGMENT_DOT_MASK;

    if (last_digit == 0) {
        KeyboardUpdate(numeric, current_state);
    }
    current_state = NumericKeyboardScan(last_digit, current_state);

    GPIO_BC(DIGITS_GPIO) = DIGITS_MASK;
}

void SegmentsTurnOn(uint8_t segments) {
    GPIO_BC(SEGMENTS_GPIO) = SEGMENTS_MASK;
    GPIO_BC(SEGMENT_DOT_GPIO) = SEGMENT_DOT_MASK;

    GPIO_BOP(SEGMENTS_GPIO) = (segments << SEGMENT_A_BIT) & SEGMENTS_MASK;
    if (segments & SEGMENT_P) {
        GPIO_BOP(SEGMENT_DOT_GPIO) = SEGMENT_DOT_MASK;
    }
}

void DigitTurnOn(uint8_t digit) {
    last_digit = digit;

    GPIO_BC(DIGITS_GPIO) = DIGITS_MASK;
    GPIO_BOP(DIGITS_GPIO) = ((1 << digit) << DIGIT_1_BIT) & DIGITS_MASK;
}

uint32_t FunctionKeyboardScan(uint8_t key_count) {
    uint32_t result = 0;

    result = ((GPIO_ISTAT(KEYS_GPIO) & KEYS_MASK) >> KEY_ACEPT_BIT);
    return result;
}

void FunctionKeyboardInit(void) {
    rcu_periph_clock_enable(KEYS_CLK);
    gpio_init(KEYS_GPIO, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, KEYS_MASK);
}

void NumericKeyboardInit(void) {
    rcu_periph_clock_enable(NUMS_ROWS_CLK);
    gpio_init(NUMS_ROWS_GPIO, GPIO_MODE_IPD, GPIO_OSPEED_50MHZ, NUMS_ROWS_MASK);
}

void PruebaColGndRowPullUp(void) {
    uint16_t resultado;
    volatile uint32_t valor;

    rcu_periph_clock_enable(SEGMENTS_CLK);
    rcu_periph_clock_enable(DIGITS_CLK);

    GPIO_BC(NUMS_ROWS_GPIO) = NUMS_ROWS_MASK;
    GPIO_BC(NUMS_COLS_GPIO) = NUMS_COLS_MASK;

    gpio_init(NUMS_COLS_GPIO, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, NUMS_COLS_MASK);

    while (1) {
        resultado = 0;

        for (int col = 0; col < 4; col++) {
            gpio_init(NUMS_ROWS_GPIO, GPIO_MODE_IPD, GPIO_OSPEED_50MHZ, NUMS_ROWS_MASK);

            switch (col) {
            case 1:
                // gpio_init(NUMS_COLS_GPIO, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, NUMS_C2_MASK);
                // GPIO_BOP(NUMS_COLS_GPIO) = NUMS_C2_MASK;
                gpio_init(NUMS_ROWS_GPIO, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, NUMS_R2_MASK);
                // GPIO_BOP(NUMS_ROWS_GPIO) = NUMS_R2_MASK;
                break;
            case 2:
                // gpio_init(NUMS_COLS_GPIO, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, NUMS_C3_MASK);
                // GPIO_BOP(NUMS_COLS_GPIO) = NUMS_C3_MASK;
                gpio_init(NUMS_ROWS_GPIO, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, NUMS_R3_MASK);
                // GPIO_BOP(NUMS_ROWS_GPIO) = NUMS_R3_MASK;
                break;
            case 3:
                // gpio_init(NUMS_COLS_GPIO, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, NUMS_C4_MASK);
                // GPIO_BOP(NUMS_COLS_GPIO) = NUMS_C4_MASK;
                gpio_init(NUMS_ROWS_GPIO, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, NUMS_R4_MASK);
                // GPIO_BOP(NUMS_ROWS_GPIO) = NUMS_R4_MASK;
                break;
            default:
                // gpio_init(NUMS_COLS_GPIO, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, NUMS_C1_MASK);
                // GPIO_BOP(NUMS_COLS_GPIO) = NUMS_C1_MASK;
                gpio_init(NUMS_ROWS_GPIO, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, NUMS_R1_MASK);
                // GPIO_BOP(NUMS_ROWS_GPIO) = NUMS_R1_MASK;
                break;
            }
            delay_1ms(1);

            // valor = GPIO_ISTAT(NUMS_ROWS_GPIO);
            // valor &= NUMS_ROWS_MASK;
            // valor >>= NUMS_R1_BIT;

            valor = GPIO_ISTAT(NUMS_COLS_GPIO);
            valor &= NUMS_COLS_MASK;
            valor >>= NUMS_C1_BIT;

            // gpio_init(NUMS_COLS_GPIO, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, NUMS_COLS_MASK);
            resultado |= (valor << (col * 4));
        }
        if (resultado) {
            LedRgbSetLevel(LED_RGB_CHANNEL_R, 20000);
        } else {
            LedRgbSetLevel(LED_RGB_CHANNEL_R, 0);
        }
    }
}

void Prueba2(void) {
    uint16_t resultado;
    volatile uint32_t valor;

    rcu_periph_clock_enable(SEGMENTS_CLK);
    rcu_periph_clock_enable(DIGITS_CLK);

    GPIO_BC(NUMS_ROWS_GPIO) = NUMS_ROWS_MASK;
    GPIO_BC(NUMS_COLS_GPIO) = NUMS_COLS_MASK;

    while (1) {
        // resultado = NumericKeyboardScan();

        // if (resultado) {
        //     LedRgbSetLevel(LED_RGB_CHANNEL_R, 20000);
        // } else {
        //     LedRgbSetLevel(LED_RGB_CHANNEL_R, 0);
        // }
    }

    gpio_init(NUMS_ROWS_GPIO, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, NUMS_ROWS_MASK);

    while (1) {
        resultado = 0;

        for (int col = 0; col < 4; col++) {
            gpio_init(NUMS_COLS_GPIO, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, NUMS_COLS_MASK);

            switch (col) {
            case 1:
                gpio_init(NUMS_COLS_GPIO, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, NUMS_C2_MASK);
                GPIO_BC(NUMS_COLS_GPIO) = NUMS_C2_MASK;
                // GPIO_BOP(NUMS_COLS_GPIO) = NUMS_C2_MASK;
                // gpio_init(NUMS_ROWS_GPIO, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, NUMS_R2_MASK);
                // GPIO_BOP(NUMS_ROWS_GPIO) = NUMS_R2_MASK;
                break;
            case 2:
                gpio_init(NUMS_COLS_GPIO, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, NUMS_C3_MASK);
                GPIO_BC(NUMS_COLS_GPIO) = NUMS_C3_MASK;
                // GPIO_BOP(NUMS_COLS_GPIO) = NUMS_C3_MASK;
                // gpio_init(NUMS_ROWS_GPIO, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, NUMS_R3_MASK);
                // GPIO_BOP(NUMS_ROWS_GPIO) = NUMS_R3_MASK;
                break;
            case 3:
                gpio_init(NUMS_COLS_GPIO, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, NUMS_C4_MASK);
                GPIO_BC(NUMS_COLS_GPIO) = NUMS_C4_MASK;
                // GPIO_BOP(NUMS_COLS_GPIO) = NUMS_C4_MASK;
                // gpio_init(NUMS_ROWS_GPIO, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, NUMS_R4_MASK);
                // GPIO_BOP(NUMS_ROWS_GPIO) = NUMS_R4_MASK;
                break;
            default:
                gpio_init(NUMS_COLS_GPIO, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, NUMS_C1_MASK);
                GPIO_BC(NUMS_COLS_GPIO) = NUMS_C1_MASK;
                // GPIO_BOP(NUMS_COLS_GPIO) = NUMS_C1_MASK;
                // gpio_init(NUMS_ROWS_GPIO, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, NUMS_R1_MASK);
                // GPIO_BOP(NUMS_ROWS_GPIO) = NUMS_R1_MASK;
                break;
            }
            delay_1ms(1);

            valor = ~GPIO_ISTAT(NUMS_ROWS_GPIO);
            valor &= NUMS_ROWS_MASK;
            valor >>= NUMS_R1_BIT;

            // valor = GPIO_ISTAT(NUMS_COLS_GPIO);
            // valor &= NUMS_COLS_MASK;
            // valor >>= NUMS_C1_BIT;

            // gpio_init(NUMS_COLS_GPIO, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, NUMS_COLS_MASK);
            resultado |= (valor << (col * 4));
        }
        if (resultado) {
            LedRgbSetLevel(LED_RGB_CHANNEL_R, 20000);
        } else {
            LedRgbSetLevel(LED_RGB_CHANNEL_R, 0);
        }
    }
}

void AnalogInit(void) {
    /* enable ADC clock */
    rcu_periph_clock_enable(RCU_ADC0);
    /* config ADC clock */
    rcu_adc_clock_config(RCU_CKADC_CKAPB2_DIV8);

    /* reset ADC */
    adc_deinit(ADC0);
    /* ADC mode config */
    adc_mode_config(ADC_MODE_FREE);
    /* ADC scan function enable */
    adc_special_function_config(ADC0, ADC_SCAN_MODE, ENABLE);
    /* ADC data alignment config */
    adc_data_alignment_config(ADC0, ADC_DATAALIGN_RIGHT);
    /* ADC temperature and Vrefint enable */
    adc_tempsensor_vrefint_enable();

    /* ADC channel length config */
    adc_channel_length_config(ADC0, ADC_INSERTED_CHANNEL, 2);

    /* ADC temperature sensor channel config */
    adc_inserted_channel_config(ADC0, 0, ADC_CHANNEL_8, ADC_SAMPLETIME_239POINT5);
    /* ADC internal reference voltage channel config */
    adc_inserted_channel_config(ADC0, 1, ADC_CHANNEL_9, ADC_SAMPLETIME_239POINT5);

    /* ADC trigger config */
    adc_external_trigger_source_config(ADC0, ADC_INSERTED_CHANNEL, ADC0_1_EXTTRIG_INSERTED_NONE);

    adc_external_trigger_config(ADC0, ADC_INSERTED_CHANNEL, ENABLE);

    /* enable ADC interface */
    adc_enable(ADC0);
    delay_1ms(1);
    /* ADC calibration and reset calibration */
    adc_calibration_enable(ADC0);
}

uint16_t AnalogRead(uint8_t channel) {
    static uint16_t valores[FILTER_LENGTH][2];
    static uint8_t siguiente[2];
    uint32_t acumulador;

    valores[siguiente[channel]][channel] = adc_inserted_data_read(ADC0, channel);
    siguiente[channel] = (siguiente[channel] + 1) % FILTER_LENGTH;

    acumulador = 0;
    for (int i = 0; i < FILTER_LENGTH; i++) {
        acumulador += valores[i][channel];
    }
    return acumulador / FILTER_LENGTH;
}
/* === Public function implementation ========================================================== */

int main(void) {
    uint32_t valor = 0;
    uint32_t divisor = 0;
    uint32_t contador = 0;
    uint8_t variable = 0;
    uint16_t temperatura = 0;
    uint16_t luminosidad = 0;
    uint16_t decimas = 0;
    uint16_t segundos = 0;
    uint8_t tecla;

    BoardSetup();
    SonidoInit();
    // SonidoPlayMelody(ESCALA);

    DigitsInit();
    SegmentsInit();
    LedRgbInit(true);
    FunctionKeyboardInit();
    NumericKeyboardInit();
    AnalogInit();
    // EncoderInit();

    // Prueba2();
    display_t display = DisplayCreate(4, &(struct display_driver_s){
                                             .ScreenTurnOff = ScreenTurnOff,
                                             .SegmentsTurnOn = SegmentsTurnOn,
                                             .DigitTurnOn = DigitTurnOn,
                                         });

    keyboard_t functions = KeyboardCreate(6, FunctionKeyboardScan);

    numeric = KeyboardCreate(16, NULL);

    DisplayWriteValue(display, valor);
    adc_software_trigger_enable(ADC0, ADC_INSERTED_CHANNEL);

    while (1) {
        if (divisor == 0) {
            divisor = LedRgbDemoTick();
        } else {
            divisor--;
        }

        decimas = decimas + 1;
        if (decimas >= 100) {
            luminosidad = AnalogRead(ADC_INSERTED_CHANNEL_0);
            temperatura = AnalogRead(ADC_INSERTED_CHANNEL_1);
            adc_software_trigger_enable(ADC0, ADC_INSERTED_CHANNEL);
        }

        if (variable) {
            if (segundos == 0) {
                switch (variable) {
                case 1:
                    DisplayWriteValue(display, temperatura);
                    break;
                case 2:
                    DisplayWriteValue(display, luminosidad);
                    break;
                }
            }
            segundos = (segundos + 1) % 500;
        }

        contador = (contador + 1) % 50;
        if (contador == 0) {

            if (KeyboardScan(functions)) {
                if (KeyboardChangeToPressed(functions, 2)) {
                    valor = (valor + 1000) % 10000;
                    variable = 0;
                    SonidoPlayNote(SONIDO_DO_6, SONIDO_NEGRA);
                }
                if (KeyboardChangeToPressed(functions, 3)) {
                    valor = (valor + 100) % 10000;
                    variable = 0;
                    SonidoPlayNote(SONIDO_MI_6, SONIDO_NEGRA);
                }
                if (KeyboardChangeToPressed(functions, 4)) {
                    valor = (valor + 10) % 10000;
                    variable = 0;
                    SonidoPlayNote(SONIDO_SOL_6, SONIDO_NEGRA);
                }
                if (KeyboardChangeToPressed(functions, 5)) {
                    valor = (valor + 1) % 10000;
                    variable = 0;
                    SonidoPlayNote(SONIDO_SI_6, SONIDO_NEGRA);
                }
                if (KeyboardChangeToPressed(functions, 0)) {
                    // valor = 9999;
                    variable = 2;
                    segundos = 0;
                    SonidoPlayMelody(LA_CUCARACHA);
                }
                if (KeyboardChangeToPressed(functions, 1)) {
                    // valor = 0;
                    variable = 1;
                    segundos = 0;
                    SonidoPlayMelody(ESCALA);
                }
                if (variable == 0) {
                    DisplayWriteValue(display, valor);
                }
            }
        }

        if (KeyboardScan(numeric)) {
            tecla = KeyboardGetFirstPressed(numeric);
            if (tecla != 0xFF) {
                valor = tecla + 1;
                variable = 0;
            }
            DisplayWriteValue(display, valor);
        }
        // if (EncoderHasChanged()) {
        //     variable = 3;
        //     DisplayWriteValue(display, EncoderReadValue());
        // }

        DisplayRefresh(display);

        delay_1ms(1);
    }
    return 0;
}

/* === End of documentation ==================================================================== */

/** @} End of module definition for doxygen */
