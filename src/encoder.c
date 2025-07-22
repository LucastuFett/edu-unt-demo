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

#include "encoder.h"
#include "board.h"

/* === Macros definitions ====================================================================== */

#define ENCODER_TIMER     TIMER2
#define ENCODER_TIMER_CLK RCU_TIMER2

/* === Private data type declarations ========================================================== */

/* === Private variable declarations =========================================================== */

/* === Private function declarations =========================================================== */

/* === Public variable definitions ============================================================= */

/* === Private variable definitions ============================================================ */

/* === Private function implementation ========================================================= */

/* === Public function implementation ========================================================== */

void EncoderInit(void) {
    rcu_periph_clock_enable(RCU_AF);

    gpio_pin_remap_config(GPIO_SWJ_NONJTRST_REMAP, ENABLE);
    gpio_pin_remap_config(GPIO_TIMER2_PARTIAL_REMAP, ENABLE);

    rcu_periph_clock_enable(ENCODER_TIMER_CLK);

    timer_quadrature_decoder_mode_config(ENCODER_TIMER, TIMER_ENCODER_MODE0, TIMER_IC_POLARITY_FALLING,
                                         TIMER_IC_POLARITY_FALLING);
    timer_autoreload_value_config(ENCODER_TIMER, 9999);
    timer_counter_value_config(ENCODER_TIMER, 0);

    timer_enable(ENCODER_TIMER);
}

bool EncoderHasChanged(void) {
    static uint32_t last_value = 0;
    uint32_t current_value = EncoderReadValue();

    if (current_value != last_value) {
        last_value = current_value;
        return true;
    }
    return false;
}

uint32_t EncoderReadValue(void) {
    return timer_counter_read(ENCODER_TIMER);
}

/* === End of documentation ====================================================================
 */

/** @} End of module definition for doxygen */
