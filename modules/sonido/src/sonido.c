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

#include "sonido.h"
#include "board.h"
#include <stddef.h>

/* === Macros definitions ====================================================================== */

#define SONIDO_TIMER     TIMER2
#define SONIDO_TIMER_CLK RCU_TIMER2
#define SONIDO_TRIGGER   TIMER_SMCFG_TRGSEL_ITI0
#define SONIDO_DIVIDER   4

#define SONIDO_GPIO_PORT GPIOC
#define SONIDO_GPIO_CLK  RCU_GPIOC

#define SONIDO_GPIO_BIT  7
#define SONIDO_GPIO_MASK (1 << SONIDO_GPIO_BIT)
#define SONIDO_CHANNEL   TIMER_CH_1

#define TIEMPO_TIMER     TIMER0
#define TIEMPO_TIMER_CLK RCU_TIMER0
#define TIEMPO_IRQ_NUM   TIMER0_Channel_IRQn

#define TIEMPO_CHANNEL   TIMER_CH_0
#define TIEMPO_INT_MASK  TIMER_INT_CH0
#define TIEMPO_INT_FLAG  TIMER_INT_FLAG_CH0

#define TiempoIRQHandler TIMER0_Channel_IRQHandler

/* === Private data type declarations ==========================================================
 */

/* === Private variable declarations ===========================================================
 */

/* === Private function declarations ===========================================================
 */

/* === Public variable definitions =============================================================
 */

const sonido_nota_t LA_CUCARACHA[] = {
    {SONIDO_SOL_5, SONIDO_CORCHEA + SONIDO_SEMICORCHEA},
    {SONIDO_SILENCIO, SONIDO_SEMICORCHEA},
    {SONIDO_SOL_5, SONIDO_CORCHEA + SONIDO_SEMICORCHEA},
    {SONIDO_SILENCIO, SONIDO_SEMICORCHEA},
    {SONIDO_SOL_5, SONIDO_CORCHEA + SONIDO_SEMICORCHEA},
    {SONIDO_SILENCIO, SONIDO_SEMICORCHEA},
    {SONIDO_DO_6, SONIDO_BLANCA},
    {SONIDO_MI_6, SONIDO_BLANCA},

    {SONIDO_SOL_5, SONIDO_CORCHEA + SONIDO_SEMICORCHEA},
    {SONIDO_SILENCIO, SONIDO_SEMICORCHEA},
    {SONIDO_SOL_5, SONIDO_CORCHEA + SONIDO_SEMICORCHEA},
    {SONIDO_SILENCIO, SONIDO_SEMICORCHEA},
    {SONIDO_SOL_5, SONIDO_CORCHEA + SONIDO_SEMICORCHEA},
    {SONIDO_SILENCIO, SONIDO_SEMICORCHEA},
    {SONIDO_DO_6, SONIDO_BLANCA},
    {SONIDO_MI_6, SONIDO_BLANCA},

    {SONIDO_SILENCIO, 0}, // Final de la canción
};

const sonido_nota_t ESCALA[] = {
    {SONIDO_DO_5, SONIDO_NEGRA},  {SONIDO_RE_5, SONIDO_NEGRA}, {SONIDO_MI_5, SONIDO_NEGRA}, {SONIDO_FA_5, SONIDO_NEGRA},
    {SONIDO_SOL_5, SONIDO_NEGRA}, {SONIDO_LA_5, SONIDO_NEGRA}, {SONIDO_SI_5, SONIDO_NEGRA},

    {SONIDO_DO_6, SONIDO_NEGRA},  {SONIDO_RE_6, SONIDO_NEGRA}, {SONIDO_MI_6, SONIDO_NEGRA}, {SONIDO_FA_6, SONIDO_NEGRA},
    {SONIDO_SOL_6, SONIDO_NEGRA}, {SONIDO_LA_6, SONIDO_NEGRA}, {SONIDO_SI_6, SONIDO_NEGRA},

    {SONIDO_SILENCIO, 0}, // Final de la canción
};

const sonido_nota_t QUINTA[] = {
    {SONIDO_MI_6, SONIDO_NEGRA},
    {SONIDO_SILENCIO, SONIDO_SEMICORCHEA},
    {SONIDO_MI_6, SONIDO_NEGRA},
    {SONIDO_SILENCIO, SONIDO_SEMICORCHEA},
    {SONIDO_MI_6, SONIDO_NEGRA},
    {SONIDO_SILENCIO, SONIDO_SEMICORCHEA},
    {SONIDO_DO_5, SONIDO_BLANCA + SONIDO_NEGRA},
    {SONIDO_SILENCIO, SONIDO_NEGRA},
    {SONIDO_RE_6, SONIDO_NEGRA},
    {SONIDO_SILENCIO, SONIDO_SEMICORCHEA},
    {SONIDO_RE_6, SONIDO_NEGRA},
    {SONIDO_SILENCIO, SONIDO_SEMICORCHEA},
    {SONIDO_RE_6, SONIDO_NEGRA},
    {SONIDO_SILENCIO, SONIDO_SEMICORCHEA},
    {SONIDO_SI_5, SONIDO_BLANCA + SONIDO_NEGRA},
    {SONIDO_SILENCIO, 0}, // Final de la canción
};

/* === Private variable definitions ============================================================ */

static const sonido_nota_t * current_note = NULL;

/* === Private function implementation ========================================================= */

void TiempoIRQHandler(void);

/* === Public function implementation ========================================================== */

void SonidoInit(void) {
    timer_parameter_struct timer_config;
    timer_oc_parameter_struct channel_config;

    /* enable the led clock */
    rcu_periph_clock_enable(RCU_AF);
    rcu_periph_clock_enable(SONIDO_GPIO_CLK);
    rcu_periph_clock_enable(SONIDO_TIMER_CLK);
    rcu_periph_clock_enable(TIEMPO_TIMER_CLK);

    /* configure led GPIO port */
    gpio_init(SONIDO_GPIO_PORT, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, SONIDO_GPIO_MASK);
    gpio_pin_remap_config(GPIO_TIMER2_FULL_REMAP, ENABLE);

    /* sound timer configuration */
    timer_deinit(SONIDO_TIMER);
    timer_struct_para_init(&timer_config);
    timer_config.prescaler = SONIDO_DIVIDER;
    timer_config.alignedmode = TIMER_COUNTER_EDGE;
    timer_config.counterdirection = TIMER_COUNTER_UP;
    timer_config.period = SystemCoreClock / 2200;
    timer_config.clockdivision = TIMER_CKDIV_DIV1;
    timer_init(SONIDO_TIMER, &timer_config);

    timer_input_trigger_source_select(SONIDO_TIMER, SONIDO_TRIGGER);
    timer_slave_mode_select(SONIDO_TIMER, TIMER_SLAVE_MODE_PAUSE);
    timer_master_slave_mode_config(SONIDO_TIMER, TIMER_MASTER_SLAVE_MODE_ENABLE);

    /* channels configuration in PWM mode */
    timer_channel_output_struct_para_init(&channel_config);
    channel_config.outputstate = TIMER_CCX_ENABLE;
    channel_config.ocpolarity = TIMER_OC_POLARITY_LOW;
    channel_config.ocidlestate = TIMER_OC_IDLE_STATE_HIGH;
    timer_channel_output_config(SONIDO_TIMER, SONIDO_CHANNEL, &channel_config);

    /* channels configuration in PWM mode1 with duty cycle 0% */
    timer_channel_output_pulse_value_config(SONIDO_TIMER, SONIDO_CHANNEL, 0);
    timer_channel_output_mode_config(SONIDO_TIMER, SONIDO_CHANNEL, TIMER_OC_MODE_PWM1);
    timer_channel_output_shadow_config(SONIDO_TIMER, SONIDO_CHANNEL, TIMER_OC_SHADOW_DISABLE);
    timer_enable(SONIDO_TIMER);

    /* duration timer configuration */
    timer_deinit(TIEMPO_TIMER);
    timer_struct_para_init(&timer_config);
    timer_config.prescaler = SystemCoreClock / 1000 - 1;
    timer_config.alignedmode = TIMER_COUNTER_EDGE;
    timer_config.counterdirection = TIMER_COUNTER_DOWN;
    timer_config.clockdivision = TIMER_CKDIV_DIV1;
    timer_init(TIEMPO_TIMER, &timer_config);

    timer_master_output_trigger_source_select(TIEMPO_TIMER, TIMER_TRI_OUT_SRC_ENABLE);
    timer_master_slave_mode_config(TIEMPO_TIMER, TIMER_MASTER_SLAVE_MODE_ENABLE);
    timer_single_pulse_mode_config(TIEMPO_TIMER, TIMER_SP_MODE_SINGLE);

    timer_channel_output_struct_para_init(&channel_config);
    channel_config.outputstate = TIMER_CCX_ENABLE;
    channel_config.outputnstate = TIMER_CCXN_DISABLE;
    channel_config.ocidlestate = TIMER_OC_IDLE_STATE_LOW;
    timer_channel_output_config(TIEMPO_TIMER, TIEMPO_CHANNEL, &channel_config);
    timer_channel_output_mode_config(TIEMPO_TIMER, TIEMPO_CHANNEL, TIMER_OC_MODE_TIMING);
    timer_channel_output_pulse_value_config(TIEMPO_TIMER, TIEMPO_CHANNEL, 0);

    // timer_enable(TIEMPO_TIMER);
}

void SonidoPlayNote(uint32_t frequency, uint32_t duration) {
    if (frequency != SONIDO_SILENCIO) {
        uint16_t period = SystemCoreClock / frequency / (SONIDO_DIVIDER + 1);
        timer_autoreload_value_config(SONIDO_TIMER, period);
        timer_channel_output_pulse_value_config(SONIDO_TIMER, SONIDO_CHANNEL, period / 2);
    } else {
        timer_channel_output_pulse_value_config(SONIDO_TIMER, SONIDO_CHANNEL, 0);
    }

    timer_counter_value_config(TIEMPO_TIMER, duration);
    timer_interrupt_flag_clear(TIEMPO_TIMER, TIEMPO_INT_FLAG);
    timer_enable(TIEMPO_TIMER);
}

void SonidoPlayMelody(const sonido_nota_t * melody) {
    // ECLIC_Register_IRQ(TIEMPO_IRQ_NUM, ECLIC_VECTOR_INTERRUPT, ECLIC_POSTIVE_EDGE_TRIGGER, 2, 0,
    //                    (void *)TiempoIRQHandler);

    eclic_mode_enable();
    // ECLIC_ClearPendingIRQ(TIEMPO_IRQ_NUM);
    eclic_clear_pending(TIEMPO_IRQ_NUM);
    // ECLIC_EnableIRQ(TIEMPO_IRQ_NUM);
    eclic_irq_enable(TIEMPO_IRQ_NUM, 1, 1);
    // __enable_irq();
    eclic_global_interrupt_enable();

    current_note = melody;
    SonidoPlayNote(current_note->frequency, current_note->duration);
    timer_interrupt_enable(TIEMPO_TIMER, TIEMPO_INT_MASK);
}

void TiempoIRQHandler(void) {
    if (timer_interrupt_flag_get(TIEMPO_TIMER, TIEMPO_INT_FLAG) != RESET) {
        timer_interrupt_flag_clear(TIEMPO_TIMER, TIEMPO_INT_FLAG);
        current_note++;
        if (current_note->duration != 0) {
            SonidoPlayNote(current_note->frequency, current_note->duration);
        } else {
            timer_interrupt_disable(TIEMPO_TIMER, TIEMPO_INT_MASK);
        }
    }
}

/* === End of documentation ==================================================================== */

/** @} End of module definition for doxygen */
