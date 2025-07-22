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

#include "rgb.h"
#include "board.h"

/* === Macros definitions ====================================================================== */

#define LED_RGB_LEDS      3

#define LED_RGB_GPIO_PORT GPIOA
#define LED_RGB_GPIO_CLK  RCU_GPIOA
#define LED_RGB_TIMER     TIMER1
#define LED_RGB_TIMER_CLK RCU_TIMER1

#define LED_R_GPIO_BIT    1
#define LED_R_GPIO_MASK   (1 << LED_R_GPIO_BIT)
#define LED_R_PWM_CHANNEL TIMER_CH_1

#define LED_G_GPIO_BIT    2
#define LED_G_GPIO_MASK   (1 << LED_G_GPIO_BIT)
#define LED_G_PWM_CHANNEL TIMER_CH_2

#define LED_B_GPIO_BIT    3
#define LED_B_GPIO_MASK   (1 << LED_B_GPIO_BIT)
#define LED_B_PWM_CHANNEL TIMER_CH_3

#define LED_PWM_PERIOD    20000
#define LED_PWM_DELTA     (LED_PWM_PERIOD / 50)
#define LEDS_GPIO_MASK    (LED_R_GPIO_MASK | LED_G_GPIO_MASK | LED_B_GPIO_MASK)

/* === Private data type declarations ========================================================== */

/* === Private variable declarations =========================================================== */

static const uint32_t LED_RGB_GPIO_MASKS[LED_RGB_LEDS] = {
    LED_R_GPIO_MASK,
    LED_G_GPIO_MASK,
    LED_B_GPIO_MASK,
};

static const uint32_t LED_RGB_PWM_CHANNELS[LED_RGB_LEDS] = {
    LED_R_PWM_CHANNEL,
    LED_G_PWM_CHANNEL,
    LED_B_PWM_CHANNEL,
};

/* === Private function declarations =========================================================== */

void LedRgbGpioInit(void);

/* === Public variable definitions ============================================================= */

/* === Private variable definitions ============================================================ */

static bool pwm_enabled = false;

/* === Private function implementation ========================================================= */

void LedRgbGpioInit(void) {
    /* enable the led clock */
    rcu_periph_clock_enable(LED_RGB_GPIO_CLK);
    /* configure led GPIO port */
    gpio_init(LED_RGB_GPIO_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, LEDS_GPIO_MASK);

    LedRgbSetOff(LED_RGB_CHANNEL_R);
    LedRgbSetOff(LED_RGB_CHANNEL_G);
    LedRgbSetOff(LED_RGB_CHANNEL_B);
}

void LedRgbPwmInit(void) {
    timer_parameter_struct timer_config;
    timer_oc_parameter_struct channel_config;

    /* enable the led clock */
    rcu_periph_clock_enable(RCU_AF);
    rcu_periph_clock_enable(LED_RGB_GPIO_CLK);
    rcu_periph_clock_enable(LED_RGB_TIMER_CLK);

    /* configure led GPIO port */
    gpio_init(LED_RGB_GPIO_PORT, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, LEDS_GPIO_MASK);

    /* timer configuration */
    timer_deinit(LED_RGB_TIMER);
    timer_struct_para_init(&timer_config);
    timer_config.prescaler = SystemCoreClock / LED_PWM_PERIOD / 20000;
    timer_config.alignedmode = TIMER_COUNTER_EDGE;
    timer_config.counterdirection = TIMER_COUNTER_UP;
    timer_config.period = LED_PWM_PERIOD;
    timer_config.clockdivision = TIMER_CKDIV_DIV1;
    timer_config.repetitioncounter = 0;
    timer_init(LED_RGB_TIMER, &timer_config);

    /* channels configuration in PWM mode */
    timer_channel_output_struct_para_init(&channel_config);
    channel_config.outputstate = TIMER_CCX_ENABLE;
    channel_config.outputnstate = TIMER_CCXN_DISABLE;
    channel_config.ocpolarity = TIMER_OC_POLARITY_LOW;
    channel_config.ocnpolarity = TIMER_OCN_POLARITY_LOW;
    channel_config.ocidlestate = TIMER_OC_IDLE_STATE_HIGH;
    channel_config.ocnidlestate = TIMER_OCN_IDLE_STATE_HIGH;
    timer_channel_output_config(LED_RGB_TIMER, LED_R_PWM_CHANNEL, &channel_config);
    timer_channel_output_config(LED_RGB_TIMER, LED_G_PWM_CHANNEL, &channel_config);
    timer_channel_output_config(LED_RGB_TIMER, LED_B_PWM_CHANNEL, &channel_config);

    /* channels configuration in PWM mode1 with duty cycle 0% */
    timer_channel_output_pulse_value_config(LED_RGB_TIMER, LED_R_PWM_CHANNEL, 0);
    timer_channel_output_mode_config(LED_RGB_TIMER, LED_R_PWM_CHANNEL, TIMER_OC_MODE_PWM1);
    timer_channel_output_shadow_config(LED_RGB_TIMER, LED_R_PWM_CHANNEL, TIMER_OC_SHADOW_DISABLE);

    timer_channel_output_pulse_value_config(LED_RGB_TIMER, LED_G_PWM_CHANNEL, 0);
    timer_channel_output_mode_config(LED_RGB_TIMER, LED_G_PWM_CHANNEL, TIMER_OC_MODE_PWM1);
    timer_channel_output_shadow_config(LED_RGB_TIMER, LED_G_PWM_CHANNEL, TIMER_OC_SHADOW_DISABLE);

    timer_channel_output_pulse_value_config(LED_RGB_TIMER, LED_B_PWM_CHANNEL, 0);
    timer_channel_output_mode_config(LED_RGB_TIMER, LED_B_PWM_CHANNEL, TIMER_OC_MODE_PWM1);
    timer_channel_output_shadow_config(LED_RGB_TIMER, LED_B_PWM_CHANNEL, TIMER_OC_SHADOW_DISABLE);

    /* auto-reload preload enable */
    timer_auto_reload_shadow_enable(LED_RGB_TIMER);
    /* auto-reload preload enable */
    timer_enable(LED_RGB_TIMER);
}

uint32_t LedRgbGpioDemoTick(void) {
    static led_rgb_channels_enum channel = LED_RGB_CHANNEL_R;

    channel = (channel + 1) % LED_RGB_LEDS;
    switch (channel) {
    case LED_RGB_CHANNEL_G:
        /* code */
        LedRgbSetOn(LED_RGB_CHANNEL_G);
        LedRgbSetOff(LED_RGB_CHANNEL_R);
        break;
    case LED_RGB_CHANNEL_B:
        /* code */
        LedRgbSetOn(LED_RGB_CHANNEL_B);
        LedRgbSetOff(LED_RGB_CHANNEL_G);
        break;

    default:
        LedRgbSetOn(LED_RGB_CHANNEL_R);
        LedRgbSetOff(LED_RGB_CHANNEL_B);
        break;
    }
    return 500;
}

uint32_t LedRgbPwmDemoTick(void) {
    static int value = 0;
    static int change = LED_PWM_DELTA;
    static led_rgb_channels_enum channel = LED_RGB_CHANNEL_R;
    uint32_t delay;

    delay = 10;
    value = value + change;
    if (value > LED_PWM_PERIOD) {
        value = LED_PWM_PERIOD;
        change = -LED_PWM_DELTA;
        delay = 100;
    } else if (value < 0) {
        value = 0;
        change = LED_PWM_DELTA;
        delay = 100;
    }

    if (value == 0) {
        LedRgbSetLevel(channel, value);
        channel = (channel + 1) % LED_RGB_LEDS;
    }

    LedRgbSetLevel(channel, value);
    return delay;
}

/* === Public function implementation ========================================================== */

void LedRgbInit(bool use_pwm) {
    pwm_enabled = use_pwm;
    if (pwm_enabled) {
        LedRgbPwmInit();
    } else {
        LedRgbGpioInit();
    }
}

void LedRgbSetOn(led_rgb_channels_enum led) {
    GPIO_BOP(LED_RGB_GPIO_PORT) = LED_RGB_GPIO_MASKS[led];
}

void LedRgbSetOff(led_rgb_channels_enum led) {
    GPIO_BC(LED_RGB_GPIO_PORT) = LED_RGB_GPIO_MASKS[led];
}

void LedRgbSetLevel(led_rgb_channels_enum led, uint32_t level) {
    timer_channel_output_pulse_value_config(LED_RGB_TIMER, LED_RGB_PWM_CHANNELS[led], level);
}

uint32_t LedRgbDemoTick(void) {
    uint32_t delay;
    if (pwm_enabled) {
        delay = LedRgbPwmDemoTick();
    } else {
        delay = LedRgbGpioDemoTick();
    }
    return delay;
}
/* === End of documentation ====================================================================
 */

/** @} End of module definition for doxygen */
