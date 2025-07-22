/* Copyright 2022, Laboratorio de Microprocesadores
 * Facultad de Ciencias Exactas y Tecnología
 * Universidad Nacional de Tucuman
 * http://www.microprocesadores.unt.edu.ar/
 * Copyright 2022, Esteban Volentini <evolentini@herrera.unt.edu.ar>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/** \brief Seven segments mutiplexed display abstraction definitions
 **
 ** \addtogroup hal HAL
 ** \brief Hardware abstraction layer
 ** @{ */

/* === Headers files inclusions =============================================================== */

#include "pantalla.h"
#include <string.h>
#include <stdbool.h>

/* === Macros definitions ====================================================================== */

#ifndef DISPLAY_MAX_DIGITS
#define DISPLAY_MAX_DIGITS 8
#endif

/* === Private data type declarations ========================================================== */

struct display_s {
    uint8_t digits;
    uint8_t active_digit;
    uint8_t memory[DISPLAY_MAX_DIGITS];
    struct display_driver_s driver[1];
};

/* === Private variable declarations =========================================================== */

static const uint8_t IMAGES[] = {
    SEGMENT_A | SEGMENT_B | SEGMENT_C | SEGMENT_D | SEGMENT_E | SEGMENT_F,             //! < 0
    SEGMENT_B | SEGMENT_C,                                                             //! < 1
    SEGMENT_A | SEGMENT_B | SEGMENT_D | SEGMENT_E | SEGMENT_G,                         //! < 2
    SEGMENT_A | SEGMENT_B | SEGMENT_C | SEGMENT_D | SEGMENT_G,                         //! < 3
    SEGMENT_B | SEGMENT_C | SEGMENT_F | SEGMENT_G,                                     //! < 4
    SEGMENT_A | SEGMENT_C | SEGMENT_D | SEGMENT_F | SEGMENT_G,                         //! < 5
    SEGMENT_A | SEGMENT_C | SEGMENT_D | SEGMENT_E | SEGMENT_F | SEGMENT_G,             //! < 6
    SEGMENT_A | SEGMENT_B | SEGMENT_C,                                                 //! < 7
    SEGMENT_A | SEGMENT_B | SEGMENT_C | SEGMENT_D | SEGMENT_E | SEGMENT_F | SEGMENT_G, //! < 8
    SEGMENT_A | SEGMENT_B | SEGMENT_C | SEGMENT_F | SEGMENT_G,                         //! < 9
};

/* === Private function declarations =========================================================== */

// Function para asignar un descriptor para crear una nueva pantalla de siete segmentos
static display_t DisplayAllocate(void);

/* === Public variable definitions ============================================================= */

/* === Private variable definitions ============================================================ */

/* === Private function implementation ========================================================= */

display_t DisplayAllocate(void) {
    static struct display_s instances[1] = {0};

    return &instances[0];
}

static bool DriverIsValid(display_driver_t driver) {
    bool result = false;
    if (driver) {
        result = (driver->ScreenTurnOff && driver->SegmentsTurnOn && driver->DigitTurnOn);
    }
    return result;
}
/* === Public function implementation ========================================================= */

display_t DisplayCreate(uint8_t digits, display_driver_t driver) {
    display_t display = NULL;

    if (DriverIsValid(driver)) {
        display = DisplayAllocate();
    }

    if (display) {
        display->digits = digits;
        display->active_digit = digits - 1;
        memcpy(display->driver, driver, sizeof(display->driver));
        memset(display->memory, 0, sizeof(display->memory));
        display->driver->ScreenTurnOff();
    }

    return display;
}

void DisplayWriteBCD(display_t display, uint8_t * number, uint8_t size) {
    if (display) {
        memset(display->memory, 0, sizeof(display->memory));
        for (int index = 0; index < size; index++) {
            if (index >= display->digits)
                break;
            display->memory[index] = IMAGES[number[index]];
        }
    }
}

void DisplayWriteValue(display_t display, uint32_t value) {
    uint8_t digit;
    if (display) {
        memset(display->memory, 0, sizeof(display->memory));
        for (int index = display->digits; index > 0; index--) {
            digit = value % 10;
            value = value / 10;
            display->memory[index - 1] = IMAGES[digit];
        }
    }
}

void DisplaySetDot(display_t display, uint32_t digits) {
    if (display) {
        for (int index = 0; index < display->digits; index++) {
            if (digits & (1 << index)) {
                display->memory[index] |= SEGMENT_P;
            }
        }
    }
}

void DisplayRefresh(display_t display) {
    if (display) {
        display->driver->ScreenTurnOff();
        display->active_digit = (display->active_digit + 1) % display->digits;
        display->driver->SegmentsTurnOn(display->memory[display->active_digit]);
        display->driver->DigitTurnOn(display->active_digit);
    }
}

/* === End of documentation ==================================================================== */

/** @} End of module definition for doxygen */
