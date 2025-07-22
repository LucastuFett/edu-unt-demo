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

#include "teclas.h"
#include <stddef.h>

/* === Macros definitions ====================================================================== */
#define MAX_INSTANCES 2

/* === Private data type declarations ========================================================== */

struct keyboard_s {
    keyboard_scan_f ScanFunction;
    uint8_t key_count;
    uint32_t current_state;
    uint32_t previous_state;
    uint32_t updated_state;
};

/* === Private variable declarations =========================================================== */

/* === Private function declarations =========================================================== */

/* === Public variable definitions ============================================================= */

/* === Private variable definitions ============================================================ */

/* === Private function implementation ========================================================= */

static keyboard_t CreateInstance(void) {
    static struct keyboard_s instances[MAX_INSTANCES];

    for (int indice = 0; indice < MAX_INSTANCES; indice++) {
        if (instances[indice].key_count == 0) {
            return &(instances[indice]);
        }
    }
    return NULL;
}

/* === Public function implementation ========================================================== */

keyboard_t KeyboardCreate(uint8_t key_count, keyboard_scan_f scan_function) {
    keyboard_t self = CreateInstance();

    if (self) {
        self->ScanFunction = scan_function;
        self->key_count = key_count;
        self->current_state = 0;
        self->previous_state = 0;
    }
    return self;
}

bool KeyboardScan(keyboard_t self) {
    bool result = false;

    if (self) {
        self->previous_state = self->current_state;
        if (self->ScanFunction) {
            self->current_state = self->ScanFunction(self->key_count);
        } else {
            self->current_state = self->updated_state;
        }
        result = (self->current_state != self->previous_state);
    }
    return result;
}

void KeyboardUpdate(keyboard_t self, uint32_t new_state) {
    if (self) {
        self->updated_state = new_state;
    }
}

bool KeyboardChangeState(keyboard_t self, uint8_t key) {
    bool result = false;
    if (self) {
        result = (self->current_state ^ self->previous_state) & (1 << key);
    }
    return result;
}

bool KeyboardChangeToPressed(keyboard_t self, uint8_t key) {
    bool result = false;
    if (self) {
        result = (self->current_state & ~self->previous_state) & (1 << key);
    }
    return result;
}

bool KeyboardChangeToReleased(keyboard_t self, uint8_t key) {
    bool result = false;
    if (self) {
        result = (~self->current_state & self->previous_state) & (1 << key);
    }
    return result;
}

bool KeyboardIsActualyPressed(keyboard_t self, uint8_t key) {
    bool result = false;
    if (self) {
        result = (self->current_state & (1 << key));
    }
    return result;
}

bool KeyboardIsActualyReleased(keyboard_t self, uint8_t key) {
    bool result = false;
    if (self) {
        result = (~self->current_state & (1 << key));
    }
    return result;
}

uint8_t KeyboardGetFirstPressed(keyboard_t keyboard) {
    uint8_t result = 0xFF;

    if (keyboard) {
        for (uint8_t i = 0; i < keyboard->key_count; i++) {
            if (KeyboardChangeToPressed(keyboard, i)) {
                result = i;
                break;
            }
        }
    }
    return result;
}

/* === End of documentation ====================================================================
 */

/** @} End of module definition for doxygen */
