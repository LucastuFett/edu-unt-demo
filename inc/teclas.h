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

#ifndef TECLAS_H
#define TECLAS_H

/** \brief Longan NANO board declarations
 **
 ** \addtogroup board Board support
 ** \brief Board agnostic configuration module
 ** @{ */

/* === Headers files inclusions ================================================================ */

#include <stdint.h>
#include <stdbool.h>

/* === Cabecera C++ ============================================================================ */

#ifdef __cplusplus
extern "C" {
#endif

/* === Public macros definitions =============================================================== */

/* === Public data type declarations =========================================================== */

//! @brief Tipo de dato para referenciar a un teclado
typedef struct keyboard_s * keyboard_t;

//! Función de callback para apagar los segmentos y digitos de la pantalla
typedef uint32_t (*keyboard_scan_f)(uint8_t key_count);

/* === Public variable declarations ============================================================ */

/* === Public function declarations ============================================================ */

keyboard_t KeyboardCreate(uint8_t key_count, keyboard_scan_f scan_function);

bool KeyboardScan(keyboard_t keyboard);
void KeyboardUpdate(keyboard_t keyboard, uint32_t new_state);
bool KeyboardChangeState(keyboard_t keyboard, uint8_t key);
bool KeyboardChangeToPressed(keyboard_t keyboard, uint8_t key);
bool KeyboardChangeToReleased(keyboard_t keyboard, uint8_t key);
bool KeyboardIsActualyPressed(keyboard_t keyboard, uint8_t key);
bool KeyboardIsActualyReleased(keyboard_t keyboard, uint8_t key);
uint8_t KeyboardGetFirstPressed(keyboard_t keyboard);
/* === End of documentation ==================================================================== */

#ifdef __cplusplus
}
#endif

/** @} End of module definition for doxygen */

#endif /* TECLAS_H */
