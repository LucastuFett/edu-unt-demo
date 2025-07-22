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

#ifndef SONIDO_H
#define SONIDO_H

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

#ifndef SONIDO_BPM
#define SONIDO_BPM 120
#endif

#define SONIDO_NEGRA       (60000 / SONIDO_BPM)
#define SONIDO_BLANCA      (2 * SONIDO_NEGRA)
#define SONIDO_REDONDA     (2 * SONIDO_BLANCA)
#define SONIDO_CORCHEA     (SONIDO_NEGRA / 2)
#define SONIDO_SEMICORCHEA (SONIDO_CORCHEA / 2)

#define SONIDO_SILENCIO    0

#define SONIDO_DO_4        261
#define SONIDO_RE_4        294
#define SONIDO_MI_4        329
#define SONIDO_FA_4        349
#define SONIDO_SOL_4       392
#define SONIDO_LA_4        440
#define SONIDO_SI_4        493

#define SONIDO_DO_5        523
#define SONIDO_RE_5        587
#define SONIDO_MI_5        659
#define SONIDO_FA_5        698
#define SONIDO_SOL_5       784
#define SONIDO_LA_5        880
#define SONIDO_SI_5        988

#define SONIDO_DO_6        1047
#define SONIDO_RE_6        1175
#define SONIDO_MI_6        1319
#define SONIDO_FA_6        1397
#define SONIDO_SOL_6       1568
#define SONIDO_LA_6        1760
#define SONIDO_SI_6        1976

/* === Public data type declarations =========================================================== */

typedef struct sonido_nota_s {
    uint32_t frequency;
    uint32_t duration;
} sonido_nota_t;

/* === Public variable declarations ============================================================ */

extern const sonido_nota_t LA_CUCARACHA[];

extern const sonido_nota_t ESCALA[];

/* === Public function declarations ============================================================ */

void SonidoInit(void);

void SonidoPlayNote(uint32_t frequency, uint32_t duration);

void SonidoPlayMelody(const sonido_nota_t * melody);

void SonidoPlayIntro(void);

/* === End of documentation ==================================================================== */

#ifdef __cplusplus
}
#endif

/** @} End of module definition for doxygen */

#endif /* SONIDO_H */
