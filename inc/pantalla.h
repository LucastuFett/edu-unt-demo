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

#ifndef PANTALLA_H
#define PANTALLA_H

/** \brief Seven segments mutiplexed display abstraction declarations
 **
 ** \addtogroup hal HAL
 ** \brief Hardware abstraction layer
 ** @{ */

/* === Headers files inclusions ================================================================ */

#include <stdint.h>

/* === Cabecera C++ ============================================================================ */

#ifdef __cplusplus
extern "C" {
#endif

/* === Public macros definitions =============================================================== */

// Definiciones de bits asociados a cada segmento para construir los digitos
#define SEGMENT_A (1 << 0)
#define SEGMENT_B (1 << 1)
#define SEGMENT_C (1 << 2)
#define SEGMENT_D (1 << 3)
#define SEGMENT_E (1 << 4)
#define SEGMENT_F (1 << 5)
#define SEGMENT_G (1 << 6)
#define SEGMENT_P (1 << 7)

/* === Public data type declarations =========================================================== */

//! Referencia a un descriptor para gestionar una pantalla de siete segmentos mutiplezada
typedef struct display_s * display_t;

//! Función de callback para apagar los segmentos y digitos de la pantalla
typedef void (*display_screen_off_f)(void);

//! Función de callback para prender los segmentos de la pantalla
typedef void (*display_segments_on_f)(uint8_t segments);

//! Función de callback para prender un digito de la pantalla
typedef void (*display_digit_on_f)(uint8_t digit);

//! Estrcutura con las función de bajo nivel para manejo de la pantalla
typedef struct display_driver_s {
    display_screen_off_f ScreenTurnOff;   //!< Función para apagar los segmentos y los digitos
    display_segments_on_f SegmentsTurnOn; //!< Función para prender determinados segmentos
    display_digit_on_f DigitTurnOn;       //!< Función para prender un digito
} const * const display_driver_t;         //!< Puntero a al controlador de la pantalla

/* === Public variable declarations ============================================================ */

/**
 * @brief Metodo para crear una pantalla mutiplexada de siete segmentos
 *
 * @param   digits      Cantidad de digitos que forman la pantalla mutiplexada
 * @param   driver      Puntero a la estrctura con las funciones de bajo nivel
 * @return  display_t   Puntero al descriptor de la pantalla creada
 */
display_t DisplayCreate(uint8_t digits, display_driver_t driver);

/**
 * @brief Función para escribir un numero BCD en la pantalla de siete segmentos
 *
 * @param display       Puntero al descriptor de la pantalla en la que se escribe
 * @param number        Puntero el primer elemento de numero BDC a escribir
 * @param size          Cantidad de elementos en el vector que contienen al numero BCD
 */
void DisplayWriteBCD(display_t display, uint8_t * number, uint8_t size);

void DisplayWriteValue(display_t display, uint32_t value);

/**
 * @brief Función para refrescar la pantalla
 *
 * @param display       Puntero al descriptor de la pantalla que se debe refrescar
 */
void DisplayRefresh(display_t display);

void DisplaySetDot(display_t display, uint32_t digits);
/* === End of documentation ==================================================================== */

#ifdef __cplusplus
}
#endif

/** @} End of module definition for doxygen */

#endif /* PANTALLA_H */
