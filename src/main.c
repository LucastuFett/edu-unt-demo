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

#include "bsp.h"
#include "systick.h"
#include "lcd.h"
#include "gd32v_tf_card_if.h"
#include "rtc.h"

#include <stddef.h>
#include <stdio.h>
#include <string.h>

/* === Macros definitions ====================================================================== */

/* === Private data type declarations ========================================================== */

/* === Private variable declarations =========================================================== */

/* === Private function declarations =========================================================== */

/* === Public variable definitions ============================================================= */

/* === Private variable definitions ============================================================ */

/* === Private function implementation ========================================================= */

static void demo_sdcard(void) {
    SonidoPlayNote(SONIDO_DO_5, SONIDO_NEGRA);

    /* LCD initiation handles SPI configuration
     *  If you don't need the LCD you can check how the LCD initiates SPI in the library */
    LCD_Init();
    SonidoPlayNote(SONIDO_RE_5, SONIDO_NEGRA);

    LCD_Clear(BLACK);
    SonidoPlayNote(SONIDO_MI_5, SONIDO_NEGRA);

    LCD_ShowString(0, 0, "Starting...", WHITE);
    SonidoPlayNote(SONIDO_FA_5, SONIDO_NEGRA);

    /* Handle for the mounted filesystem */
    FATFS fs;

    /* FatFs return code */
    volatile FRESULT fr;

    /* File handle */
    FIL fil;

    /* Used for bytes written, and bytes read */
    // UINT bw = 999;
    UINT br = 0;

    /* A string to write to file */
    char buf[128] = "write this to a file\n";

    /* A buffer for storing an image from the SD-card */
    uint16_t image_buffer[20 * 30] = {0};

    /* For sequencing numbers in loop */
    uint32_t file_no = 0;

    /* Sets a valid date for when writing to file */
    set_fattime(1980, 1, 1, 0, 0, 0);

    /* This function "mounts" the SD-card which makes the filesystem available */
    fr = f_mount(&fs, "", 1); // Mount storage device
    if (fr != FR_OK) {
        LCD_ShowString(0, 0, "SD Mount failed", WHITE);
        while (1); // Stop here if mount fails
    } else {
        LCD_ShowString(0, 0, "SD Mount OK", WHITE);
    }
    SonidoPlayNote(SONIDO_SOL_5, SONIDO_NEGRA);

    /* This function opens a file. In this case we are creating a file which we want to write to */
    // fr = f_open(&fil, "file.txt", FA_WRITE);

    /* Write some text to the file */
    // for (int i = 0; i < 10; i++) fr = f_write(&fil, buf, strlen(buf), &bw);

    // delay_1ms(100);

    /* Close the file */
    // f_close(&fil); // Close file

    // delay_1ms(500);

    /* Now we open a file for reading. */
    fr = f_open(&fil, "1.bin", FA_READ);

    /* Read 20x30x2 bytes since the images are 16bit 20x30px. The data is written into "image_buffer" */
    fr = f_read(&fil, image_buffer, 20 * 30 * 2 + 4, &br);
    f_close(&fil);

    /* "image_buffer" now contains the image and we just have to display it on the screen */
    LCD_ShowPicture(19, 16, 38, 45, (u8 *)image_buffer + 4);
    SonidoPlayNote(SONIDO_LA_5, SONIDO_NEGRA);

    /* The rest follows the same procedure */
    fr = f_open(&fil, "2.bin", FA_READ);
    fr = f_read(&fil, image_buffer, 20 * 30 * 2 + 4, &br);
    f_close(&fil);

    LCD_ShowPicture(39, 16, 58, 45, (u8 *)image_buffer + 4);
    SonidoPlayNote(SONIDO_SI_5, SONIDO_NEGRA);

    fr = f_open(&fil, "colon.bin", FA_READ);
    fr = f_read(&fil, image_buffer, 20 * 30 * 2 + 4, &br);
    f_close(&fil);

    LCD_ShowPicture(59, 16, 78, 45, (u8 *)image_buffer + 4);
    SonidoPlayNote(SONIDO_DO_6, SONIDO_NEGRA);

    fr = f_open(&fil, "3.bin", FA_READ);
    fr = f_read(&fil, image_buffer, 20 * 30 * 2 + 4, &br);
    f_close(&fil);

    LCD_ShowPicture(79, 16, 98, 45, (u8 *)image_buffer + 4);
    SonidoPlayNote(SONIDO_RE_6, SONIDO_NEGRA);

    /* We can also select the file dynamically */
    strcpy(buf, "0.bin");

    // while (1) {

    /* buf[0] will have the value '0' - '9' so the string will loop through "0.bin" -> "1.bin" -> ... -> "9.bin" and
     * so on */
    buf[0] = file_no + '0';
    file_no += 1;
    file_no %= 10;

    fr = f_open(&fil, buf, FA_READ);
    fr = f_read(&fil, image_buffer, 20 * 30 * 2 + 4, &br);
    f_close(&fil);

    LCD_ShowPicture(99, 16, 118, 45, (u8 *)image_buffer + 4);
    SonidoPlayNote(SONIDO_MI_6, SONIDO_BLANCA);
    // }
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

    board_t board = BoardCreate();

    DisplayWriteValue(board->display, valor);
    adc_software_trigger_enable(ADC0, ADC_INSERTED_CHANNEL);

    //rtc_demo();
    //demo_sdcard();

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
                    DisplayWriteValue(board->display, temperatura);
                    break;
                case 2:
                    DisplayWriteValue(board->display, luminosidad);
                    break;
                case 3:
                    DisplayWriteValue(board->display, EncoderReadValue());
                    break;
                }
            }
            segundos = (segundos + 1) % 500;
        }

        contador = (contador + 1) % 50;
        if (contador == 0) {

            if (KeyboardScan(board->functions)) {
                if (KeyboardChangeToPressed(board->functions, 2)) {
                    valor = (valor + 1000) % 10000;
                    variable = 0;
                    SonidoPlayNote(SONIDO_DO_6, SONIDO_NEGRA);
                }
                if (KeyboardChangeToPressed(board->functions, 3)) {
                    valor = (valor + 100) % 10000;
                    variable = 0;
                    SonidoPlayNote(SONIDO_MI_6, SONIDO_NEGRA);
                }
                if (KeyboardChangeToPressed(board->functions, 4)) {
                    valor = (valor + 10) % 10000;
                    variable = 0;
                    SonidoPlayNote(SONIDO_SOL_6, SONIDO_NEGRA);
                }
                if (KeyboardChangeToPressed(board->functions, 5)) {
                    valor = (valor + 1) % 10000;
                    variable = 0;
                    SonidoPlayNote(SONIDO_SI_6, SONIDO_NEGRA);
                }
                if (KeyboardChangeToPressed(board->functions, 0)) {
                    // valor = 9999;
                    variable = 1;
                    segundos = 0;
                    SonidoPlayMelody(LA_CUCARACHA);
                }
                if (KeyboardChangeToPressed(board->functions, 1)) {
                    // valor = 0;
                    variable = 2;
                    segundos = 0;
                    SonidoPlayMelody(QUINTA);
                }
                if (KeyboardChangeToPressed(board->functions, 6)) {
                    // valor = 0;
                    variable = 3;
                    SonidoPlayMelody(ESCALA);
                }
                if (variable == 0) {
                    DisplayWriteValue(board->display, valor);
                }
            }
        }

        if (KeyboardScan(board->numeric)) {
            tecla = KeyboardGetFirstPressed(board->numeric);
            if (tecla != 0xFF) {
                valor = tecla + 1;
                variable = 0;
            }
            DisplayWriteValue(board->display, valor);
        }

        DisplayRefresh(board->display);

        delay_1ms(1);
    }
    return 0;
}

/* === End of documentation ==================================================================== */

/** @} End of module definition for doxygen */
