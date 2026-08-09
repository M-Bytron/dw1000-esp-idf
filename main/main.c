/*
 * DW1000 ranging application - ESP-IDF (C)
 *
 * Minimal entry point. The whole DW1000 ranging application lives in the
 * dwm1000 library (see components/dwm1000); main.c only starts it.
 *
 * Build / flash / monitor (ESP-IDF):
 *   idf.py set-target esp32
 *   idf.py -p COMx flash monitor
 */

#include "dwm1000.h"

void app_main(void)
{
    dwm1000_start();
}
