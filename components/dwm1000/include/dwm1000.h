/*
 * DW1000 ranging application library - ESP-IDF (C)
 *
 * This library wraps the whole DW1000 two-way ranging application (ranging
 * initiator/tag on Device A, responder/anchor on Device B) behind a single
 * entry point so main.c stays minimal. All DW1000 logic lives here and uses
 * the plain-C driver API in dw1000_c.h (see components/arduino_dw1000).
 */
#ifndef DWM1000_H
#define DWM1000_H

#ifdef __cplusplus
extern "C" {
#endif

/* Set up the console UART, probe the DW1000 and start the ranging task.
   Called once from app_main(); never returns. */
void dwm1000_start(void);

#ifdef __cplusplus
}
#endif

#endif /* DWM1000_H */
