/*
 * Arduino-compat shim for ESP-IDF: GPIO and time functions.
 */
#include "Arduino.h"

#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void pinMode(uint8_t pin, uint8_t mode)
{
    gpio_config_t cfg = {};
    cfg.pin_bit_mask = (1ULL << pin);
    switch (mode) {
    case INPUT:
        cfg.mode = GPIO_MODE_INPUT;
        break;
    case INPUT_PULLUP:
        cfg.mode = GPIO_MODE_INPUT;
        cfg.pull_up_en = GPIO_PULLUP_ENABLE;
        break;
    case INPUT_PULLDOWN:
        cfg.mode = GPIO_MODE_INPUT;
        cfg.pull_down_en = GPIO_PULLDOWN_ENABLE;
        break;
    case OUTPUT:
        cfg.mode = GPIO_MODE_OUTPUT;
        break;
    case OUTPUT_OPEN_DRAIN:
        cfg.mode = GPIO_MODE_OUTPUT_OD;
        break;
    default:
        cfg.mode = GPIO_MODE_INPUT;
        break;
    }
    gpio_config(&cfg);
}

void digitalWrite(uint8_t pin, uint8_t val)
{
    gpio_set_level((gpio_num_t)pin, val ? 1 : 0);
}

int digitalRead(uint8_t pin)
{
    return gpio_get_level((gpio_num_t)pin) ? HIGH : LOW;
}

void delay(uint32_t ms)
{
    if (ms > 0) {
        vTaskDelay(pdMS_TO_TICKS(ms));
    }
}

void delayMicroseconds(uint32_t us)
{
    esp_rom_delay_us(us);
}

uint32_t millis(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

uint32_t micros(void)
{
    return (uint32_t)esp_timer_get_time();
}

void interrupts(void)
{
    /*
     * Arduino's interrupts()/noInterrupts() are only used by DW1000Mac.cpp,
     * which is not part of this port. There is no safe task-level global
     * interrupt toggle in ESP-IDF, so these are intentionally no-ops.
     */
}

void noInterrupts(void)
{
    /* see interrupts() */
}
