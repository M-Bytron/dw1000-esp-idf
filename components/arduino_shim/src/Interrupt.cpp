/*
 * Arduino-compat shim for ESP-IDF: external interrupts.
 *
 * Arduino runs the DW1000 interrupt handler (which does SPI transactions)
 * directly in ISR context. ESP-IDF's SPI master driver is not safely usable
 * from a raw ISR, so interrupts are deferred: the GPIO ISR only sets a
 * pending bit and notifies a high-priority FreeRTOS task, which then runs the
 * registered Arduino handler in task context.
 */
#include "Arduino.h"

#include "driver/gpio.h"
#include "soc/soc_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define SHIM_IRQ_TASK_STACK   8192
#define SHIM_IRQ_TASK_PRIORITY (configMAX_PRIORITIES - 1)

static void (*s_handlers[SOC_GPIO_PIN_COUNT])(void) = { 0 };
static volatile uint64_t s_pending = 0;
static TaskHandle_t s_task = NULL;
static bool s_serviceInstalled = false;

static void IRAM_ATTR shim_gpio_isr(void *arg)
{
    uint32_t pin = (uint32_t)(uintptr_t)arg;
    s_pending |= (1ULL << pin);
    BaseType_t woken = pdFALSE;
    vTaskNotifyGiveFromISR(s_task, &woken);
    if (woken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

static void shim_isr_task(void *arg)
{
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        uint64_t pending = s_pending;
        s_pending = 0;
        for (uint32_t pin = 0; pin < SOC_GPIO_PIN_COUNT; pin++) {
            if ((pending >> pin) & 1ULL) {
                void (*h)(void) = s_handlers[pin];
                if (h != NULL) {
                    h();
                }
            }
        }
    }
}

static void ensureIsrTask(void)
{
    if (s_task != NULL) {
        return;
    }
    if (!s_serviceInstalled) {
        esp_err_t ret = gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1);
        if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
            /* only fail if the service could not be installed at all */
            return;
        }
        s_serviceInstalled = true;
    }
    xTaskCreate(shim_isr_task, "arduino_isr", SHIM_IRQ_TASK_STACK, NULL,
                SHIM_IRQ_TASK_PRIORITY, &s_task);
}

void attachInterrupt(uint8_t pin, void (*handler)(void), int mode)
{
    if (pin >= SOC_GPIO_PIN_COUNT || handler == NULL) {
        return;
    }
    ensureIsrTask();

    s_handlers[pin] = handler;

    gpio_config_t cfg = {};
    cfg.pin_bit_mask = (1ULL << pin);
    cfg.mode = GPIO_MODE_INPUT;
    switch (mode) {
    case RISING:
        cfg.intr_type = GPIO_INTR_POSEDGE;
        break;
    case FALLING:
        cfg.intr_type = GPIO_INTR_NEGEDGE;
        break;
    case CHANGE:
        cfg.intr_type = GPIO_INTR_ANYEDGE;
        break;
    default:
        cfg.intr_type = GPIO_INTR_DISABLE;
        break;
    }
    gpio_config(&cfg);
    gpio_isr_handler_add((gpio_num_t)pin, shim_gpio_isr, (void *)(uintptr_t)pin);
}

void detachInterrupt(uint8_t pin)
{
    if (pin >= SOC_GPIO_PIN_COUNT) {
        return;
    }
    gpio_isr_handler_remove((gpio_num_t)pin);
    s_handlers[pin] = NULL;
}
