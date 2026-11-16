/*
 * led.c - Task FreeRTOS per la gestione del LED di stato (GPIO22)
 *
 * Ripreso dal firmware smart_switch. Il task gira in loop e implementa il
 * pattern corrispondente allo stato corrente; il resto del codice chiama
 * solo led_set_state().
 */

#include "led.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static volatile led_state_t s_state = LED_BOOT;

static inline void led_on(void)
{
    gpio_set_level(LED_GPIO, 1);
}

static inline void led_off(void)
{
    gpio_set_level(LED_GPIO, 0);
}

/* Singolo blink: accende on_ms, spegne off_ms */
static void blink(uint32_t on_ms, uint32_t off_ms)
{
    led_on();
    vTaskDelay(pdMS_TO_TICKS(on_ms));
    led_off();
    vTaskDelay(pdMS_TO_TICKS(off_ms));
}

static void led_task(void *pv)
{
    for (;;) {
        switch (s_state) {

        case LED_BOOT:
            blink(100, 100);
            break;

        case LED_ZIGBEE_SEARCHING:
            blink(100, 100);
            blink(100, 1700);
            break;

        case LED_OPERATIONAL:
            blink(100, 4900);
            break;

        case LED_OTA_IN_PROGRESS:
            blink(500, 500);
            break;

        case LED_ERROR:
            blink(100, 100);
            blink(100, 100);
            blink(100, 2600);
            break;

        case LED_FACTORY_RESET:
            led_on();
            vTaskDelay(pdMS_TO_TICKS(100));
            break;

        default:
            vTaskDelay(pdMS_TO_TICKS(100));
            break;
        }
    }
}

void led_init(void)
{
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << LED_GPIO),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);
    led_off();

    xTaskCreate(led_task, "led_task", 2048, NULL, 2, NULL);
}

void led_set_state(led_state_t state)
{
    s_state = state;
}
