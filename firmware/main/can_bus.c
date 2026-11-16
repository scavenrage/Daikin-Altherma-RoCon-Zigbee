/*
 * can_bus.c - Implementazione su driver TWAI legacy (driver/twai.h)
 */

#include "can_bus.h"

#include <string.h>

#include "driver/twai.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

static const char *TAG = "can_bus";

static uint32_t s_recoveries;
static bool s_recovery_requested;

esp_err_t can_bus_init(int tx_gpio, int rx_gpio)
{
    twai_general_config_t g_config =
        TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)tx_gpio, (gpio_num_t)rx_gpio, TWAI_MODE_NORMAL);
    g_config.rx_queue_len = 16;   /* all'avvio il bus manda una raffica di frame */

    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_20KBITS();
    t_config.triple_sampling = true;

    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    esp_err_t err = twai_driver_install(&g_config, &t_config, &f_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "twai_driver_install: %s", esp_err_to_name(err));
        return err;
    }
    err = twai_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "twai_start: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "TWAI avviato: tx=GPIO%d rx=GPIO%d, 20 kbit/s", tx_gpio, rx_gpio);
    return ESP_OK;
}

esp_err_t can_bus_send(const can_frame_t *frame)
{
    twai_message_t msg = {0};
    msg.identifier = frame->id;
    msg.data_length_code = frame->len;
    memcpy(msg.data, frame->data, frame->len);
    return twai_transmit(&msg, 0);
}

esp_err_t can_bus_receive(can_frame_t *frame, uint32_t timeout_ms)
{
    twai_message_t msg;
    esp_err_t err = twai_receive(&msg, pdMS_TO_TICKS(timeout_ms));
    if (err != ESP_OK) {
        return err;
    }
    frame->id = msg.identifier;
    frame->len = msg.data_length_code > 8 ? 8 : msg.data_length_code;
    memcpy(frame->data, msg.data, frame->len);
    return ESP_OK;
}

void can_bus_get_status(can_bus_status_t *status)
{
    memset(status, 0, sizeof(*status));
    twai_status_info_t info;
    if (twai_get_status_info(&info) != ESP_OK) {
        status->state = CAN_BUS_STOPPED;
        return;
    }

    switch (info.state) {
    case TWAI_STATE_RUNNING:    status->state = CAN_BUS_RUNNING;    break;
    case TWAI_STATE_BUS_OFF:    status->state = CAN_BUS_OFF;        break;
    case TWAI_STATE_RECOVERING: status->state = CAN_BUS_RECOVERING; break;
    default:                    status->state = CAN_BUS_STOPPED;    break;
    }
    status->tec = info.tx_error_counter;
    status->rec = info.rx_error_counter;
    status->rx_missed = info.rx_missed_count;
    status->rx_overrun = info.rx_overrun_count;
    status->bus_errors = info.bus_error_count;
    status->arb_lost = info.arb_lost_count;
    status->tx_failed = info.tx_failed_count;
    status->recoveries = s_recoveries;
}

bool can_bus_maintain(void)
{
    twai_status_info_t info;
    if (twai_get_status_info(&info) != ESP_OK) {
        return false;
    }

    switch (info.state) {
    case TWAI_STATE_RUNNING:
        return true;

    case TWAI_STATE_BUS_OFF:
        /* Il ripristino richiede 128 sequenze di 11 bit recessivi sul bus,
         * dopodiche' il controller passa in STOPPED e va riavviato. */
        if (!s_recovery_requested) {
            ESP_LOGW(TAG, "Bus-off (tec=%lu): avvio ripristino", (unsigned long)info.tx_error_counter);
            if (twai_initiate_recovery() == ESP_OK) {
                s_recovery_requested = true;
            }
        }
        return false;

    case TWAI_STATE_STOPPED:
        if (s_recovery_requested) {
            if (twai_start() == ESP_OK) {
                s_recovery_requested = false;
                s_recoveries++;
                ESP_LOGW(TAG, "Bus ripristinato (ripristini totali: %lu)", (unsigned long)s_recoveries);
                return true;
            }
        }
        return false;

    case TWAI_STATE_RECOVERING:
    default:
        return false;
    }
}
