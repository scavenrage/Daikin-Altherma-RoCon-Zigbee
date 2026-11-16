/*
 * main.c - altherma_bridge: ponte tra il bus CAN RoCon della Daikin Altherma
 * (EHSX08P30BB / Rotex RoCon B1-U1) e, nelle fasi successive, Zigbee.
 *
 * Il task rocon interroga a rotazione tutti i parametri della tabella e
 * passa i valori decodificati a zigbee.c. Log: una riga di stato ogni 5 s.
 */

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#include "ble_update.h"
#include "can_bus.h"
#include "config.h"
#include "led.h"
#include "maint.h"
#include "rocon_params.h"
#include "timesync.h"
#include "zigbee.h"

static const char *TAG = "altherma";

static int64_t now_ms(void)
{
    return esp_timer_get_time() / 1000;
}

static uint32_t s_rx_frames;
static uint32_t s_rx_decoded;
static uint32_t s_tx_frames;

static void handle_frame(const can_frame_t *f)
{
    s_rx_frames++;
    const rocon_param_t *p = rocon_param_match(f->data, f->len);
    if (p == NULL || rocon_frame_is_request(p, f->data)) {
        return;
    }
    float value;
    if (rocon_param_decode(p, f->data, f->len, &value)) {
        s_rx_decoded++;
        zigbee_notify_rocon_value(p->name, value);
        timesync_on_rocon_value(p->name, value);
#if DATETIME_TEST_ENABLED
        if (strncmp(p->name, "system_", 7) == 0) {
            ESP_LOGI(TAG, "Orologio RoCon: %s = %d", p->name, (int)value);
        }
#endif
    }
}

#if DATETIME_TEST_ENABLED
/* Scrive data e ora con lo stesso formato e ordine visti dal pannello:
 * un frame per campo, pochi ms tra l'uno e l'altro. */
static void send_datetime_test(void)
{
    static const struct { const char *name; int value; } fields[] = {
        { "system_date_day",    DATETIME_TEST_DAY    },
        { "system_date_month",  DATETIME_TEST_MONTH  },
        { "system_date_year",   DATETIME_TEST_YEAR   },
        { "system_time_hour",   DATETIME_TEST_HOUR   },
        { "system_time_minute", DATETIME_TEST_MINUTE },
    };
    ESP_LOGW(TAG, "TEST DATA/ORA: giorno %d mese %d anno %d ora %d minuti %d (-1 = non scritto)",
             DATETIME_TEST_DAY, DATETIME_TEST_MONTH, DATETIME_TEST_YEAR,
             DATETIME_TEST_HOUR, DATETIME_TEST_MINUTE);
    for (size_t i = 0; i < sizeof(fields) / sizeof(fields[0]); i++) {
        if (fields[i].value < 0) {
            continue;
        }
        const rocon_param_t *p = rocon_param_find(fields[i].name);
        can_frame_t f = { .id = ROCON_REQUEST_CAN_ID };
        f.len = p ? rocon_param_build_write(p, (float)fields[i].value, f.data) : 0;
        if (f.len == 0) {
            ESP_LOGE(TAG, "TEST DATA/ORA: %s non scrivibile", fields[i].name);
            continue;
        }
        esp_err_t err = can_bus_send(&f);
        printf("TEST DATA/ORA TX %-19s", fields[i].name);
        for (int b = 0; b < f.len; b++) {
            printf(" %02X", f.data[b]);
        }
        printf("  %s\n", err == ESP_OK ? "OK" : esp_err_to_name(err));
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
#endif

#if SNIFF_MODE_ENABLED
/* Stampa un frame grezzo. Marca quelli con indice esteso FA 01 22..27
 * (system_date_day .. system_time_second) in qualunque posizione. */
static void sniff_print(const can_frame_t *f)
{
    bool date_time = false;
    for (int i = 0; i + 2 < f->len; i++) {
        if (f->data[i] == 0xFA && f->data[i + 1] == 0x01 &&
            f->data[i + 2] >= 0x22 && f->data[i + 2] <= 0x27) {
            date_time = true;
        }
    }
    const rocon_param_t *p = rocon_param_match(f->data, f->len);

    printf("SNIFF %8lld ms  ID 0x%03" PRIX32 "  [%d]", (long long)now_ms(), f->id, f->len);
    for (int i = 0; i < f->len; i++) {
        printf(" %02X", f->data[i]);
    }
    printf("%s%s%s\n", p ? "  " : "", p ? p->name : "", date_time ? "  <<< DATA/ORA" : "");
}
#endif

static void send_read_request(int index)
{
    const rocon_param_t *p = rocon_param_at(index);
    can_frame_t f = { .id = ROCON_REQUEST_CAN_ID };
    f.len = rocon_param_build_read(p, f.data);

    esp_err_t err = can_bus_send(&f);
    if (err == ESP_OK) {
        s_tx_frames++;
    } else {
        ESP_LOGW(TAG, "Lettura %s non inviata: %s", p->name, esp_err_to_name(err));
    }
}

#if WRITE_TEST_ENABLED
static void send_write_test(void)
{
    const rocon_param_t *p = rocon_param_find(WRITE_TEST_PARAM);
    if (p == NULL) {
        ESP_LOGE(TAG, "Test di scrittura: parametro %s non trovato in tabella", WRITE_TEST_PARAM);
        return;
    }

    can_frame_t f = { .id = ROCON_REQUEST_CAN_ID };
    f.len = rocon_param_build_write(p, WRITE_TEST_VALUE, f.data);
    if (f.len == 0) {
        ESP_LOGE(TAG, "Test di scrittura: %s rifiutato (non scrivibile o non in lista curata)", p->name);
        return;
    }

    printf("TEST SCRITTURA: %s = %g -> TX", p->name, (double)WRITE_TEST_VALUE);
    for (int i = 0; i < f.len; i++) {
        printf(" %02X", f.data[i]);
    }
    printf("\n");

    esp_err_t err = can_bus_send(&f);
    ESP_LOGW(TAG, "Invio %s: %s", p->name, err == ESP_OK ? "OK" : esp_err_to_name(err));
}
#endif

static void log_status(void)
{
    static const char *state_name[] = { "STOPPED", "RUNNING", "BUS-OFF", "RECOVERING" };
    can_bus_status_t s;
    can_bus_get_status(&s);
    ESP_LOGI(TAG, "CAN %s tx=%" PRIu32 " rx=%" PRIu32 " (decodificati %" PRIu32 ") err=%" PRIu32
             " tx_failed=%" PRIu32 " | Zigbee %s report=%" PRIu32,
             state_name[s.state], s_tx_frames, s_rx_frames, s_rx_decoded, s.bus_errors,
             s.tx_failed, zigbee_is_ready() ? "OK" : "non connesso", zigbee_reports_sent());
}

static void rocon_task(void *pv)
{
    const int count = rocon_param_count();
    int next = 0;
    int64_t last_poll = 0;
    int64_t last_status = now_ms();
    int64_t last_rx = now_ms();
#if WRITE_TEST_ENABLED || DATETIME_TEST_ENABLED
    int64_t task_start = now_ms();
#endif
#if WRITE_TEST_ENABLED
    bool write_test_done = false;
#endif
#if DATETIME_TEST_ENABLED
    bool datetime_test_done = false;
#endif

    for (;;) {
        can_frame_t f;
        if (can_bus_receive(&f, 50) == ESP_OK) {
#if SNIFF_MODE_ENABLED
            sniff_print(&f);
#endif
            handle_frame(&f);
            last_rx = now_ms();
        }

        int64_t now = now_ms();
        bool bus_ok = can_bus_maintain();

        /* in modalita' ascolto non interroghiamo: sul bus resta solo il
         * traffico degli altri nodi (pannello, scheda, unita' esterna) */
        if (!SNIFF_MODE_ENABLED && bus_ok && now - last_poll >= ROCON_POLL_PERIOD_MS) {
            send_read_request(next);
            next = (next + 1) % count;
            last_poll = now;
        }

        /* sincronizzazione orologio (non fa nulla finche' HA non manda l'ora) */
        if (!SNIFF_MODE_ENABLED && bus_ok) {
            timesync_process();
        }

#if WRITE_TEST_ENABLED
        if (!write_test_done && bus_ok && now - task_start >= WRITE_TEST_DELAY_MS) {
            send_write_test();
            write_test_done = true;
        }
#endif
#if DATETIME_TEST_ENABLED
        if (!datetime_test_done && bus_ok && now - task_start >= DATETIME_TEST_DELAY_MS) {
            send_datetime_test();
            datetime_test_done = true;
        }
#endif

        /* LED: riflette sia il bus CAN sia lo stato Zigbee. Un problema sul
         * bus ha priorita' (e' il piu' grave); a bus sano, "operativo" solo
         * se anche la rete Zigbee e' su e il coordinator raggiungibile,
         * altrimenti "in ricerca" (stato aggiornato anche da zigbee.c sui
         * cambi di stato, qui lo riconfermiamo ogni ciclo). */
        bool silent = (now - last_rx) > RX_SILENCE_TIMEOUT_MS;
        if (!bus_ok || silent) {
            led_set_state(LED_ERROR);
        } else {
            led_set_state(zigbee_is_ready() ? LED_OPERATIONAL : LED_ZIGBEE_SEARCHING);
        }

        if (now - last_status >= STATUS_LOG_PERIOD_MS) {
            log_status();
            if (silent) {
                ESP_LOGW(TAG, "Nessun frame ricevuto da %lld s", (long long)((now - last_rx) / 1000));
            }
            last_status = now;
        }
    }
}

void app_main(void)
{
    /* NVS prima del controllo sul reset: serve a leggere il segno di riavvio voluto
     * (stesso schema di Daikin VAM; prima era in zigbee_start). */
    ESP_ERROR_CHECK(nvs_flash_init());

    {
        /* Reset non pulito (brownout/panic/wdt): il radio 802.15.4 puo' restare
         * in stato degradato. Deep sleep 3s = power cycle equivalente. */
        esp_reset_reason_t rr = esp_reset_reason();
        bool commanded = maint_take_commanded_restart();   /* letto e azzerato sempre */
        if (rr != ESP_RST_POWERON && rr != ESP_RST_DEEPSLEEP && rr != ESP_RST_EXT) {
            if (commanded) {
                /* Riavvio deciso dall'utente (BLE, log BLE, factory reset, seriale): avvio normale. */
                ESP_LOGI(TAG, "Riavvio comandato (reason=%d) - avvio normale, niente deep sleep", (int)rr);
            } else {
                ESP_LOGW(TAG, "Reset non pulito (reason=%d) - deep sleep 3s per reset radio", (int)rr);
                esp_sleep_enable_timer_wakeup(3ULL * 1000 * 1000);
                esp_deep_sleep_start();
            }
        }
    }

    /* Modalita' aggiornamento BLE: niente Zigbee, niente bus CAN. Non ritorna. */
    if (ble_update_pending()) {
        ble_update_run();
    }

    led_init();
    led_set_state(LED_BOOT);

    ESP_LOGI(TAG, "altherma_bridge - %d parametri in tabella", rocon_param_count());
#if SNIFF_MODE_ENABLED
    ESP_LOGW(TAG, "MODALITA' ASCOLTO ATTIVA: nessuna interrogazione, stampo ogni frame del bus");
#endif

    if (can_bus_init(CAN_TX_GPIO, CAN_RX_GPIO) != ESP_OK) {
        led_set_state(LED_ERROR);
        return;
    }

    zigbee_start();

    xTaskCreate(rocon_task, "rocon_task", 4096, NULL, 5, NULL);
}
