/*
 * timesync.c - Allinea l'orologio della RoCon all'ora di Home Assistant.
 *
 * Quando arriva un'ora di riferimento (timesync_set_reference, dal task
 * Zigbee): rilegge dal bus data e ora della RoCon, le confronta e, se lo
 * scarto supera TIMESYNC_TOLERANCE_MIN minuti o la data e' diversa, scrive
 * i campi della data diversi e poi sempre ora e minuti (come fa il pannello).
 * Poi rilegge per verificare; fino a TIMESYNC_MAX_ATTEMPTS tentativi.
 *
 * I frame sono distanziati di TIMESYNC_FRAME_GAP_MS: nel primo test con
 * 10 ms tra un frame e l'altro uno e' andato perso (tx_failed=1).
 *
 * Tutto tranne timesync_set_reference gira nel task rocon.
 */

#include "timesync.h"

#include <stdbool.h>
#include <string.h>
#include <time.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"

#include "can_bus.h"
#include "config.h"
#include "rocon_params.h"

static const char *TAG = "timesync";

#define TIMESYNC_TOLERANCE_MIN   2
#define TIMESYNC_MAX_ATTEMPTS    3
#define TIMESYNC_FRAME_GAP_MS    100
#define TIMESYNC_READ_TIMEOUT_MS 5000
#define TIMESYNC_APPLY_WAIT_MS   3000

/* Secondi tra 1970-01-01 e 2000-01-01 (epoca Zigbee) */
#define ZIGBEE_EPOCH_OFFSET      946684800LL

enum { F_DAY, F_MONTH, F_YEAR, F_HOUR, F_MIN, F_COUNT };

static const char *const s_names[F_COUNT] = {
    "system_date_day",
    "system_date_month",
    "system_date_year",
    "system_time_hour",
    "system_time_minute",
};
#define ALL_FRESH ((1u << F_COUNT) - 1)

typedef enum {
    ST_IDLE,
    ST_READ,        /* invio delle letture dei 5 campi, uno ogni GAP */
    ST_WAIT_READ,   /* attesa delle risposte */
    ST_WRITE,       /* invio delle scritture necessarie, una ogni GAP */
    ST_WAIT_APPLY,  /* pausa prima di rileggere per verifica */
} state_t;

/* riferimento da HA (scritto dal task Zigbee) */
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static uint32_t s_new_ref_secs;
static int64_t  s_new_ref_us;
static bool     s_new_ref;

/* stato del task rocon */
static uint32_t s_ref_secs;
static int64_t  s_ref_us;
static int      s_rocon[F_COUNT];
static uint32_t s_fresh;
static state_t  s_state = ST_IDLE;
static int      s_step;
static int      s_attempt;
static int64_t  s_deadline_ms;
static int64_t  s_next_frame_ms;
static int      s_write_val[F_COUNT];   /* -1 = campo da non scrivere */

static int64_t now_ms(void)
{
    return esp_timer_get_time() / 1000;
}

void timesync_set_reference(uint32_t local_secs_since_2000)
{
    portENTER_CRITICAL(&s_mux);
    s_new_ref_secs = local_secs_since_2000;
    s_new_ref_us   = esp_timer_get_time();
    s_new_ref      = true;
    portEXIT_CRITICAL(&s_mux);
}

void timesync_on_rocon_value(const char *name, float value)
{
    for (int i = 0; i < F_COUNT; i++) {
        if (strcmp(name, s_names[i]) == 0) {
            s_rocon[i] = (int)value;
            s_fresh |= 1u << i;
            return;
        }
    }
}

/* Ora locale attuale secondo HA, in campi di calendario. */
static void ha_now(struct tm *out)
{
    int64_t secs = (int64_t)s_ref_secs + (esp_timer_get_time() - s_ref_us) / 1000000;
    time_t t = (time_t)(secs + ZIGBEE_EPOCH_OFFSET);
    gmtime_r(&t, out);   /* local_time e' gia' locale: nessun fuso da applicare */
}

/* Giorni dal 1970-01-01 per una data civile (algoritmo di H. Hinnant). */
static int64_t days_from_civil(int y, int m, int d)
{
    y -= m <= 2;
    int64_t era = (y >= 0 ? y : y - 399) / 400;
    int64_t yoe = y - era * 400;
    int64_t doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
    int64_t doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    return era * 146097 + doe - 719468;
}

static int64_t minutes_of(int y, int mo, int d, int h, int mi)
{
    return days_from_civil(y, mo, d) * 1440 + h * 60 + mi;
}

static bool send_frame(const char *name, bool write, int value)
{
    const rocon_param_t *p = rocon_param_find(name);
    if (p == NULL) {
        return false;
    }
    can_frame_t f = { .id = ROCON_REQUEST_CAN_ID };
    f.len = write ? rocon_param_build_write(p, (float)value, f.data)
                  : rocon_param_build_read(p, f.data);
    if (f.len == 0) {
        ESP_LOGE(TAG, "%s: frame non costruibile (scrivibile/curato?)", name);
        return false;
    }
    return can_bus_send(&f) == ESP_OK;
}

static void start_read(void)
{
    s_fresh = 0;
    s_step = 0;
    s_next_frame_ms = now_ms();
    s_state = ST_READ;
}

/* Confronta e decide: true se serve scrivere (e prepara s_write_val). */
static bool compare_and_plan(void)
{
    struct tm ha;
    ha_now(&ha);
    int ha_y = ha.tm_year + 1900, ha_mo = ha.tm_mon + 1, ha_d = ha.tm_mday;

    int64_t ha_min = minutes_of(ha_y, ha_mo, ha_d, ha.tm_hour, ha.tm_min);
    int64_t rc_min = minutes_of(2000 + s_rocon[F_YEAR], s_rocon[F_MONTH], s_rocon[F_DAY],
                                s_rocon[F_HOUR], s_rocon[F_MIN]);
    int64_t diff = ha_min - rc_min;
    if (diff < 0) {
        diff = -diff;
    }

    ESP_LOGI(TAG, "RoCon %02d/%02d/20%02d %02d:%02d  HA %02d/%02d/%04d %02d:%02d  scarto %lld min",
             s_rocon[F_DAY], s_rocon[F_MONTH], s_rocon[F_YEAR], s_rocon[F_HOUR], s_rocon[F_MIN],
             ha_d, ha_mo, ha_y, ha.tm_hour, ha.tm_min, (long long)diff);

    if (diff <= TIMESYNC_TOLERANCE_MIN) {
        return false;
    }

    s_write_val[F_DAY]   = (s_rocon[F_DAY]   != ha_d)          ? ha_d          : -1;
    s_write_val[F_MONTH] = (s_rocon[F_MONTH] != ha_mo)         ? ha_mo         : -1;
    s_write_val[F_YEAR]  = (s_rocon[F_YEAR]  != ha_y - 2000)   ? ha_y - 2000   : -1;
    s_write_val[F_HOUR]  = ha.tm_hour;   /* il pannello scrive sempre ora e minuti */
    s_write_val[F_MIN]   = ha.tm_min;
    return true;
}

void timesync_process(void)
{
    /* nuovo riferimento da HA: riparte da capo */
    portENTER_CRITICAL(&s_mux);
    bool fresh_ref = s_new_ref;
    if (fresh_ref) {
        s_ref_secs = s_new_ref_secs;
        s_ref_us   = s_new_ref_us;
        s_new_ref  = false;
    }
    portEXIT_CRITICAL(&s_mux);

    if (fresh_ref && s_state == ST_IDLE) {
        s_attempt = 0;
        start_read();
    }

    int64_t now = now_ms();

    switch (s_state) {
    case ST_IDLE:
        break;

    case ST_READ:
        if (now >= s_next_frame_ms) {
            send_frame(s_names[s_step], false, 0);
            s_next_frame_ms = now + TIMESYNC_FRAME_GAP_MS;
            if (++s_step >= F_COUNT) {
                s_deadline_ms = now + TIMESYNC_READ_TIMEOUT_MS;
                s_state = ST_WAIT_READ;
            }
        }
        break;

    case ST_WAIT_READ:
        if (s_fresh == ALL_FRESH) {
            if (!compare_and_plan()) {
                if (s_attempt == 0) {
                    ESP_LOGI(TAG, "Orologio RoCon gia' allineato");
                } else {
                    ESP_LOGI(TAG, "Orologio RoCon corretto e verificato");
                }
                s_state = ST_IDLE;
            } else if (s_attempt >= TIMESYNC_MAX_ATTEMPTS) {
                ESP_LOGE(TAG, "Orologio RoCon non corretto dopo %d tentativi", s_attempt);
                s_state = ST_IDLE;
            } else {
                s_attempt++;
                ESP_LOGW(TAG, "Correggo l'orologio RoCon (tentativo %d/%d)",
                         s_attempt, TIMESYNC_MAX_ATTEMPTS);
                s_step = 0;
                s_next_frame_ms = now;
                s_state = ST_WRITE;
            }
        } else if (now >= s_deadline_ms) {
            if (++s_attempt > TIMESYNC_MAX_ATTEMPTS) {
                ESP_LOGE(TAG, "Nessuna risposta dalla RoCon su data/ora, rinuncio");
                s_state = ST_IDLE;
            } else {
                ESP_LOGW(TAG, "Risposte data/ora incomplete (0x%02x), rileggo", (unsigned)s_fresh);
                start_read();
            }
        }
        break;

    case ST_WRITE:
        /* salta i campi da non scrivere */
        while (s_step < F_COUNT && s_write_val[s_step] < 0) {
            s_step++;
        }
        if (s_step >= F_COUNT) {
            s_deadline_ms = now + TIMESYNC_APPLY_WAIT_MS;
            s_state = ST_WAIT_APPLY;
        } else if (now >= s_next_frame_ms) {
            ESP_LOGI(TAG, "Scrivo %s = %d", s_names[s_step], s_write_val[s_step]);
            send_frame(s_names[s_step], true, s_write_val[s_step]);
            s_next_frame_ms = now + TIMESYNC_FRAME_GAP_MS;
            s_step++;
        }
        break;

    case ST_WAIT_APPLY:
        if (now >= s_deadline_ms) {
            start_read();   /* verifica */
        }
        break;
    }
}
