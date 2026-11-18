/*
 * zigbee.c - Stack Zigbee/ZHA e ponte verso i parametri RoCon curati.
 *
 * Infrastruttura di join/keepalive/watchdog/factory-reset portata quasi
 * identica dal progetto smart_switch (ESP32-H2-Custom-Zigbee 5.5), che la
 * usa in produzione da tempo. Niente OTA Zigbee: il livello di compatibilita'
 * 1.x di esp-zigbee-lib 2.x non lo supporta (vedi ZB2); aggiornamento via BLE (EP20).
 *
 * esp-zigbee-lib 2.x (senza ZBOSS) tramite il livello di compatibilita' 1.x
 * (CONFIG_ZB_SDK_1xx), come smart switch ZB2 e Daikin VAM. Tutte le chiamate
 * all'SDK avvengono nel task Zigbee (callback e allarmi): rocon_task passa i
 * valori solo in memoria (zigbee_notify_rocon_value), quindi niente lock.
 *
 * Interruttori virtuali di manutenzione EP20-23 (su smart switch/Daikin sono
 * EP10-13, qui EP10-11 sono gia' usati): 20 = aggiornamento BLE, 21 = PROG
 * (download seriale), 22 = RESCRED (factory reset Zigbee), 23 = log via BLE.
 */

#include "zigbee.h"

#include <inttypes.h>
#include <math.h>
#include <string.h>

#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_partition.h"
#include "esp_sleep.h"
#include "esp_zigbee_core.h"
#include "esp_zigbee_cluster.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "zdo/esp_zigbee_zdo_command.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "nvs_flash.h"

#include "ble_log.h"
#include "ble_update.h"
#include "can_bus.h"
#include "config.h"
#include "maint.h"
#include "led.h"
#include "rocon_params.h"
#include "timesync.h"

static const char *TAG = "zigbee";

/* ── Configurazione Zigbee ───────────────────────────────────────────── */
#define INSTALLCODE_POLICY_ENABLE    false
#define MAX_CHILDREN                 10
#define ESP_ZB_PRIMARY_CHANNEL_MASK  ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK   /* canali 11-26: si associa al coordinatore su qualunque canale (come ConsoleZ906) */

#define ESP_MANUFACTURER_NAME  "\x09""Handmade!"
#define ESP_MODEL_IDENTIFIER   "\x0E""AlthermaBridge"

#define ESP_ZB_ZR_CONFIG() {                                        \
    .esp_zb_role         = ESP_ZB_DEVICE_TYPE_ROUTER,               \
    .install_code_policy = INSTALLCODE_POLICY_ENABLE,               \
    .nwk_cfg.zczr_cfg    = { .max_children = MAX_CHILDREN },        \
}
#define ESP_ZB_DEFAULT_RADIO_CONFIG() { .radio_mode = ZB_RADIO_MODE_NATIVE }
#define ESP_ZB_DEFAULT_HOST_CONFIG()  { .host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE }

/* ── Endpoint fissi (vedi zigbee.h per la mappatura) ─────────────────── */
#define EP_DHW_SWITCH      1   /* On/Off            -> 1_dhw                          */
#define EP_DHW_SETPOINT_1  2   /* Analog Output     -> target_hot_water_temperature_1 */
#define EP_DHW_SETPOINT_2  3   /* Analog Output     -> target_hot_water_temperature_2 */
#define EP_DHW_SETPOINT_3  4   /* Analog Output     -> target_hot_water_temperature_3 */
#define EP_DHW_TEMP        5   /* Temperature Meas. -> tdhw1                          */
#define EP_WATER_PRESSURE  6   /* Analog Input      -> water_pressure (bar)           */
#define EP_ERROR_CODE      7   /* Analog Input      -> error_code                     */
#define EP_MODE_DHW        8   /* Binary Input      -> mode_of_operating == 4         */
#define EP_MODE_HEATING    9   /* Binary Input      -> mode_of_operating == 1         */
#define EP_MODE_COOLING    10  /* Binary Input      -> mode_of_operating == 2         */
#define EP_MODE_DEFROST    11  /* Binary Input      -> mode_of_operating == 3         */

/* Interruttori virtuali di manutenzione (On/Off), come EP10-13 di smart switch/Daikin */
#define BLE_SWITCH_EP      20  /* riavvia in modalita' aggiornamento BLE              */
#define PROG_SWITCH_EP     21  /* riavvia in modalita' programmazione (download)      */
#define FACTORY_SWITCH_EP  22  /* factory reset Zigbee (RESCRED)                      */
#define LOG_SWITCH_EP      23  /* log via BLE: On attiva, Off disattiva (riavvio)     */
#define IS_VIRTUAL_EP(ep)  ((ep) >= BLE_SWITCH_EP && (ep) <= LOG_SWITCH_EP)

/* Unita' BACnet (attributo engineering_units): ZHA le converte con
 * BACNET_UNITS_TO_HA_UNITS (zha/application/platforms/number/bacnet.py). */
#define BACNET_UNIT_DEGREES_CELSIUS  62
#define BACNET_UNIT_BARS             55
#define BACNET_UNIT_NO_UNITS         95  /* BACnet "no-units" (codice errore) */

/* Limiti dei setpoint ACS esposti a HA */
#define DHW_SETPOINT_MIN   35.0f
#define DHW_SETPOINT_MAX   70.0f
#define DHW_SETPOINT_STEP  0.5f

static bool s_zigbee_ready          = false;
static bool s_coordinator_confirmed = false;
static uint8_t s_ping_tick           = 0;
static uint8_t s_ping_fail_count     = 0;
static bool    s_ping_in_flight      = false;
static uint8_t s_steering_fail_count = 0;

static void zigbee_factory_reset_and_restart(void);

bool zigbee_is_ready(void)
{
    return s_zigbee_ready && s_coordinator_confirmed;
}

/* ── Scrittura verso il bus RoCon (Zigbee -> CAN) ────────────────────── */
static void send_rocon_write(const char *name, float value)
{
    const rocon_param_t *p = rocon_param_find(name);
    if (p == NULL) {
        ESP_LOGE(TAG, "Zigbee->CAN: parametro %s non trovato", name);
        return;
    }

    can_frame_t f = { .id = ROCON_REQUEST_CAN_ID };
    f.len = rocon_param_build_write(p, value, f.data);
    if (f.len == 0) {
        ESP_LOGE(TAG, "Zigbee->CAN: %s rifiutato (non scrivibile/non curato)", name);
        return;
    }

    esp_err_t err = can_bus_send(&f);
    ESP_LOGI(TAG, "Zigbee->CAN: %s = %g (%s)", name, (double)value,
             err == ESP_OK ? "OK" : esp_err_to_name(err));
}

/* ── Lettura RoCon -> attributo Zigbee ───────────────────────────────── */
/*
 * Stesso schema di ConsoleZ906, che in HA aggiorna correttamente lo stato:
 * rocon_task non chiama MAI API Zigbee, scrive solo valore + flag "dirty".
 * zb_attr_sync_cb gira nel task Zigbee ogni 2 s e spedisce i valori cambiati
 * (set_attribute_val + report one-shot al coordinator), ma solo dopo che
 * setup_reporting() ha configurato reporting e binding verso il coordinator.
 *
 * On/Off: come in ConsoleZ906 NON c'e' il cluster On/Off in ruolo CLIENT
 * (ZHA lo mostrerebbe come binary sensor "Opening") e quindi niente report
 * manuale: basta set_attribute_val, il reporting lo configura ZHA al pairing.
 */
typedef enum {
    K_ONOFF,    /* On/Off: 0/1                                        -> bool   */
    K_FLOAT,    /* present_value Analog Output/Input = valore RoCon    -> single */
    K_TEMP,     /* Temperature Measurement: gradi -> centesimi         -> int16  */
    K_MODE_IS,  /* Binary Input: present_value = (valore RoCon == arg) -> bool   */
} zb_kind_t;

typedef struct {
    const char     *name;
    uint8_t         ep;
    uint16_t        cluster;
    uint16_t        attr;
    zb_kind_t       kind;
    float           arg;
    volatile float  value;
    volatile bool   valid;
    volatile bool   dirty;
    /* Dopo un comando da HA: fino a questo tick le letture dal bus diverse
     * dal valore comandato sono considerate vecchie e ignorate (la RoCon
     * puo' rispondere ancora col valore precedente per qualche secondo). */
    volatile TickType_t hold_until;
    /* Variazione minima rispetto all'ultimo valore inviato per mandarne uno
     * nuovo a HA (0 = qualunque variazione). Filtra il rumore del sensore. */
    float           min_change;
} zb_mapping_t;

#define HA_WRITE_HOLD_MS  30000

#define MAP(n, e, c, a, k, g) \
    { .name = (n), .ep = (e), .cluster = (c), .attr = (a), .kind = (k), .arg = (g) }
/* come MAP, con soglia minima di variazione */
#define MAP_DB(n, e, c, a, k, g, db) \
    { .name = (n), .ep = (e), .cluster = (c), .attr = (a), .kind = (k), .arg = (g), .min_change = (db) }

static zb_mapping_t s_map[] = {
    MAP("1_dhw",                          EP_DHW_SWITCH,     ESP_ZB_ZCL_CLUSTER_ID_ON_OFF,           ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID,               K_ONOFF,   0),
    MAP("target_hot_water_temperature_1", EP_DHW_SETPOINT_1, ESP_ZB_ZCL_CLUSTER_ID_ANALOG_OUTPUT,    ESP_ZB_ZCL_ATTR_ANALOG_OUTPUT_PRESENT_VALUE_ID, K_FLOAT,   0),
    MAP("target_hot_water_temperature_2", EP_DHW_SETPOINT_2, ESP_ZB_ZCL_CLUSTER_ID_ANALOG_OUTPUT,    ESP_ZB_ZCL_ATTR_ANALOG_OUTPUT_PRESENT_VALUE_ID, K_FLOAT,   0),
    MAP("target_hot_water_temperature_3", EP_DHW_SETPOINT_3, ESP_ZB_ZCL_CLUSTER_ID_ANALOG_OUTPUT,    ESP_ZB_ZCL_ATTR_ANALOG_OUTPUT_PRESENT_VALUE_ID, K_FLOAT,   0),
    MAP("tdhw1",                          EP_DHW_TEMP,       ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID,      K_TEMP,    0),
    /* pressione: il sensore oscilla di +-0,003 bar, a HA interessa da 0,1 bar */
    MAP_DB("water_pressure",              EP_WATER_PRESSURE, ESP_ZB_ZCL_CLUSTER_ID_ANALOG_INPUT,     ESP_ZB_ZCL_ATTR_ANALOG_INPUT_PRESENT_VALUE_ID,  K_FLOAT,   0, 0.1f),
    MAP("error_code",                     EP_ERROR_CODE,     ESP_ZB_ZCL_CLUSTER_ID_ANALOG_INPUT,     ESP_ZB_ZCL_ATTR_ANALOG_INPUT_PRESENT_VALUE_ID,  K_FLOAT,   0),
    /* mode_of_operating: 0 standby, 1 riscaldamento, 2 raffrescamento,
     * 3 sbrinamento, 4 produzione ACS (tutti spenti = standby) */
    MAP("mode_of_operating",              EP_MODE_DHW,       ESP_ZB_ZCL_CLUSTER_ID_BINARY_INPUT,     ESP_ZB_ZCL_ATTR_BINARY_INPUT_PRESENT_VALUE_ID,  K_MODE_IS, 4),
    MAP("mode_of_operating",              EP_MODE_HEATING,   ESP_ZB_ZCL_CLUSTER_ID_BINARY_INPUT,     ESP_ZB_ZCL_ATTR_BINARY_INPUT_PRESENT_VALUE_ID,  K_MODE_IS, 1),
    MAP("mode_of_operating",              EP_MODE_COOLING,   ESP_ZB_ZCL_CLUSTER_ID_BINARY_INPUT,     ESP_ZB_ZCL_ATTR_BINARY_INPUT_PRESENT_VALUE_ID,  K_MODE_IS, 2),
    MAP("mode_of_operating",              EP_MODE_DEFROST,   ESP_ZB_ZCL_CLUSTER_ID_BINARY_INPUT,     ESP_ZB_ZCL_ATTR_BINARY_INPUT_PRESENT_VALUE_ID,  K_MODE_IS, 3),
};
#define MAP_COUNT (sizeof(s_map) / sizeof(s_map[0]))

static bool              s_reporting_ready;
static volatile uint32_t s_reports_sent;

uint32_t zigbee_reports_sent(void)
{
    return s_reports_sent;
}

/* Chiamata da rocon_task: nessuna API Zigbee, solo memoria. */
void zigbee_notify_rocon_value(const char *name, float value)
{
    if (name == NULL) {
        return;
    }
    bool changed = false;
    TickType_t now = xTaskGetTickCount();
    for (size_t i = 0; i < MAP_COUNT; i++) {
        zb_mapping_t *m = &s_map[i];
        if (strcmp(m->name, name) != 0) {
            continue;
        }
        float v;
        switch (m->kind) {
        case K_ONOFF:   v = value != 0.0f ? 1.0f : 0.0f; break;
        case K_MODE_IS: v = lroundf(value) == lroundf(m->arg) ? 1.0f : 0.0f; break;
        default:        v = value; break;
        }
        if (m->valid && m->min_change > 0.0f && fabsf(v - m->value) < m->min_change) {
            continue;   /* variazione sotto soglia: resta l'ultimo valore inviato */
        }
        if (m->hold_until != 0) {
            if (v != m->value && (int32_t)(now - m->hold_until) < 0) {
                continue;   /* lettura vecchia subito dopo un comando da HA */
            }
            m->hold_until = 0;
        }
        if (!m->valid || v != m->value) {
            m->value = v;
            m->valid = true;
            m->dirty = true;
            changed  = true;
        }
    }
    if (changed) {
        /* una riga solo quando un valore mappato cambia (non ad ogni lettura) */
        ESP_LOGI(TAG, "RoCon->HA: %s = %g", name, (double)value);
    }
}

/* ── Comandi da HA verso il bus (come ConsoleZ906) ────────────────────────
 * Il callback Zigbee non fa lavoro: aggiorna la copia interna, mette il
 * comando in coda e ritorna subito; la scrittura CAN la fa rocon_write_task. */
typedef struct {
    const char *name;
    float       value;
} rocon_write_t;

static QueueHandle_t s_write_q;

static void rocon_write_task(void *pv)
{
    rocon_write_t w;
    for (;;) {
        if (xQueueReceive(s_write_q, &w, portMAX_DELAY) == pdTRUE) {
            send_rocon_write(w.name, w.value);
        }
    }
}

/* Chiamata dal task Zigbee quando HA scrive un attributo mappato. */
static void queue_ha_command(uint8_t ep, uint16_t cluster, float value)
{
    for (size_t i = 0; i < MAP_COUNT; i++) {
        zb_mapping_t *m = &s_map[i];
        if (m->ep != ep || m->cluster != cluster) {
            continue;
        }
        /* come ConsoleZ906 (s_vol_present_value = v): la copia interna prende
         * subito il valore comandato, cosi' non rimandiamo a HA quello vecchio */
        m->value      = value;
        m->valid      = true;
        m->dirty      = false;   /* l'attributo l'ha gia' aggiornato lo stack */
        m->hold_until = xTaskGetTickCount() + pdMS_TO_TICKS(HA_WRITE_HOLD_MS);
        if (m->hold_until == 0) {
            m->hold_until = 1;
        }
        rocon_write_t w = { .name = m->name, .value = value };
        if (xQueueSend(s_write_q, &w, 0) != pdTRUE) {
            ESP_LOGW(TAG, "Coda comandi piena, %s scartato", m->name);
        }
        return;
    }
}

/* Da qui in giu' tutto gira nel task Zigbee (callback/alarm). */
static void mark_all_dirty(void)
{
    for (size_t i = 0; i < MAP_COUNT; i++) {
        if (s_map[i].valid) {
            s_map[i].dirty = true;
        }
    }
}

static void report_to_coordinator(uint8_t ep, uint16_t cluster, uint16_t attr)
{
    esp_zb_zcl_report_attr_cmd_t rep = {0};
    rep.address_mode                        = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
    rep.zcl_basic_cmd.dst_addr_u.addr_short = 0x0000;
    rep.zcl_basic_cmd.dst_endpoint          = 1;
    rep.zcl_basic_cmd.src_endpoint          = ep;
    rep.clusterID    = cluster;
    rep.attributeID  = attr;
    rep.direction    = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI;
    esp_zb_zcl_report_attr_cmd_req(&rep);
    s_reports_sent++;
}

static void push_dirty(void)
{
    for (size_t i = 0; i < MAP_COUNT; i++) {
        zb_mapping_t *m = &s_map[i];
        if (!m->valid || !m->dirty) {
            continue;
        }
        esp_zb_zcl_status_t st;
        switch (m->kind) {
        case K_ONOFF:
        case K_MODE_IS: {
            uint8_t b = m->value != 0.0f ? 1 : 0;
            st = esp_zb_zcl_set_attribute_val(m->ep, m->cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, m->attr, &b, false);
            break;
        }
        case K_TEMP: {
            int16_t t = (int16_t)lroundf(m->value * 100.0f);
            st = esp_zb_zcl_set_attribute_val(m->ep, m->cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, m->attr, &t, false);
            break;
        }
        default: {
            float f = m->value;
            st = esp_zb_zcl_set_attribute_val(m->ep, m->cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, m->attr, &f, false);
            break;
        }
        }
        if (st != ESP_ZB_ZCL_STATUS_SUCCESS) {
            ESP_LOGW(TAG, "%s EP%d: valore %g rifiutato dal cluster (status 0x%02x)",
                     m->name, m->ep, (double)m->value, st);
        }
        if (s_reporting_ready) {
            if (m->kind != K_ONOFF) {
                report_to_coordinator(m->ep, m->cluster, m->attr);
            }
            m->dirty = false;
        }
    }
}

static void zb_attr_sync_cb(uint8_t param)
{
    (void)param;
    if (zigbee_is_ready()) {
        push_dirty();
    }
    esp_zb_scheduler_alarm(zb_attr_sync_cb, 0, 2000);
}

/* ── Ora da Home Assistant (cluster Time del coordinator) ─────────────────
 * zigpy registra sull'endpoint 1 del coordinator il cluster Time e risponde
 * a local_time (0x0007) con l'ora locale, ora legale inclusa, in secondi dal
 * 2000-01-01. Lettura all'avvio e poi ogni giorno alle TIME_SYNC_HOUR:MIN
 * (subito dopo l'eventuale cambio ora legale/solare). Stesso schema di
 * ZigbeeEP::getTime() della libreria Arduino Zigbee di Espressif. */
#define TIME_SYNC_HOUR          3
#define TIME_SYNC_MINUTE        30
#define TIME_REQ_TIMEOUT_MS     10000
#define TIME_REQ_RETRY_MS       (5 * 60 * 1000)

static bool s_time_req_pending;
static bool s_time_sync_started;

static void time_request_cb(uint8_t param);

static void time_request_timeout_cb(uint8_t param)
{
    (void)param;
    if (!s_time_req_pending) {
        return;
    }
    s_time_req_pending = false;
    ESP_LOGW(TAG, "Ora da HA: nessuna risposta, riprovo tra %d min", TIME_REQ_RETRY_MS / 60000);
    esp_zb_scheduler_alarm(time_request_cb, 0, TIME_REQ_RETRY_MS);
}

static void time_request_cb(uint8_t param)
{
    (void)param;
    if (!zigbee_is_ready()) {
        esp_zb_scheduler_alarm(time_request_cb, 0, TIME_REQ_RETRY_MS);
        return;
    }
    static uint16_t attrs[] = { ESP_ZB_ZCL_ATTR_TIME_LOCAL_TIME_ID };
    esp_zb_zcl_read_attr_cmd_t req = {0};
    req.address_mode                        = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
    req.zcl_basic_cmd.dst_addr_u.addr_short = 0x0000;
    req.zcl_basic_cmd.dst_endpoint          = 1;
    req.zcl_basic_cmd.src_endpoint          = EP_DHW_SWITCH;
    req.clusterID   = ESP_ZB_ZCL_CLUSTER_ID_TIME;
    req.attr_number = 1;
    req.attr_field  = attrs;
    s_time_req_pending = true;
    esp_zb_zcl_read_attr_cmd_req(&req);
    esp_zb_scheduler_alarm(time_request_timeout_cb, 0, TIME_REQ_TIMEOUT_MS);
}

static esp_err_t handle_read_attr_resp(const esp_zb_zcl_cmd_read_attr_resp_message_t *msg)
{
    if (msg == NULL || msg->info.cluster != ESP_ZB_ZCL_CLUSTER_ID_TIME) {
        return ESP_OK;
    }
    for (const esp_zb_zcl_read_attr_resp_variable_t *v = msg->variables; v; v = v->next) {
        if (v->status != ESP_ZB_ZCL_STATUS_SUCCESS ||
            v->attribute.id != ESP_ZB_ZCL_ATTR_TIME_LOCAL_TIME_ID ||
            v->attribute.data.value == NULL ||
            (v->attribute.data.type != ESP_ZB_ZCL_ATTR_TYPE_U32 &&
             v->attribute.data.type != ESP_ZB_ZCL_ATTR_TYPE_UTC_TIME)) {
            continue;
        }
        uint32_t local_secs;
        memcpy(&local_secs, v->attribute.data.value, sizeof(local_secs));
        if (local_secs == 0 || local_secs == 0xFFFFFFFFu) {
            continue;   /* ora non valida */
        }
        s_time_req_pending = false;
        timesync_set_reference(local_secs);

        /* prossimo controllo: alle TIME_SYNC_HOUR:MIN (oggi o domani) */
        uint32_t sod    = local_secs % 86400;
        uint32_t target = TIME_SYNC_HOUR * 3600 + TIME_SYNC_MINUTE * 60;
        uint32_t delay  = (target > sod + 60) ? target - sod : target + 86400 - sod;
        ESP_LOGI(TAG, "Ora da HA ricevuta (%02lu:%02lu), prossimo controllo tra %lu h %02lu min",
                 (unsigned long)(sod / 3600), (unsigned long)(sod / 60 % 60),
                 (unsigned long)(delay / 3600), (unsigned long)(delay / 60 % 60));
        esp_zb_scheduler_alarm(time_request_cb, 0, delay * 1000);
        return ESP_OK;
    }
    return ESP_OK;
}

static void reporting_ready_cb(uint8_t param)
{
    (void)param;
    s_reporting_ready = true;
    mark_all_dirty();
    ESP_LOGI(TAG, "Reporting e binding verso il coordinator pronti");

    /* primo controllo dell'orologio RoCon poco dopo l'avvio (una sola volta:
     * dopo, la catena di alarm giornaliera si mantiene da sola) */
    if (!s_time_sync_started) {
        s_time_sync_started = true;
        esp_zb_scheduler_alarm(time_request_cb, 0, 5000);
    }
}

/* Come setup_reporting() di ConsoleZ906: configura il reporting in locale
 * e fa il bind di ogni cluster verso l'endpoint 1 del coordinator.
 * Ogni endpoint ha un solo cluster mappato, quindi una voce = un bind. */
static void setup_reporting(const esp_zb_ieee_addr_t coord_ieee)
{
    uint16_t own_addr = esp_zb_get_short_address();
    static uint8_t bool_change  = 1;
    static int16_t temp_change  = 10;    /* 0,1 gradi */
    static float   float_change = 0.1f;

    for (size_t i = 0; i < MAP_COUNT; i++) {
        const zb_mapping_t *m = &s_map[i];

        esp_zb_zcl_config_report_record_t rec = {0};
        rec.direction    = ESP_ZB_ZCL_REPORT_DIRECTION_SEND;
        rec.attributeID  = m->attr;
        rec.min_interval = 0;
        rec.max_interval = 300;
        switch (m->kind) {
        case K_ONOFF:
        case K_MODE_IS:
            rec.attrType          = ESP_ZB_ZCL_ATTR_TYPE_BOOL;
            rec.reportable_change = &bool_change;
            break;
        case K_TEMP:
            rec.attrType          = ESP_ZB_ZCL_ATTR_TYPE_S16;
            rec.reportable_change = &temp_change;
            break;
        default:
            rec.attrType          = ESP_ZB_ZCL_ATTR_TYPE_SINGLE;
            rec.reportable_change = &float_change;
            break;
        }

        esp_zb_zcl_config_report_cmd_t cfg = {0};
        cfg.address_mode                        = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
        cfg.zcl_basic_cmd.dst_addr_u.addr_short = own_addr;
        cfg.zcl_basic_cmd.src_endpoint          = m->ep;
        cfg.zcl_basic_cmd.dst_endpoint          = m->ep;
        cfg.clusterID     = m->cluster;
        cfg.record_number = 1;
        cfg.record_field  = &rec;
        esp_zb_zcl_config_report_cmd_req(&cfg);

        esp_zb_zdo_bind_req_param_t bind = {0};
        bind.cluster_id    = m->cluster;
        bind.dst_addr_mode = ESP_ZB_ZDO_BIND_DST_ADDR_MODE_64_BIT_EXTENDED;
        bind.dst_endp      = 1;
        bind.req_dst_addr  = own_addr;
        bind.src_endp      = m->ep;
        esp_zb_ieee_address_by_short(own_addr, bind.src_address);
        memcpy(bind.dst_address_u.addr_long, coord_ieee, sizeof(esp_zb_ieee_addr_t));
        esp_zb_zdo_device_bind_req(&bind, NULL, NULL);
    }

    esp_zb_scheduler_alarm(reporting_ready_cb, 0, 500);
}

/* ── Costruzione endpoint ─────────────────────────────────────────────── */
static esp_zb_attribute_list_t *create_basic_cluster(void)
{
    esp_zb_basic_cluster_cfg_t cfg = {
        .zcl_version  = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE,
        .power_source = 0x01,
    };
    esp_zb_attribute_list_t *c = esp_zb_basic_cluster_create(&cfg);
    esp_zb_basic_cluster_add_attr(c, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, ESP_MANUFACTURER_NAME);
    esp_zb_basic_cluster_add_attr(c, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID,  ESP_MODEL_IDENTIFIER);
    return c;
}

/* Basic + Identify, comuni a tutti gli endpoint */
static esp_zb_cluster_list_t *new_cluster_list(void)
{
    esp_zb_cluster_list_t *cl = esp_zb_zcl_cluster_list_create();
    esp_zb_cluster_list_add_basic_cluster(cl, create_basic_cluster(),
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_identify_cluster_cfg_t id_cfg = { .identify_time = 0 };
    esp_zb_cluster_list_add_identify_cluster(cl,
        esp_zb_identify_cluster_create(&id_cfg), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    return cl;
}

static void register_ep(esp_zb_ep_list_t *ep_list, esp_zb_cluster_list_t *cl,
                        uint8_t ep, uint16_t device_id, const char *what)
{
    esp_zb_endpoint_config_t epcfg = {
        .endpoint           = ep,
        .app_profile_id     = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id      = device_id,
        .app_device_version = 0,
    };
    esp_zb_ep_list_add_ep(ep_list, cl, epcfg);
    ESP_LOGI(TAG, "EP%d: %s", ep, what);
}

/* Aggiunge un attributo opzionale e segnala se la libreria non lo accetta */
static void add_opt_attr(esp_err_t (*add)(esp_zb_attribute_list_t *, uint16_t, void *),
                         esp_zb_attribute_list_t *attrs, uint16_t id, void *val, uint8_t ep)
{
    if (add(attrs, id, val) != ESP_OK) {
        ESP_LOGW(TAG, "EP%d: attributo 0x%04x non aggiunto", ep, id);
    }
}

/* EP1: On/Off -> 1_dhw (solo SERVER, come ConsoleZ906: niente "Opening") */
static void add_dhw_switch_endpoint(esp_zb_ep_list_t *ep_list)
{
    esp_zb_cluster_list_t *cl = new_cluster_list();
    esp_zb_on_off_cluster_cfg_t oo = { .on_off = 0 };
    esp_zb_cluster_list_add_on_off_cluster(cl,
        esp_zb_on_off_cluster_create(&oo), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    /* Time CLIENT: per leggere l'ora dal coordinator (come la libreria
     * Arduino Zigbee, che aggiunge il cluster Time sull'endpoint) */
    esp_zb_cluster_list_add_time_cluster(cl,
        esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_TIME), ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);
    register_ep(ep_list, cl, EP_DHW_SWITCH, ESP_ZB_HA_ON_OFF_OUTPUT_DEVICE_ID, "On/Off -> 1_dhw");
}

/* EP2..4: Analog Output -> entita' "Numero" in HA (come il volume di ConsoleZ906).
 * description = nome dell'entita', engineering_units = gradi C. */
static void add_dhw_setpoint_endpoint(esp_zb_ep_list_t *ep_list, uint8_t ep, char *description)
{
    esp_zb_cluster_list_t *cl = new_cluster_list();

    esp_zb_analog_output_cluster_cfg_t ao_cfg = {
        .out_of_service = false, .present_value = 0.0f, .status_flags = 0,
    };
    esp_zb_attribute_list_t *ao = esp_zb_analog_output_cluster_create(&ao_cfg);
    static float    min_v = DHW_SETPOINT_MIN, max_v = DHW_SETPOINT_MAX, step = DHW_SETPOINT_STEP;
    static uint16_t units = BACNET_UNIT_DEGREES_CELSIUS;
    add_opt_attr(esp_zb_analog_output_cluster_add_attr, ao, ESP_ZB_ZCL_ATTR_ANALOG_OUTPUT_MIN_PRESENT_VALUE_ID, &min_v, ep);
    add_opt_attr(esp_zb_analog_output_cluster_add_attr, ao, ESP_ZB_ZCL_ATTR_ANALOG_OUTPUT_MAX_PRESENT_VALUE_ID, &max_v, ep);
    add_opt_attr(esp_zb_analog_output_cluster_add_attr, ao, ESP_ZB_ZCL_ATTR_ANALOG_OUTPUT_RESOLUTION_ID,        &step,  ep);
    add_opt_attr(esp_zb_analog_output_cluster_add_attr, ao, ESP_ZB_ZCL_ATTR_ANALOG_OUTPUT_ENGINEERING_UNITS_ID, &units, ep);
    add_opt_attr(esp_zb_analog_output_cluster_add_attr, ao, ESP_ZB_ZCL_ATTR_ANALOG_OUTPUT_DESCRIPTION_ID,       description, ep);
    esp_zb_cluster_list_add_analog_output_cluster(cl, ao, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    /* CLIENT role necessario perche' report_attr_cmd_req con direction=TO_CLI
       non vada in assert (esp-idf#15962), come in ConsoleZ906 */
    esp_zb_cluster_list_add_analog_output_cluster(cl,
        esp_zb_analog_output_cluster_create(&ao_cfg), ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);

    register_ep(ep_list, cl, ep, 0x000D /* come ConsoleZ906 */, "Analog Output -> setpoint ACS");
}

/* EP5: Temperature Measurement -> tdhw1 */
static void add_dhw_temp_endpoint(esp_zb_ep_list_t *ep_list)
{
    esp_zb_cluster_list_t *cl = new_cluster_list();
    esp_zb_temperature_meas_cluster_cfg_t t_cfg = {
        .measured_value = 0,
        .min_value      = 0,
        .max_value      = 9000,
    };
    esp_zb_cluster_list_add_temperature_meas_cluster(cl,
        esp_zb_temperature_meas_cluster_create(&t_cfg), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    /* CLIENT role: vedi nota sugli Analog Output (esp-idf#15962) */
    esp_zb_cluster_list_add_temperature_meas_cluster(cl,
        esp_zb_temperature_meas_cluster_create(&t_cfg), ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);
    register_ep(ep_list, cl, EP_DHW_TEMP, ESP_ZB_HA_TEMPERATURE_SENSOR_DEVICE_ID, "Temperature Measurement -> tdhw1");
}

/* EP6/7: Analog Input -> sensore in HA (nome da description, unita' da engineering_units) */
/* ZHA crea il sensore solo se esistono description ED engineering_units
 * (AnalogInputSensor._is_supported); resolution -> decimali mostrati in HA. */
static void add_analog_input_endpoint(esp_zb_ep_list_t *ep_list, uint8_t ep,
                                      char *description, uint16_t *units, float *resolution)
{
    esp_zb_cluster_list_t *cl = new_cluster_list();
    esp_zb_analog_input_cluster_cfg_t ai_cfg = {
        .out_of_service = false, .present_value = 0.0f, .status_flags = 0,
    };
    esp_zb_attribute_list_t *ai = esp_zb_analog_input_cluster_create(&ai_cfg);
    add_opt_attr(esp_zb_analog_input_cluster_add_attr, ai, ESP_ZB_ZCL_ATTR_ANALOG_INPUT_DESCRIPTION_ID, description, ep);
    add_opt_attr(esp_zb_analog_input_cluster_add_attr, ai, ESP_ZB_ZCL_ATTR_ANALOG_INPUT_ENGINEERING_UNITS_ID, units, ep);
    add_opt_attr(esp_zb_analog_input_cluster_add_attr, ai, ESP_ZB_ZCL_ATTR_ANALOG_INPUT_RESOLUTION_ID, resolution, ep);
    esp_zb_cluster_list_add_analog_input_cluster(cl, ai, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_analog_input_cluster(cl,
        esp_zb_analog_input_cluster_create(&ai_cfg), ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);
    register_ep(ep_list, cl, ep, ESP_ZB_HA_SIMPLE_SENSOR_DEVICE_ID, "Analog Input");
}

/* EP8..11: Binary Input -> binary sensor in HA (nome da description) */
static void add_binary_input_endpoint(esp_zb_ep_list_t *ep_list, uint8_t ep, char *description)
{
    esp_zb_cluster_list_t *cl = new_cluster_list();
    esp_zb_binary_input_cluster_cfg_t bi_cfg = {
        .out_of_service = false, .status_flags = 0, .present_value = false,
    };
    esp_zb_attribute_list_t *bi = esp_zb_binary_input_cluster_create(&bi_cfg);
    add_opt_attr(esp_zb_binary_input_cluster_add_attr, bi, ESP_ZB_ZCL_ATTR_BINARY_INPUT_DESCRIPTION_ID, description, ep);
    esp_zb_cluster_list_add_binary_input_cluster(cl, bi, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_binary_input_cluster(cl,
        esp_zb_binary_input_cluster_create(&bi_cfg), ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);
    register_ep(ep_list, cl, ep, ESP_ZB_HA_SIMPLE_SENSOR_DEVICE_ID, "Binary Input");
}

/* ── Endpoint virtuali di manutenzione (come smart switch ZB2 / Daikin) ── */
static void add_virtual_switch_endpoint(esp_zb_ep_list_t *ep_list, uint8_t ep, const char *what,
                                        bool initial_on)
{
    esp_zb_cluster_list_t *cl = esp_zb_zcl_cluster_list_create();
    esp_zb_cluster_list_add_basic_cluster(cl, create_basic_cluster(),
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_on_off_cluster_cfg_t oo = { .on_off = initial_on ? 1 : 0 };
    esp_zb_cluster_list_add_on_off_cluster(cl,
        esp_zb_on_off_cluster_create(&oo), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_endpoint_config_t epcfg = {
        .endpoint           = ep,
        .app_profile_id     = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id      = ESP_ZB_HA_ON_OFF_OUTPUT_DEVICE_ID,
        .app_device_version = 0,
    };
    esp_zb_ep_list_add_ep(ep_list, cl, epcfg);
    ESP_LOGI(TAG, "EP%d: interruttore virtuale %s", ep, what);
}

static void create_endpoints(esp_zb_ep_list_t *ep_list)
{
    /* Stringhe ZCL: primo byte = lunghezza (come ESP_MANUFACTURER_NAME) */
    static char d_sp1[]   = "\x0E""Setpoint ACS 1";
    static char d_sp2[]   = "\x0E""Setpoint ACS 2";
    static char d_sp3[]   = "\x0E""Setpoint ACS 3";
    static char d_press[] = "\x0F""Pressione acqua";
    static char d_err[]   = "\x0D""Codice errore";
    static char d_dhw[]   = "\x0E""Produzione ACS";
    static char d_heat[]  = "\x0D""Riscaldamento";
    static char d_cool[]  = "\x0E""Raffrescamento";
    static char d_defr[]  = "\x0B""Sbrinamento";
    static uint16_t u_bar = BACNET_UNIT_BARS;
    static uint16_t u_none = BACNET_UNIT_NO_UNITS;
    static float res_press = 0.01f;  /* 2 decimali */
    static float res_err   = 1.0f;   /* intero */

    add_dhw_switch_endpoint(ep_list);
    add_dhw_setpoint_endpoint(ep_list, EP_DHW_SETPOINT_1, d_sp1);
    add_dhw_setpoint_endpoint(ep_list, EP_DHW_SETPOINT_2, d_sp2);
    add_dhw_setpoint_endpoint(ep_list, EP_DHW_SETPOINT_3, d_sp3);
    add_dhw_temp_endpoint(ep_list);
    add_analog_input_endpoint(ep_list, EP_WATER_PRESSURE, d_press, &u_bar,  &res_press);
    add_analog_input_endpoint(ep_list, EP_ERROR_CODE,     d_err,   &u_none, &res_err);
    add_binary_input_endpoint(ep_list, EP_MODE_DHW,     d_dhw);
    add_binary_input_endpoint(ep_list, EP_MODE_HEATING, d_heat);
    add_binary_input_endpoint(ep_list, EP_MODE_COOLING, d_cool);
    add_binary_input_endpoint(ep_list, EP_MODE_DEFROST, d_defr);

    add_virtual_switch_endpoint(ep_list, BLE_SWITCH_EP,     "aggiornamento BLE",        false);
    add_virtual_switch_endpoint(ep_list, PROG_SWITCH_EP,    "modalita' programmazione", false);
    add_virtual_switch_endpoint(ep_list, FACTORY_SWITCH_EP, "factory reset Zigbee",     false);
    add_virtual_switch_endpoint(ep_list, LOG_SWITCH_EP,     "log via BLE",              ble_log_enabled());
}

/* ── Interruttori virtuali: azioni (come Daikin) ─────────────────────── */
/* L'azione parte da un task a parte, cosi' la risposta Zigbee al comando fa in tempo a partire. */
typedef void (*maint_action_t)(void);

static void maint_action_task(void *pv)
{
    vTaskDelay(pdMS_TO_TICKS(1500));
    ((maint_action_t)pv)();
    vTaskDelete(NULL);
}

static void log_switch_task(void *pv)
{
    vTaskDelay(pdMS_TO_TICKS(1500));
    ble_log_set_enabled_and_restart((bool)(uintptr_t)pv);
    vTaskDelete(NULL);
}

/* Report singolo On/Off degli EP virtuali verso il coordinator (0x0000, endpoint 1 di ZHA):
 * senza, dopo un riavvio HA continua a mostrare l'ultimo valore (es. EP20 ON dopo
 * l'aggiornamento BLE). Chiamato dalla callback del ping (contesto Zigbee): niente lock. */
static void report_virtual_switches(void)
{
    static const uint8_t eps[] = { BLE_SWITCH_EP, PROG_SWITCH_EP, FACTORY_SWITCH_EP, LOG_SWITCH_EP };
    for (size_t i = 0; i < sizeof(eps); i++) {
        esp_zb_zcl_report_attr_cmd_t cmd = {0};
        cmd.zcl_basic_cmd.dst_addr_u.addr_short = 0x0000;
        cmd.zcl_basic_cmd.dst_endpoint = 1;
        cmd.zcl_basic_cmd.src_endpoint = eps[i];
        cmd.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
        cmd.clusterID = ESP_ZB_ZCL_CLUSTER_ID_ON_OFF;
        cmd.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI;
        cmd.manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC;
        cmd.attributeID = ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID;
        esp_err_t err = esp_zb_zcl_report_attr_cmd_req(&cmd);
        ESP_LOGI(TAG, "EP%d: report stato interruttore virtuale: %s", eps[i], esp_err_to_name(err));
    }
}

static esp_err_t handle_virtual_switch(const esp_zb_zcl_set_attr_value_message_t *msg)
{
    static bool requested = false;
    if (requested) return ESP_OK;
    const bool on = *(bool *)msg->attribute.data.value;

    /* Il log via BLE si attiva e si disattiva; gli altri reagiscono solo a ON. */
    if (msg->info.dst_endpoint == LOG_SWITCH_EP) {
        if (on == ble_log_enabled()) return ESP_OK;
        requested = true;
        ESP_LOGW(TAG, "Zigbee: log via BLE %s - riavvio tra 1.5 s", on ? "ON" : "OFF");
        xTaskCreate(log_switch_task, "log_sw", 4096, (void *)(uintptr_t)on, 5, NULL);
        return ESP_OK;
    }
    if (!on) return ESP_OK;

    maint_action_t action;
    const char *what;
    switch (msg->info.dst_endpoint) {
    case BLE_SWITCH_EP:
        action = ble_update_request_and_restart;   what = "modalita' aggiornamento BLE"; break;
    case PROG_SWITCH_EP:
        action = maint_enter_download_mode;        what = "modalita' programmazione";    break;
    case FACTORY_SWITCH_EP:
        action = zigbee_factory_reset_and_restart; what = "factory reset Zigbee";        break;
    default:
        return ESP_OK;
    }
    requested = true;
    ESP_LOGW(TAG, "Zigbee: richiesta %s - avvio tra 1.5 s", what);
    xTaskCreate(maint_action_task, "maint_act", 4096, action, 5, NULL);
    return ESP_OK;
}

/* ── Handler comandi in arrivo da Zigbee (Zigbee -> CAN) ─────────────── */
static esp_err_t handle_on_off(const esp_zb_zcl_set_attr_value_message_t *msg)
{
    if (msg->attribute.id != ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID) return ESP_OK;
    if (msg->attribute.data.type != ESP_ZB_ZCL_ATTR_TYPE_BOOL) return ESP_OK;
    if (IS_VIRTUAL_EP(msg->info.dst_endpoint)) return handle_virtual_switch(msg);
    if (msg->info.dst_endpoint != EP_DHW_SWITCH) return ESP_OK;

    bool state = *(bool *)msg->attribute.data.value;
    queue_ha_command(EP_DHW_SWITCH, ESP_ZB_ZCL_CLUSTER_ID_ON_OFF, state ? 1.0f : 0.0f);
    return ESP_OK;
}

static esp_err_t handle_analog_output(const esp_zb_zcl_set_attr_value_message_t *msg)
{
    if (msg->attribute.id != ESP_ZB_ZCL_ATTR_ANALOG_OUTPUT_PRESENT_VALUE_ID) return ESP_OK;

    float value = *(float *)msg->attribute.data.value;
    if (value < DHW_SETPOINT_MIN || value > DHW_SETPOINT_MAX) {
        return ESP_OK;   /* fuori limiti: ignorato (HA non lo permette comunque) */
    }
    queue_ha_command(msg->info.dst_endpoint, ESP_ZB_ZCL_CLUSTER_ID_ANALOG_OUTPUT, value);
    return ESP_OK;
}

static esp_err_t zb_attribute_handler(const esp_zb_zcl_set_attr_value_message_t *msg)
{
    ESP_RETURN_ON_FALSE(msg, ESP_FAIL, TAG, "msg vuoto");
    ESP_RETURN_ON_FALSE(msg->info.status == ESP_ZB_ZCL_STATUS_SUCCESS,
        ESP_ERR_INVALID_ARG, TAG, "status errore");

    if (msg->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF)
        return handle_on_off(msg);
    if (msg->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_ANALOG_OUTPUT)
        return handle_analog_output(msg);
    return ESP_OK;
}

static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t cb_id, const void *msg)
{
    switch (cb_id) {
    case ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID:
        return zb_attribute_handler((esp_zb_zcl_set_attr_value_message_t *)msg);
    case ESP_ZB_CORE_CMD_READ_ATTR_RESP_CB_ID:
        return handle_read_attr_resp((const esp_zb_zcl_cmd_read_attr_resp_message_t *)msg);
    case ESP_ZB_CORE_CMD_DEFAULT_RESP_CB_ID:
    case ESP_ZB_CORE_CMD_REPORT_CONFIG_RESP_CB_ID:
    case ESP_ZB_CORE_IDENTIFY_EFFECT_CB_ID:
        return ESP_OK;
    default:
        ESP_LOGW(TAG, "Azione non gestita: 0x%x", cb_id);
        return ESP_OK;
    }
}

/* ── Segnali Zigbee (join/steering/keepalive) ─────────────────────────
 * Portato da smart_switch, che lo usa in produzione: gestisce steering,
 * island detection tramite ping ZDO periodico al coordinator. Coordinator
 * irraggiungibile per COORDINATOR_PING_MAX_FAILS ping: deep sleep 3 s e riavvio
 * CON le credenziali. Steering fallito STEERING_MAX_FAILS volte: phy reset
 * (credenziali conservate). Le credenziali si cancellano solo su richiesta
 * (BOOT 5 s, RESCRED, pagina BLE) o per il bug noto dello stack (steering
 * "riuscito" ma indirizzo 0xFFFF). */
#define COORDINATOR_PING_TICK_MS       60000
#define COORDINATOR_PING_TICKS         5        /* 5 x 60s = 5 min tra i ping */
#define COORDINATOR_PING_MAX_FAILS     6
#define COORDINATOR_PING_TIMEOUT_MS    40000
#define STEERING_MAX_FAILS             10

static void coordinator_ping_cb(uint8_t param);
static void coordinator_ping_timeout_cb(uint8_t param);

static void zigbee_hard_reset(const char *reason)
{
    /* 2.x: credenziali nel namespace NVS "zigbee" (+ zb_fct, vecchia zb_storage):
     * cancellate da maint_erase_zigbee_credentials (stessa funzione di ZB2/Daikin). */
    ESP_LOGW(TAG, "*** ZIGBEE HARD RESET (%s) - cancello credenziali Zigbee ***", reason);
    led_set_state(LED_ERROR);
    maint_erase_zigbee_credentials(true);
    vTaskDelay(pdMS_TO_TICKS(200));
    esp_restart();
}

/* Factory reset Zigbee (GPIO9 tenuto 5 s oppure EP22): riavvio voluto, senza deep sleep. */
static void zigbee_factory_reset_and_restart(void)
{
    ESP_LOGW(TAG, "*** FACTORY RESET ZIGBEE - cancello credenziali... ***");
    led_set_state(LED_FACTORY_RESET);
    maint_mark_commanded_restart();
    maint_erase_zigbee_credentials(false);
    ESP_LOGW(TAG, "Riavvio...");
    vTaskDelay(pdMS_TO_TICKS(50));
    esp_restart();
}

static void zigbee_phy_reset(const char *reason)
{
    ESP_LOGW(TAG, "*** ZIGBEE PHY RESET (%s) - cancello phy_init ***", reason);
    led_set_state(LED_ERROR);
    const esp_partition_t *p = esp_partition_find_first(
        ESP_PARTITION_TYPE_ANY, ESP_PARTITION_SUBTYPE_ANY, "phy_init");
    if (p) {
        esp_err_t err = esp_partition_erase_range(p, 0, p->size);
        ESP_LOGW(TAG, "  'phy_init': %s", err == ESP_OK ? "OK" : esp_err_to_name(err));
    }
    vTaskDelay(pdMS_TO_TICKS(200));
    esp_restart();
}

static void bdb_start_cb(uint8_t mode_mask)
{
    ESP_RETURN_ON_FALSE(
        esp_zb_bdb_start_top_level_commissioning(mode_mask) == ESP_OK,
        , TAG, "Errore commissioning BDB");
}

static void coordinator_ping_result(esp_zb_zdp_status_t zdo_status,
                                     esp_zb_zdo_ieee_addr_rsp_t *resp,
                                     void *user_ctx)
{
    s_ping_in_flight = false;
    if (!s_zigbee_ready) return;

    if (zdo_status == ESP_ZB_ZDP_STATUS_SUCCESS) {
        s_ping_tick       = 0;
        s_ping_fail_count = 0;

        if (!s_coordinator_confirmed) {
            s_coordinator_confirmed = true;
            ESP_LOGI(TAG, "Coordinator raggiungibile - operativo");
            led_set_state(LED_OPERATIONAL);
            setup_reporting(resp->ieee_addr);
            /* Stato reale degli interruttori virtuali EP20-23 (dopo un riavvio HA mostrava l'ultimo). */
            report_virtual_switches();
        }

        /* Ogni keepalive riuscito (5 min): reinvia tutto, HA resta allineato. */
        mark_all_dirty();

        esp_zb_scheduler_alarm(coordinator_ping_cb, 0, COORDINATOR_PING_TICK_MS);
    } else {
        s_ping_fail_count++;
        ESP_LOGW(TAG, "Coordinator irraggiungibile (0x%02x) - fail %d/%d",
                 zdo_status, s_ping_fail_count, COORDINATOR_PING_MAX_FAILS);
        s_zigbee_ready          = false;
        s_coordinator_confirmed = false;
        s_reporting_ready       = false;
        s_ping_tick             = 0;
        led_set_state(LED_ZIGBEE_SEARCHING);

        if (s_ping_fail_count >= COORDINATOR_PING_MAX_FAILS) {
            /* Coordinator irraggiungibile da ~4-5 min. NON si cancellano le credenziali: il caso
             * tipico e' il coordinator spento (manutenzione di HA, chiavetta guasta) e cancellarle
             * obbligherebbe a riassociare il dispositivo. Si riavvia passando dal deep sleep di
             * 3 s (reinizializza il radio come un power cycle) mantenendo le credenziali:
             * all'avvio lo stack tenta il rejoin con i dati salvati. Se il coordinator e' ancora
             * spento il ciclo si ripete finche' non torna. Stessa correzione dello SmartSwitch
             * v2.0.1 (prima qui c'era zigbee_hard_reset(), eredita' dei problemi di ZBOSS).
             * Il risveglio da deep sleep (ESP_RST_DEEPSLEEP) non fa scattare il recupero da
             * reset non pulito in app_main. */
            ESP_LOGW(TAG, "Coordinator irraggiungibile da %d ping - deep sleep 3s, credenziali conservate",
                     s_ping_fail_count);
            vTaskDelay(pdMS_TO_TICKS(100));
            esp_sleep_enable_timer_wakeup(3ULL * 1000 * 1000);
            esp_deep_sleep_start();
        }
        esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_cb,
                               ESP_ZB_BDB_MODE_NETWORK_STEERING, 30000);
    }
}

static void coordinator_ping_timeout_cb(uint8_t param)
{
    (void)param;
    if (!s_ping_in_flight) return;
    ESP_LOGW(TAG, "Keepalive: nessuna risposta ZDO in %ds - forzo fallimento",
             COORDINATOR_PING_TIMEOUT_MS / 1000);
    coordinator_ping_result(ESP_ZB_ZDP_STATUS_TIMEOUT, NULL, NULL);
}

static void coordinator_ping_cb(uint8_t param)
{
    (void)param;
    if (!s_zigbee_ready) return;

    s_ping_tick++;
    if (s_ping_tick < COORDINATOR_PING_TICKS) {
        esp_zb_scheduler_alarm(coordinator_ping_cb, 0, COORDINATOR_PING_TICK_MS);
        return;
    }
    s_ping_tick = 0;

    esp_zb_zdo_ieee_addr_req_param_t req = {
        .dst_nwk_addr     = 0x0000,
        .addr_of_interest = 0x0000,
        .request_type     = 0,
        .start_index      = 0,
    };
    s_ping_in_flight = true;
    esp_zb_zdo_ieee_addr_req(&req, coordinator_ping_result, NULL);
    esp_zb_scheduler_alarm(coordinator_ping_timeout_cb, 0, COORDINATOR_PING_TIMEOUT_MS);
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *sig)
{
    uint32_t *p                   = sig->p_app_signal;
    esp_err_t err                 = sig->esp_err_status;
    esp_zb_app_signal_type_t type = *p;

    switch (type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Stack inizializzato");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;

    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "Avvio steering...");
            led_set_state(LED_ZIGBEE_SEARCHING);
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
        } else {
            ESP_LOGW(TAG, "Init fallita, riprovo...");
            led_set_state(LED_ERROR);
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_cb,
                                   ESP_ZB_BDB_MODE_INITIALIZATION, 1000);
        }
        break;

    case ESP_ZB_ZDO_SIGNAL_LEAVE:
        s_zigbee_ready          = false;
        s_coordinator_confirmed = false;
        s_reporting_ready       = false;
        ESP_LOGW(TAG, "Segnale LEAVE ricevuto - riprovo steering tra 30s...");
        led_set_state(LED_ZIGBEE_SEARCHING);
        esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_cb,
                               ESP_ZB_BDB_MODE_NETWORK_STEERING, 30000);
        break;

    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err == ESP_OK) {
            if (esp_zb_get_short_address() == 0xFFFF) {
                zigbee_hard_reset("addr=0xFFFF");
            }
            s_steering_fail_count = 0;
            s_zigbee_ready = true;
            ESP_LOGI(TAG, "Rete trovata: canale %d, PAN 0x%04hx, addr 0x%04hx - verifico coordinator...",
                     esp_zb_get_current_channel(), esp_zb_get_pan_id(), esp_zb_get_short_address());
            s_ping_tick = COORDINATOR_PING_TICKS - 1;
            /* una sola catena di keepalive (in 2.x due catene erano possibili, visto su ZB2) */
            esp_zb_scheduler_alarm_cancel(coordinator_ping_cb, 0);
            esp_zb_scheduler_alarm(coordinator_ping_cb, 0, 1000);
        } else {
            s_steering_fail_count++;
            ESP_LOGW(TAG, "Steering fallito (%d/%d), riprovo tra 30s...",
                     s_steering_fail_count, STEERING_MAX_FAILS);
            if (s_steering_fail_count >= STEERING_MAX_FAILS) {
                zigbee_phy_reset("steering loop");
            }
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_cb,
                                   ESP_ZB_BDB_MODE_NETWORK_STEERING, 30000);
        }
        break;

    case ESP_ZB_NWK_SIGNAL_NO_ACTIVE_LINKS_LEFT:
        if (s_zigbee_ready) {
            s_zigbee_ready          = false;
            s_coordinator_confirmed = false;
            s_reporting_ready       = false;
            ESP_LOGW(TAG, "Nessun link attivo (0x18) - forzo riconnessione");
            led_set_state(LED_ZIGBEE_SEARCHING);
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_cb,
                                   ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
        }
        break;

    case ESP_ZB_NLME_STATUS_INDICATION:
    case ESP_ZB_ZDO_DEVICE_UNAVAILABLE:
        break;

    default:
        ESP_LOGI(TAG, "Segnale: %s (0x%x) %s",
                 esp_zb_zdo_signal_to_string(type), type, esp_err_to_name(err));
        break;
    }
}

/* ── Watchdog applicativo Zigbee (come smart_switch) ─────────────────── */
#define ZB_WDT_FEED_INTERVAL_MS   60000
#define ZB_WDT_TIMEOUT_MS         180000
#define ZB_WDT_CHECK_INTERVAL_MS  30000

static volatile TickType_t s_zb_last_feed = 0;

static void zb_wdt_feed_cb(uint8_t param)
{
    (void)param;
    s_zb_last_feed = xTaskGetTickCount();
    esp_zb_scheduler_alarm(zb_wdt_feed_cb, 0, ZB_WDT_FEED_INTERVAL_MS);
}

static void zb_watchdog_task(void *pv)
{
    vTaskDelay(pdMS_TO_TICKS(ZB_WDT_FEED_INTERVAL_MS + 30000));
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(ZB_WDT_CHECK_INTERVAL_MS));
        TickType_t elapsed = xTaskGetTickCount() - s_zb_last_feed;
        if (elapsed > pdMS_TO_TICKS(ZB_WDT_TIMEOUT_MS)) {
            ESP_LOGE("zb_wdt", "Stack Zigbee non risponde da %lu s - deep sleep 3s",
                     (unsigned long)(elapsed * portTICK_PERIOD_MS / 1000));
            vTaskDelay(pdMS_TO_TICKS(100));
            esp_sleep_enable_timer_wakeup(3ULL * 1000 * 1000);
            esp_deep_sleep_start();
        }
    }
}

/* ── Factory reset Zigbee (pulsante BOOT / GPIO9, come smart_switch) ─── */
#define FACTORY_RESET_GPIO       GPIO_NUM_9
#define FACTORY_RESET_HOLD_S     5

static void factory_reset_task(void *pv)
{
    gpio_config_t btn = {
        .pin_bit_mask = (1ULL << FACTORY_RESET_GPIO),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&btn);

    vTaskDelay(pdMS_TO_TICKS(10000)); /* ignora i primi 10s (DTR/RTS al boot) */

    for (;;) {
        if (gpio_get_level(FACTORY_RESET_GPIO) != 0) {
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        ESP_LOGW(TAG, "GPIO9 premuto - tieni premuto %ds per factory reset Zigbee...",
                 FACTORY_RESET_HOLD_S);
        led_set_state(LED_FACTORY_RESET);

        bool held = true;
        int held_ticks = 0;
        for (int i = 0; i < FACTORY_RESET_HOLD_S * 10; i++) {
            vTaskDelay(pdMS_TO_TICKS(100));
            if (gpio_get_level(FACTORY_RESET_GPIO) != 0) {
                held = false;
                held_ticks = i + 1;
                break;
            }
        }

        if (!held) {
            /* Pressione breve (0.3 s - 5 s) con il log via BLE attivo: lo disattiva e riavvia
             * (come smart switch ZB2 / Daikin). */
            if (held_ticks >= 3 && ble_log_enabled()) {
                ESP_LOGW(TAG, "GPIO9: pressione breve - disattivo il log via BLE");
                ble_log_set_enabled_and_restart(false);
            }
            ESP_LOGI(TAG, "GPIO9 rilasciato - annullato");
            led_set_state(s_zigbee_ready ? LED_OPERATIONAL : LED_ZIGBEE_SEARCHING);
            continue;
        }

        zigbee_factory_reset_and_restart();
    }
}

/* ── Task Zigbee ───────────────────────────────────────────────────── */
/* Migrazione una tantum dei dati Zigbee 1.x (zb_storage, ZBOSS) -> 2.x (nvs, namespace
 * "zigbee"), come in ZB2 (provata: il modulo rientra in rete da solo) e Daikin, basata
 * sull'esempio Espressif examples/nvram_migration. Dopo nvs_flash_init, prima dell'avvio
 * dello stack. Il marcatore "zb2_mig/done" evita di ripeterla; un solo tentativo,
 * riuscito o no, senza bloccare l'avvio. */
static void zb2_migrate_legacy_nvram(void)
{
    nvs_handle_t h;
    if (nvs_open("zb2_mig", NVS_READWRITE, &h) != ESP_OK) {
        ESP_LOGE(TAG, "migrazione 1.x: nvs_open fallito, salto");
        return;
    }
    uint8_t done = 0;
    if (nvs_get_u8(h, "done", &done) == ESP_OK) {
        nvs_close(h);
        return;                                       /* gia' fatta */
    }
    esp_err_t err = esp_zigbee_nvram_migrate(NULL);   /* default: zb_storage -> nvs, IEEE = MAC */
    ESP_LOGW(TAG, "migrazione dati Zigbee 1.x -> 2.x: %s", esp_err_to_name(err));
    nvs_set_u8(h, "done", 1);
    nvs_commit(h);
    nvs_close(h);
}

static void esp_zb_task(void *pv)
{
    esp_zb_cfg_t cfg = ESP_ZB_ZR_CONFIG();
    esp_zb_init(&cfg);

    esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();
    create_endpoints(ep_list);

    esp_zb_device_register(ep_list);
    esp_zb_core_action_handler_register(zb_action_handler);
    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);

    ESP_ERROR_CHECK(esp_zb_start(false));

    s_zb_last_feed = xTaskGetTickCount();
    esp_zb_scheduler_alarm(zb_wdt_feed_cb, 0, ZB_WDT_FEED_INTERVAL_MS);
    /* Keepalive avviato solo dopo STEERING riuscito: avviarlo anche qui creava
     * due catene di allarmi in 2.x (visto su ZB2) */
    esp_zb_scheduler_alarm(zb_attr_sync_cb, 0, 2000);

    esp_zb_stack_main_loop();
}

/* nvs_flash_init, controllo del reset non pulito e modalita' aggiornamento BLE
 * sono in app_main (main.c), prima di questa funzione. */
void zigbee_start(void)
{
    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config  = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };
    zb2_migrate_legacy_nvram();
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));
    maint_serial_start();
    ble_log_start_if_enabled();

    gpio_install_isr_service(0);
    s_write_q = xQueueCreate(8, sizeof(rocon_write_t));
    xTaskCreate(rocon_write_task, "rocon_write", 3072, NULL, 4, NULL);
    xTaskCreate(factory_reset_task, "factory_rst", 2048, NULL, 1, NULL);
    xTaskCreate(zb_watchdog_task, "zb_wdt", 2048, NULL, 2, NULL);
    xTaskCreate(esp_zb_task, "Zigbee_main", 8192, NULL, 5, NULL);
}
