/*
 * rocon_params.c - Ricerca, decodifica e costruzione dei frame RoCon
 */

#include "rocon_params.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#include "rocon_table.h"

#define ROCON_WRITE_OPCODE 0x30

/*
 * Lista ristretta dei parametri che ha senso rendere scrivibili (decisa con
 * l'utente il 2026-09-23): il tasto "1x Acqua Calda" e i tre setpoint ACS.
 * Tutti gli altri parametri "writable" in tabella restano leggibili soltanto
 * finche' non si decide di aggiungerli qui.
 */
static const char *const s_curated_writable[] = {
    "1_dhw",
    "target_hot_water_temperature_1",
    "target_hot_water_temperature_2",
    "target_hot_water_temperature_3",
    /* orologio: comando sniffato dal pannello il 2026-09-24
     * (30 00 FA 01 25 HH 00, poi 30 00 FA 01 26 MM 00); data da verificare */
    "system_date_day",
    "system_date_month",
    "system_date_year",
    "system_time_hour",
    "system_time_minute",
};
#define CURATED_WRITABLE_COUNT (sizeof(s_curated_writable) / sizeof(s_curated_writable[0]))

int rocon_param_count(void)
{
    return (int)(sizeof(s_rocon_table) / sizeof(s_rocon_table[0]));
}

const rocon_param_t *rocon_param_at(int index)
{
    if (index < 0 || index >= rocon_param_count()) {
        return NULL;
    }
    return &s_rocon_table[index];
}

const rocon_param_t *rocon_param_find(const char *name)
{
    for (int i = 0; i < rocon_param_count(); i++) {
        if (strcmp(s_rocon_table[i].name, name) == 0) {
            return &s_rocon_table[i];
        }
    }
    return NULL;
}

const rocon_param_t *rocon_param_match(const uint8_t *data, uint8_t len)
{
    for (int i = 0; i < rocon_param_count(); i++) {
        const rocon_param_t *p = &s_rocon_table[i];
        int addr_len = p->data_offset - 2;
        if (p->cmd[2] == 0xFA && addr_len > 3) {
            addr_len = 3;
        }
        if (addr_len < 1 || p->data_offset + p->data_size > len) {
            continue;
        }
        if (memcmp(&data[2], &p->cmd[2], addr_len) == 0) {
            return p;
        }
    }
    return NULL;
}

bool rocon_frame_is_request(const rocon_param_t *p, const uint8_t *data)
{
    return data[0] == p->cmd[0] || data[0] == ROCON_WRITE_OPCODE;
}

bool rocon_param_decode(const rocon_param_t *p, const uint8_t *data, uint8_t len, float *value)
{
    if (p->data_offset + p->data_size > len) {
        return false;
    }

    uint32_t raw = 0;
    for (int i = 0; i < p->data_size; i++) {
        raw = (raw << 8) | data[p->data_offset + i];
    }

    int32_t v = (int32_t)raw;
    if (p->is_signed) {
        int bits = p->data_size * 8;
        if (raw & (1u << (bits - 1))) {
            v -= (int32_t)(1u << bits);
        }
    }

    *value = (float)v / p->divider;
    return true;
}

uint8_t rocon_param_build_read(const rocon_param_t *p, uint8_t out[ROCON_FRAME_LEN])
{
    memcpy(out, p->cmd, p->cmd_len);
    return p->cmd_len;
}

bool rocon_param_is_curated_writable(const rocon_param_t *p)
{
    if (p == NULL) {
        return false;
    }
    for (size_t i = 0; i < CURATED_WRITABLE_COUNT; i++) {
        if (strcmp(p->name, s_curated_writable[i]) == 0) {
            return true;
        }
    }
    return false;
}

uint8_t rocon_param_build_write(const rocon_param_t *p, float value, uint8_t out[ROCON_FRAME_LEN])
{
    if (!p->writable || !rocon_param_is_curated_writable(p)) {
        return 0;
    }
    if (p->data_offset + p->data_size > ROCON_FRAME_LEN) {
        return 0;
    }

    memset(out, 0, ROCON_FRAME_LEN);
    memcpy(out, p->cmd, p->cmd_len);
    out[0] = ROCON_WRITE_OPCODE;
    out[1] = 0x00;

    int32_t raw = (int32_t)lroundf(value * p->divider);
    for (int i = p->data_size - 1; i >= 0; i--) {
        out[p->data_offset + i] = (uint8_t)(raw & 0xFF);
        raw >>= 8;
    }
    return p->cmd_len;
}
