/*
 * rocon_params.h - Tabella dei parametri del bus RoCon e codifica dei frame
 *
 * La tabella (rocon_table.h) e' generata da tools/gen_rocon_table.py a partire
 * dal progetto ESPHome Trunks1982/Daikin-Rotex-HPSU-CAN-Seriell.
 *
 * Struttura di un frame (7 byte, ID CAN a 11 bit):
 *   byte 0      tipo: 0x31 (o 0x61/0xA1) lettura, 0x30 scrittura, altri = risposta
 *   byte 1      identifica il nodo che parla
 *   byte 2..    indirizzo del parametro (spesso FA + 2 byte)
 *   data_offset valore big-endian, da dividere per "divider"
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#define ROCON_FRAME_LEN 7

typedef struct {
    const char *name;
    uint8_t cmd[ROCON_FRAME_LEN];   /* comando di lettura, completato a 7 byte */
    uint8_t cmd_len;                /* lunghezza reale del comando             */
    uint8_t data_offset;            /* primo byte del valore                   */
    uint8_t data_size;              /* 1 o 2 byte                              */
    float divider;
    bool is_signed;
    bool writable;
} rocon_param_t;

/** Numero di parametri in tabella. */
int rocon_param_count(void);

/** Parametro per indice (0 .. count-1), NULL se fuori range. */
const rocon_param_t *rocon_param_at(int index);

/** Parametro per nome, NULL se assente. */
const rocon_param_t *rocon_param_find(const char *name);

/**
 * Cerca il parametro a cui si riferisce un frame ricevuto, confrontando
 * l'indirizzo (byte 2 .. data_offset-1). I byte 0-1 sono ignorati perche'
 * dipendono dal nodo mittente. Per gli indirizzi che iniziano con FA si
 * confrontano solo FA + 2 byte: i successivi sono riempimento e nelle
 * risposte non sono sempre a zero.
 */
const rocon_param_t *rocon_param_match(const uint8_t *data, uint8_t len);

/**
 * true se il frame e' una richiesta (di lettura o di scrittura) e non una
 * risposta: in quel caso il campo valore e' vuoto e non va interpretato.
 */
bool rocon_frame_is_request(const rocon_param_t *p, const uint8_t *data);

/** Estrae il valore da un frame di risposta. false se il frame e' troppo corto. */
bool rocon_param_decode(const rocon_param_t *p, const uint8_t *data, uint8_t len, float *value);

/** Prepara il frame di lettura del parametro. Restituisce la lunghezza. */
uint8_t rocon_param_build_read(const rocon_param_t *p, uint8_t out[ROCON_FRAME_LEN]);

/**
 * true se il parametro e' nella lista ristretta dei parametri che ha senso
 * rendere scrivibili (vedi rocon_params.c): non tutti i 64 "writable" della
 * tabella, solo quelli che l'utente ha deciso di esporre.
 */
bool rocon_param_is_curated_writable(const rocon_param_t *p);

/**
 * Prepara il frame di scrittura (byte 0 = 0x30, byte 1 = 0x00, valore *
 * divider in big-endian a data_offset). Restituisce 0 (frame non preparato)
 * se il parametro non e' scrivibile in tabella o non e' nella lista
 * ristretta di rocon_param_is_curated_writable.
 */
uint8_t rocon_param_build_write(const rocon_param_t *p, float value, uint8_t out[ROCON_FRAME_LEN]);
