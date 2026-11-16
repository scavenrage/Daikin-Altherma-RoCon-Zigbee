/*
 * can_bus.h - Accesso al bus CAN RoCon tramite il controller TWAI
 *
 * Unico modulo che conosce il driver TWAI. Oggi usa il driver "legacy"
 * (driver/twai.h), ancora presente in ESP-IDF 5.5 ma deprecato: una futura
 * migrazione alla nuova API esp_twai tocca solo can_bus.c.
 *
 * Bus RoCon: 20 kbit/s, triple sampling, ID a 11 bit.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

typedef struct {
    uint32_t id;
    uint8_t len;
    uint8_t data[8];
} can_frame_t;

typedef enum {
    CAN_BUS_STOPPED = 0,
    CAN_BUS_RUNNING,
    CAN_BUS_OFF,
    CAN_BUS_RECOVERING,
} can_bus_state_t;

typedef struct {
    can_bus_state_t state;
    uint32_t tec;           /* contatore errori di trasmissione   */
    uint32_t rec;           /* contatore errori di ricezione      */
    uint32_t rx_missed;     /* frame persi per coda RX piena       */
    uint32_t rx_overrun;    /* frame persi per overrun della FIFO  */
    uint32_t bus_errors;
    uint32_t arb_lost;
    uint32_t tx_failed;
    uint32_t recoveries;    /* ripristini da bus-off effettuati    */
} can_bus_status_t;

/** Installa e avvia il driver in modalita' normale (ricezione e trasmissione). */
esp_err_t can_bus_init(int tx_gpio, int rx_gpio);

/** Accoda un frame in trasmissione senza attendere. */
esp_err_t can_bus_send(const can_frame_t *frame);

/** Attende un frame fino a timeout_ms. ESP_ERR_TIMEOUT se non arriva nulla. */
esp_err_t can_bus_receive(can_frame_t *frame, uint32_t timeout_ms);

/** Stato del controller e contatori. */
void can_bus_get_status(can_bus_status_t *status);

/**
 * Da chiamare periodicamente: se il controller e' andato in bus-off avvia il
 * ripristino e, a ripristino completato, lo riavvia. Restituisce true se il
 * bus e' operativo.
 */
bool can_bus_maintain(void);
