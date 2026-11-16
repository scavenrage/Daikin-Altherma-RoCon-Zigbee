/*
 * zigbee.h - Ponte tra i parametri RoCon curati e la rete Zigbee/ZHA
 *
 * Mappatura scelta con l'utente il 2026-09-23 (cluster standard, nessun
 * quirk ZHA da installare):
 *   EP1  On/Off              -> 1_dhw (interruttore "1x Acqua Calda")
 *   EP2-4 Analog Output      -> target_hot_water_temperature_1/2/3 (Numero, 35-70 C)
 *   EP5  Temperature Meas.   -> tdhw1 (temperatura reale del bollitore)
 *   EP6  Analog Input        -> water_pressure (bar)
 *   EP7  Analog Input        -> error_code
 *   EP8-11 Binary Input      -> mode_of_operating == 4/1/2/3
 *        (Produzione ACS / Riscaldamento / Raffrescamento / Sbrinamento;
 *         tutti spenti = standby)
 * Nomi delle entita' in HA dall'attributo description (ZHA fallback name).
 *
 * Invio dello stato a HA con lo stesso schema di ConsoleZ906 (vedi zigbee.c).
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

/** Inizializza NVS/piattaforma Zigbee e avvia il task dello stack. Va
 *  chiamata una sola volta da app_main(), dopo can_bus_init(). */
void zigbee_start(void);

/** true se il dispositivo e' unito alla rete e il coordinator ha risposto
 *  all'ultimo keepalive (vedi zigbee.c). */
bool zigbee_is_ready(void);

/**
 * Memorizza un valore letto dal bus RoCon se "name" e' uno di quelli mappati.
 * Non chiama API Zigbee (sicura da qualunque task): l'invio a HA lo fa il
 * task Zigbee ogni 2 s, solo per i valori cambiati.
 */
void zigbee_notify_rocon_value(const char *name, float value);

/** Numero di report inviati al coordinator dall'avvio (per il log di stato). */
uint32_t zigbee_reports_sent(void);
