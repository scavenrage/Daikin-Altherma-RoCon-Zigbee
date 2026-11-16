/*
 * timesync.h - Sincronizzazione dell'orologio RoCon con l'ora di Home Assistant
 *
 * L'ora arriva dal coordinator Zigbee (cluster Time, attributo local_time:
 * secondi dal 2000-01-01 in ora locale, ora legale inclusa). Il confronto e
 * le scritture sul bus avvengono nel task rocon (vedi main.c).
 *
 * Formato di scrittura verificato il 2026-09-24 (sniffing del pannello e
 * test): 30 00 FA 01 22..26 VV 00 = giorno, mese, anno (2 cifre), ora, minuti.
 */

#pragma once

#include <stdint.h>

/** Nuova ora di riferimento da HA. Solo memoria: sicura dal task Zigbee. */
void timesync_set_reference(uint32_t local_secs_since_2000);

/** Valore letto dal bus (chiamata dal task rocon per ogni parametro decodificato). */
void timesync_on_rocon_value(const char *name, float value);

/** Macchina a stati: va chiamata a ogni giro del task rocon. */
void timesync_process(void);
