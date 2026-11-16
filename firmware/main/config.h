/*
 * config.h - Costanti del firmware altherma_bridge
 *
 * Pin allineati allo schema in hardware/ (U3 ESP32-H2-MINI-1):
 *   IO4 -> CTX (U4 pin 1, D)     IO5 -> CRX (U4 pin 4, R)
 *   IO22 -> LED di stato (vedi led.h)
 */

#pragma once

/* ------------------------------------------------------------------ */
/*  Bus CAN                                                            */
/* ------------------------------------------------------------------ */
#define CAN_TX_GPIO              4
#define CAN_RX_GPIO              5

/* ID CAN delle nostre richieste: lo stesso usato dal progetto ESPHome
 * Trunks1982/Daikin-Rotex-HPSU-CAN-Seriell (TEntity::sendGet). */
#define ROCON_REQUEST_CAN_ID     0x680

/* Intervallo minimo tra due richieste, come nel progetto di riferimento. */
#define ROCON_POLL_PERIOD_MS     250

/* ------------------------------------------------------------------ */
/*  Diagnostica                                                        */
/* ------------------------------------------------------------------ */
#define STATUS_LOG_PERIOD_MS     5000

/* Senza frame ricevuti per questo tempo il bus e' considerato muto
 * (pompa spenta, cavo scollegato): il LED passa a errore. */
#define RX_SILENCE_TIMEOUT_MS    10000

/* ------------------------------------------------------------------ */
/*  Test di scrittura (singolo, va disabilitato dopo l'uso)           */
/* ------------------------------------------------------------------ */
/* Se a 1, WRITE_TEST_DELAY_MS dopo l'avvio invia UNA scrittura sul
 * parametro WRITE_TEST_PARAM (deve essere in rocon_param_is_curated_writable).
 * Rimettere a 0 dopo il test: altrimenti scrive di nuovo a ogni riavvio. */
#define WRITE_TEST_ENABLED       0
#define WRITE_TEST_PARAM         "1_dhw"
#define WRITE_TEST_VALUE         1.0f
#define WRITE_TEST_DELAY_MS      10000

/* ------------------------------------------------------------------ */
/*  Test scrittura data/ora (singolo, va disabilitato dopo l'uso)     */
/* ------------------------------------------------------------------ */
/* Se a 1, DATETIME_TEST_DELAY_MS dopo l'avvio scrive UNA volta data e ora
 * sulla RoCon (data: giorno, mese, anno; poi ora e minuti, come il pannello)
 * e stampa ogni lettura di data/ora che arriva dal bus, per verificare che
 * la RoCon le abbia prese. Anno a 2 cifre (27 = 2027). Un campo a -1 non
 * viene scritto. Rimettere a 0 dopo. */
#define DATETIME_TEST_ENABLED    0
#define DATETIME_TEST_DAY        -1
#define DATETIME_TEST_MONTH      -1
#define DATETIME_TEST_YEAR       27
#define DATETIME_TEST_HOUR       -1
#define DATETIME_TEST_MINUTE     -1
#define DATETIME_TEST_DELAY_MS   10000

/* ------------------------------------------------------------------ */
/*  Modalita' ascolto bus (diagnostica, va disabilitata dopo l'uso)   */
/* ------------------------------------------------------------------ */
/* Se a 1: nessuna interrogazione della RoCon, stampa OGNI frame che passa
 * sul bus (tempo, ID, dati, nome del parametro se noto). Serve a scoprire
 * i comandi che il pannello RoCon manda alla scheda, es. cambio data/ora.
 * I frame che toccano gli indici data/ora noti (FA 01 22..27) sono marcati
 * con "<<< DATA/ORA". Zigbee resta attivo. Rimettere a 0 dopo l'uso. */
#define SNIFF_MODE_ENABLED       0
