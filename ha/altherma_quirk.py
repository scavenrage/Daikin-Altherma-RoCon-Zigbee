"""Handmade! AlthermaBridge (Daikin Altherma via bus CAN RoCon): nomi degli interruttori virtuali.

Firmware: firmware/main/zigbee.c (endpoint 20-23). Gli endpoint 1-11 usano cluster standard
e non vengono toccati (AO/AI/BI prendono il nome dall'attributo description).
Modello: ha/daikin_vam_quirk.py del progetto Daikin-VAM-P1P2-SlaveController.
"""

from zigpy.zcl.clusters.general import OnOff

from zhaquirks.builder import QuirkBuilder


def _name_switch(builder, endpoint_id, key, name, enabled=True):
    """Dà un nome all'interruttore On/Off dell'endpoint."""
    return builder.change_entity_metadata(
        endpoint_id=endpoint_id,
        cluster_id=OnOff.cluster_id,
        new_primary=False,
        new_translation_key=key,
        new_fallback_name=name,
        new_entity_registry_enabled_default=enabled,
    )


builder = QuirkBuilder("Handmade!", "AlthermaBridge")

# Interruttori virtuali di manutenzione (abilitati):
# EP20 = BLE (riavvia in modalità aggiornamento via Bluetooth),
# EP21 = PROG (modalità programmazione / download seriale),
# EP22 = RESCRED (cancella le credenziali Zigbee),
# EP23 = LOG (log via BLE: ON attiva e resta attivo anche dopo i riavvii, OFF o BOOT breve lo spegne).
builder = _name_switch(builder, 20, "ble_update", "BLE")
builder = _name_switch(builder, 21, "programming_mode", "PROG")
builder = _name_switch(builder, 22, "zigbee_factory_reset", "RESCRED")
builder = _name_switch(builder, 23, "ble_log", "LOG")

builder.add_to_registry()
