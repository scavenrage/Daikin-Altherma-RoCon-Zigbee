#!/usr/bin/env python3
"""
gen_rocon_table.py - genera main/rocon_table.h dalla tabella comandi del
progetto ESPHome Trunks1982/Daikin-Rotex-HPSU-CAN-Seriell
(components/daikin_rotex_can/__init__.py).

Uso:
    python gen_rocon_table.py                  scarica l'ultima versione da GitHub
    python gen_rocon_table.py <__init__.py>    usa un file locale

Regole di conversione:
  - un parametro entra in tabella solo se ha "command", "data_offset" e
    "data_size" e se il comando e' esadecimale valido (nel sorgente originale
    building_insulation contiene "OC" con la lettera O: viene scartato);
  - i parametri power_* nel sorgente usano handle_lambda invece dei campi
    standard; la formula ((data[5] << 8) | data[6]) / 0x64 viene tradotta in
    offset 5, dimensione 2, divisore 100 SOLO se la lambda e' ancora quella;
  - writable = parametri esposti come number/select/switch nel progetto originale.
"""
import datetime
import os
import re
import sys
import urllib.request

URL = ("https://raw.githubusercontent.com/Trunks1982/Daikin-Rotex-HPSU-CAN-Seriell/"
       "main/components/daikin_rotex_can/__init__.py")
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "main", "rocon_table.h")

READ_TYPES = ("sensor", "binary_sensor", "text_sensor")
WRITE_TYPES = ("number", "select", "switch")

POWER_LAMBDA = "((data[5] << 8) | data[6]) / 0x64"
LAMBDA_OVERRIDE = {n: (5, 2, 100.0) for n in ("power_dhw", "power_ehs_1", "power_ehs_2", "power_biv")}


def load_source(argv):
    if len(argv) > 1:
        with open(argv[1], encoding="utf-8") as f:
            return f.read(), argv[1]
    with urllib.request.urlopen(URL) as r:
        return r.read().decode("utf-8"), URL


def entity_blocks(src):
    """Restituisce (tipo, testo del blocco) per ogni entita' del sorgente."""
    for m in re.finditer(r'"type"\s*:\s*"(\w+)"', src):
        start = src.rfind("{", 0, m.start())
        depth = 0
        for i in range(start, len(src)):
            if src[i] == "{":
                depth += 1
            elif src[i] == "}":
                depth -= 1
                if depth == 0:
                    yield m.group(1), src[start:i + 1]
                    break


def field(block, pattern, cast=None):
    m = re.search(pattern, block)
    if not m:
        return None
    return cast(m.group(1)) if cast else m.group(1)


def main():
    src, origin = load_source(sys.argv)

    rows, skipped, seen = [], [], set()
    for etype, block in entity_blocks(src):
        name = field(block, r'"name"\s*:\s*"([^"]+)"')
        if not name:
            continue
        name = name.strip()
        if name in seen:
            continue
        seen.add(name)

        if etype not in READ_TYPES + WRITE_TYPES:
            skipped.append("%s (tipo %s)" % (name, etype))
            continue

        cmd = field(block, r'"command"\s*:\s*"([^"]+)"')
        off = field(block, r'"data_offset"\s*:\s*(\d+)', int)
        size = field(block, r'"data_size"\s*:\s*(\d+)', int)
        div = field(block, r'"divider"\s*:\s*([\d.]+)', float)

        if name in LAMBDA_OVERRIDE:
            if POWER_LAMBDA in block:
                off, size, div = LAMBDA_OVERRIDE[name]
            else:
                skipped.append("%s (handle_lambda cambiata nel sorgente, da verificare)" % name)
                continue

        if not cmd or off is None or size is None:
            skipped.append("%s (senza comando CAN)" % name)
            continue

        parts = cmd.split()
        bad = [p for p in parts if not re.fullmatch(r"[0-9A-Fa-f]{2}", p)]
        if bad or len(parts) > 7:
            skipped.append("%s (comando non valido: %s)" % (name, cmd))
            continue
        if off + size > 7:
            skipped.append("%s (offset %d + dimensione %d oltre il frame)" % (name, off, size))
            continue

        rows.append({
            "name": name,
            "cmd": parts + ["00"] * (7 - len(parts)),
            "cmd_len": len(parts),
            "off": off,
            "size": size,
            "div": div if div else 1.0,
            "signed": '"signed": True' in block,
            "writable": etype in WRITE_TYPES,
        })

    out = [
        "/*",
        " * rocon_table.h - GENERATO AUTOMATICAMENTE da tools/gen_rocon_table.py",
        " * Non modificare a mano: rigenerare con lo script.",
        " *",
        " * Fonte:     %s" % origin,
        " * Generato:  %s" % datetime.date.today().isoformat(),
        " * Parametri: %d (%d scrivibili)" % (len(rows), sum(r["writable"] for r in rows)),
        " */",
        "",
        "#pragma once",
        "",
        '#include "rocon_params.h"',
        "",
        "static const rocon_param_t s_rocon_table[] = {",
    ]
    for r in rows:
        div = "%g" % r["div"]
        if "." not in div and "e" not in div:
            div += ".0"
        out.append('    { "%s", { %s }, %d, %d, %d, %sf, %s, %s },' % (
            r["name"], ", ".join("0x" + b.upper() for b in r["cmd"]), r["cmd_len"],
            r["off"], r["size"], div,
            "true" if r["signed"] else "false",
            "true" if r["writable"] else "false"))
    out.append("};")

    with open(OUT, "w", encoding="utf-8", newline="\n") as f:
        f.write("\n".join(out) + "\n")

    print("Scritto %s" % os.path.normpath(OUT))
    print("Parametri: %d, di cui scrivibili: %d" % (len(rows), sum(r["writable"] for r in rows)))
    print("Scartati:")
    for s in skipped:
        print("  - " + s)


if __name__ == "__main__":
    main()
