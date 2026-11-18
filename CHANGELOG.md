# Changelog

All notable changes to this project will be documented in this file.

---

## 2026-11-18

- **Fix (important):** when the coordinator stayed unreachable for 6 consecutive keepalive
  pings (≈ 4–5 minutes) the firmware erased the Zigbee credentials. Switching off the
  coordinator for a few minutes (Home Assistant maintenance, faulty USB stick) therefore
  un-paired the bridge. Now the bridge enters deep sleep for 3 s and restarts **keeping the
  credentials**; at boot the stack retries the rejoin with the stored network data, and the
  cycle repeats until the coordinator is back. The erase after steering remains only for the
  invalid short address 0xFFFF
- Tested with the coordinator switched off for 10 minutes: the bridge rejoined by itself
  when it came back, without pairing again
- Binaries rebuilt from this source

---

## 2026-11-16

First published version: esp-zigbee-lib 2.0.5 (1.x compatibility layer), joins on any channel
(11–26), DHW control and sensors, RoCon clock sync, maintenance switches EP20–23 with the
Web Bluetooth pages, ZHA quirk.
