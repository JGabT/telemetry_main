# Cardinal One Telemetry

ESP32 telemetry logger for the Cardinal One vehicle.

## Features

- 5 Hz GPS logging via TinyGPS++ (10 Hz NMEA configured on the receiver).
- Daly BMS telemetry over BLE (voltage, current, SOC, cell min/max, remaining capacity).
- SSD1306 OLED dashboard with GPS, BMS, drive state, PIT/TRACK, and record count.
- Pit/Track mode controlled by the ESP32 onboard **BOOT** button.
- Automatic recording: logging starts as soon as a valid GPS fix is acquired.
- Restart failsafe: the active run path and record count are stored in NVS, so a
  reset/power cycle resumes appending to the same CSV instead of losing the run.
- Buffered writes flush every 2 seconds to limit data loss on an unexpected reset.
- Wi-Fi access point + web file manager for downloading runs.

## Button behavior

The onboard BOOT button is read on `GPIO0` (active LOW, internal pull-up).
Change `PIT_BUTTON_PIN` near the top of the sketch if your button is wired
elsewhere.

| Action | Result |
| --- | --- |
| Short press | Toggle **PIT** / **TRACK** mode |
| Hold 2 seconds while stopped | Flush the current run and start a new CSV |

PIT/TRACK is recorded in every row of the CSV, so pit data can be filtered out
later. Logging continues in PIT mode; it is only tagged, not discarded.

In PIT mode the Wi-Fi access point starts after 5 seconds parked instead of the
normal 30 seconds.

## Recording and restart failsafe

1. On boot the logger waits for a valid GPS fix (location valid, age < 2 s, at
   least 4 satellites).
2. On the first valid fix it creates `/run_XXXX.csv` and starts logging.
3. Every flush stores the active path and record count in ESP32 NVS.
4. If the ESP32 resets, the next boot re-opens that same file, appends to it,
   and continues record numbering. A partial final line from a power loss is
   terminated so new rows stay parseable.
5. A short press while running only changes PIT/TRACK. Use a 2-second hold while
   stopped (or the web UI button) to intentionally start a new run.

## Wiring

| Signal | ESP32 pin |
| --- | --- |
| GPS RX | GPIO16 |
| GPS TX | GPIO17 |
| OLED SDA | GPIO21 |
| OLED SCL | GPIO22 |
| Pit button | GPIO0 (BOOT) |
| BMS | BLE, MAC `d2:1a:01:01:c9:a7` |

## CSV format

```text
Record,Millis,PST_Date,PST_Time,Fix,Lat,Lon,Speed_kmh,Raw_Speed_kmh,Sats,State,Pit,BMS_Connected,Total_V,Current_A,SOC_Pct,Max_Cell_mV,Min_Cell_mV,RemCap_mAh
```

## Web interface

The access point is named `Cardinal-Telemetry` (password `mapua1234`). Open
`http://192.168.4.1` to download the active run or any stored CSV. The page also
has PIT/TRACK and "Start New Run" controls.

## Build

Compiled with the Arduino ESP32 core:

```sh
arduino-cli core install esp32:esp32@3.3.5
arduino-cli compile --fqbn esp32:esp32:esp32doit-devkit-v1 .
arduino-cli upload -p /dev/ttyUSB0 --fqbn esp32:esp32:esp32doit-devkit-v1 .
```

Libraries: `Adafruit GFX`, `Adafruit SSD1306`, `TinyGPSPlus`, `NimBLE-Arduino`,
`ESP Async WebServer`, `ESPAsyncTCP` (pulled in by the async web server).
