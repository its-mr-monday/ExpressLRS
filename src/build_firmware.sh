#!/bin/bash

# This script produces a firmware binary to be flashed via the Wi-Fi interface on the Radio Master Radio Micro 2.2 (ESP32-S2 based).

set -e

PLATFORMIO_HOME_DIR=.piohome
pio run -e Unified_ESP32_2400_TX_via_WIFI -j12