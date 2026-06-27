#!/bin/bash
# flash smart-lock-gate via UF2 mass storage
set -e

UF2_FILE="build/zephyr/zephyr.uf2"
BOOT_DIR="/media/$USER/NICENANO"

if [ ! -f "$UF2_FILE" ]; then
    echo "ERROR: $UF2_FILE not found. Run 'west build' first."
    exit 1
fi

if [ ! -d "$BOOT_DIR" ]; then
    echo "BOOT drive not found at $BOOT_DIR"
    echo " Double-tap the reset button on your Pro Micro nRF52840."
    echo " The red LED should pulse, and a USB drive should appear."
    read -p "Press Enter once you've done this..."
fi

if [ ! -d "$BOOT_DIR" ]; then
    echo "ERROR: Still can't find BOOT drive. Check /media/$USER/ for NICENANO."
    exit 1
fi

echo "Copying $UF2_FILE to $BOOT_DIR..."
cp "$UF2_FILE" "$BOOT_DIR/"
echo "Flashed. The board will reboot automatically."
echo " Open serial monitor: screen /dev/ttyACM0 115200"