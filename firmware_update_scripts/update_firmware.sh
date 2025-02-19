#!/bin/bash

# Check if firmware file provided
if [ $# -ne 2 ]; then
    echo "Usage: $0 <firmware.bin> <slave_id>"
    exit 1
fi

FIRMWARE=$1
SLAVE_ID=$2

# Check if firmware exists
if [ ! -f "$FIRMWARE" ]; then
    echo "Error: Firmware file $FIRMWARE not found"
    exit 1
fi

echo "Starting firmware update for slave $SLAVE_ID with $FIRMWARE..."

# Write firmware
echo "Writing firmware..."
ethercat foe_write -p$SLAVE_ID "$FIRMWARE" --verbose
if [ $? -ne 0 ]; then
    echo "Error: Failed to write firmware"
    exit 1
fi

# Read back bytes received
echo "Verifying bytes received..."
BYTES_RECEIVED=$(ethercat upload -p$SLAVE_ID 0x100 1 -t uint32 | awk '{print $2}')
if [ -z "$BYTES_RECEIVED" ]; then
    echo "Error: Failed to read bytes received"
    exit 1
fi

# Get firmware size
FIRMWARE_SIZE=$(stat -c %s "$FIRMWARE")

echo "Firmware size: $FIRMWARE_SIZE"
echo "Bytes received: $BYTES_RECEIVED"

# Verify size matches
if [ "$BYTES_RECEIVED" -ne "$FIRMWARE_SIZE" ]; then
    echo "Error: Size mismatch"
    exit 1
fi

# Confirm update
echo "Confirming firmware update..."
ethercat download -p$SLAVE_ID 0x100 1 -t uint32 "$BYTES_RECEIVED"
if [ $? -ne 0 ]; then
    echo "Error: Failed to confirm update"
    exit 1
fi

echo "Firmware update completed successfully"