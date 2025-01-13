#!/bin/bash
set -e

# Convert the binary to a hex file
llvm-objcopy target/thumbv7em-none-eabihf/release/firmware_poulpe firmware.hex --output-target=ihex

echo "Hex file generated: firmware.hex"
