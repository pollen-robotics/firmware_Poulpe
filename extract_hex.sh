#!/bin/bash
set -e

# Convert the binary to a hex file
llvm-objcopy target/thumbv7em-none-eabihf/release/firmware_poulpe firmware.bin --output-target=binary

echo "Bin file generated: firmware.bin"
