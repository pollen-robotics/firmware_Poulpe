# Utility scripts for updating the firmware over EtherCAT

This directory contains utility scripts for updating the firmware over EtherCAT.

- `extract_hex.sh`: Extracts the firmware hex file from the firmware binary file.
- `update_firmware.sh`: Updates and verifies the firmware over EtherCAT.
    - Usage: `sh update_firmware.sh <firmware_file> <slave_id>`

The folder also contains a simple example firmware binary file `firmware_blinky.bin` that once uploaded will blink the red LED on the board.

## Usage

### Extract the firmware hex file

To extract the firmware hex file from the firmware binary file, run the following command from the main directory:

```bash
sh firmware_update_scripts/extract_hex.sh
```

<details markdown="1">
<summary>Example output</summary>

```bash
> sh firmware_update_scripts/extract_hex.sh
Bin file generated: firmware.bin
```

</details>

### Update the firmware over EtherCAT

To update and verify the firmware over EtherCAT, run the following command from the main directory:

```bash
sh firmware_update_scripts/update_firmware.sh firmware.bin 0 # set the slave id (ex. 0)
```

<details markdown="1">
<summary>Example output</summary>

```bash
> sh firmware_update_scripts/update_firmware.sh firmware.bin
    Starting firmware update with firmware.bin...
    Writing firmware...
    Read 22152 bytes of FoE data.
    FoE writing finished.
    Verifying bytes received...
    Firmware size: 22152
    Bytes received: 22152
    Confirming firmware update...
    Firmware update completed successfully
```

</details>

### Blinky firmware

The `firmware_blinky.bin` is a simple example firmware binary file that once uploaded will blink the red LED on the board.

```bash
cd firmware_update_scripts
sh update_firmware.sh firmware_blinky.bin 0 # set the slave id (ex. 0)
```

Once the firmware is uploaded, the red LED on the board will start blinking. And upon the board reset, the firmware will roll back to the previous firmware version.