# Guide to flashing the Reachy's firmware over ethercat

In order to flash the whole robot with the firmware, you need to follow the next steps:

- Make sure to have the `firmware_poulpe` repo cloned in your computer.
- Make sure to have Rust and the ethercat master installed and running in your computer. - [see the guide](https://pollen-robotics.github.io/poulpe_ethercat_controller/installation/installation_ethercat/)
- Make sure that your poulpe boards have the firmware versions of v1.5+. 


## Firmware uploading steps

1. Compile the firmware for both `orbita2d` and `orbita3d` and extract the hex files. 

   - For **orbita2d** (pvt) version run:

        ```sh
        DEFMT_LOG=off cargo build --release --features orbita2d_pvt 
        ```

        Once it compiles extract the `firmware.hex` file
        ```shell
        sh firmware_update_scripts/extract_hex.sh
        ```
        <details markdown="1">
        <summary>Example output</summary>

        ```sh
        > sh firmware_update_scripts/extract_hex.sh
        Bin file generated: firmware.bin
        ```
        </details>

        Then rename it to something more meaningful 

        ```shell
        mv firmware.hex firmware_orbita2d.hex
        ```

   - Do the same for **orbita3d**

        ```sh
        DEFMT_LOG=off cargo build --release --features orbita3d_pvt 
        ```

        Extract the `hex`

        ```shell
        sh firmware_update_scripts/extract_hex.sh
        mv firmware.hex firmware_orbita3d.hex
        ```

        Now you should have both firmware files extracted `firmware_orbita3d.hex` and `firmware_orbita2d.hex`. 

2. Check which poulpe boards do you have in your network and what are their ids.

    For example:
    ```sh
    $ ehtercat slave

    0  0:0  PREOP  +  NeckOrbita3d
    1  0:1  PREOP  +  RightShoulderOrbita2d
    2  0:2  PREOP  +  RightElbowOrbita2d
    3  0:3  PREOP  +  RightWristOrbita3d
    4  0:4  PREOP  +  LeftShoulderOrbita2d
    5  0:5  PREOP  +  LeftElbowOrbita2d
    6  0:6  PREOP  +  LeftWristOrbita3d
    ```

3. Upload the firmware to the boards

    Following the ids from the above step and the extracted files `firmware_orbita3d.hex` and `firmware_orbita2d.hex` you can uplad the firmware using the script `update_firmware`


    ```bash
    sh firmware_update_scripts/update_firmware.sh firmware_orbita3d.bin 0 # <hex file path> <poulpe id>
    ```

    <details markdown="1">
    <summary>Example output</summary>

    ```shell
    > sh firmware_update_scripts/update_firmware.sh firmware_orbita3d.bin 0
        Starting firmware update with firmware_orbita3d.bin...
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

    > In general all the poulpes having names ending with Orbita2d need orbita2d firmware and ending with Orbita3d need the orbita3d firmware. 

4. Test if you the boards have the good version of the firmware.

    Read the git hash of the firmware on the boards using the command
    
    ```shell
    ethercat upload -p0 0x200 1 -t string # read the Git hash of the slave 0 (-p0)
    ```

    
    <details markdown="1">
    <summary>Example output</summary>

    ```bash
    $ ethercat upload -p0 0x200 1 -t string
    c9ab43abdac3c5a914a9c19bfc0df489f20add2f
    ```

    </details>