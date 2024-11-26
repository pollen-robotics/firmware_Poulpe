# EtherCAT module

This module contains the EtherCAT communication functions that are used in the project.

- `lan9252`: Contains the functions to communicate with the LAN9252 chip
- `task.rs`: Contains the task that implements the EtherCAT communication, sending and receiving data from the LAN9252 chip and updating the shared memory