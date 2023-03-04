MCP_CAN_lib ported to Native ESP-LIB for ESP32.
Allows use of MCP2515 CAN drivers on native ESP-LIB platform.

Ported from MCP_CAN_lib Library for Arduino by Cory J Fowler, v1.5

This is a Non-Arduino port using native ESP-LIB calls only.
The original Arduino MCP_CAN_lib API and functionality is preserved as well as possible.

An example program is included which configures 2x MCP2515 chips on an SPI bus and transfers data from one to the other over a loopback CAN bus.

Refer to the original MCP_CAN_lib for additional API example programs.

This library is compatible with any ESP32 board that uses the MCP2515 or MCP25625 CAN protocol controller attached by SPI bus.

===============

*Happy Coding!*
