# Dynamixel-IDF

## purpose

ESP32 processors should make excellent controllers for Dynamixel servos.

Each ESP32 has three UARTs, but one is often used for debugging and data upload.

Timing constraints for RS-486 and servo control often require careful code.

## Dynamixel C library

The dynamixel C library has a wide variety of flaws. Beyond being strictly non-functional
in its Linux port as of 1/5/21, it allocates and frees memory frequently.

Thus, let us create a C library which is efficient and can be used with the ESP-IDF
RTOS for high efficieny.

## Hey! Where are the pins configured???

This library uses 3 pins per output, because most RS486 chipsets require an enable pin.

The pins are configured in:
```
dynamixel-idf/components/DynamixelSDK-c/include/port_handler_esp_idf.h
```
## Hey! What kind of chips are you using, and what data rates are you getting?

The goal is fast response time. We have to level-shift between 5v and 3.3v quickly, experimentation shows the
Adafruit implementation of the BSS138 to be sufficient for 4Mhz transport. https://www.adafruit.com/product/757 - note that
in this implementation, the "OE" output enable must be left to float.

For the 486 converter, we are using the TI 486 chip: https://www.ti.com/lit/ds/symlink/sn65hvd485e.pdf

The more expensive TI chip - https://www.ti.com/lit/ds/symlink/sn75lbc176.pdf - did not function, so we simply swapped it out.
