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
