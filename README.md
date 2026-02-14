# TwaiTaskBasedLibraryWROOM32

FreeRTOS task-based TWAI (CAN) communication wrapper for ESP32-WROOM-32.

## Overview

This library wraps the ESP32-WROOM-32 native TWAI (Two-Wire Automotive Interface) driver with FreeRTOS tasks to provide asynchronous, non-blocking CAN bus communication.

**Key Features:**
- Asynchronous CAN communication via FreeRTOS tasks
- Callback-based RX and TX notifications
- TX message queueing (16-message deep)
- Support for standard CAN 2.0 messaging
- Configurable baudrates: 125k, 250k, 500k, 1000k bps
- Accept-all filter configuration

## Hardware Requirements

- ESP32-WROOM-32 microcontroller
- CAN transceiver module (e.g., MCP2551, TJA1050)
- Two GPIO pins for TWAI TX and RX

## Installation

Add this library to your PlatformIO `platformio.ini`:

```ini
lib_deps =
    git@github.com:trailcurrentoss/TwaiTaskBasedLibraryWROOM32.git#0.0.1
```

## Usage Example

```cpp
#include <Arduino.h>
#include <TwaiTaskBased.h>

// RX callback - called when a message is received
void onCanRx(const twai_message_t &msg) {
  Serial.printf("RX - ID: 0x%X, DLC: %d\n", msg.identifier, msg.data_length_code);
  for (int i = 0; i < msg.data_length_code; i++) {
    Serial.printf("  Data[%d] = 0x%02X\n", i, msg.data[i]);
  }
}

// TX callback - called when a message is transmitted
void onCanTx(bool success) {
  Serial.printf("TX %s\n", success ? "OK" : "FAILED");
}

void setup() {
  Serial.begin(115200);

  // Register callbacks
  TwaiTaskBased::onReceive(onCanRx);
  TwaiTaskBased::onTransmit(onCanTx);

  // Initialize TWAI interface
  // GPIO 15 = TX, GPIO 13 = RX, 500 kbps
  if (!TwaiTaskBased::begin(GPIO_NUM_15, GPIO_NUM_13, 500000)) {
    Serial.println("Failed to initialize TWAI");
    while (1);  // Halt on error
  }

  Serial.println("TWAI initialized successfully");
}

void loop() {
  // Example: Send a CAN message
  twai_message_t msg = {
    .identifier = 0x123,                      // CAN message ID
    .data_length_code = 8,                    // 8 bytes
    .data = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08}
  };

  if (TwaiTaskBased::send(msg)) {
    Serial.println("Message queued for transmission");
  } else {
    Serial.println("Failed to queue message");
  }

  delay(1000);
}
```

## API Reference

### Static Methods

#### `bool begin(gpio_num_t txPin, gpio_num_t rxPin, uint32_t baudrate, uint32_t rxTaskStack, UBaseType_t rxTaskPrio, UBaseType_t txTaskPrio)`

Initialize the TWAI interface.

**Parameters:**
- `txPin` - GPIO pin for CAN TX
- `rxPin` - GPIO pin for CAN RX
- `baudrate` - CAN bus speed (125000, 250000, 500000, 1000000) - default 500000
- `rxTaskStack` - RX task stack size in bytes - default 4096
- `rxTaskPrio` - RX task priority - default 10
- `txTaskPrio` - TX task priority - default 9

**Returns:**
- `true` if initialization successful
- `false` if initialization failed

#### `void onReceive(RxCallback cb)`

Register a callback function for CAN message reception.

**Parameters:**
- `cb` - Function pointer: `void callback(const twai_message_t &msg)`

**Example:**
```cpp
void rxCallback(const twai_message_t &msg) {
  // Handle received message
}
TwaiTaskBased::onReceive(rxCallback);
```

#### `void onTransmit(TxCallback cb)`

Register a callback function for transmission completion.

**Parameters:**
- `cb` - Function pointer: `void callback(bool success)`

**Example:**
```cpp
void txCallback(bool success) {
  if (success) {
    Serial.println("Message sent successfully");
  } else {
    Serial.println("Transmission failed");
  }
}
TwaiTaskBased::onTransmit(txCallback);
```

#### `bool send(const twai_message_t &msg, TickType_t timeout)`

Queue a CAN message for transmission.

**Parameters:**
- `msg` - TWAI message to send
- `timeout` - Maximum wait time for queue space (default: portMAX_DELAY for infinite)

**Returns:**
- `true` if message was queued successfully
- `false` if queue is full or library not initialized

**Example:**
```cpp
twai_message_t msg = {
  .identifier = 0x100,
  .data_length_code = 2,
  .data = {0x12, 0x34}
};

if (TwaiTaskBased::send(msg)) {
  Serial.println("Message queued");
} else {
  Serial.println("Queue full");
}
```

## CAN Message Structure

Messages are sent and received using the `twai_message_t` structure:

```cpp
twai_message_t msg = {
  .identifier = 0x123,        // 11-bit CAN ID
  .data_length_code = 8,      // 0-8 bytes
  .data = {                   // Message data
    0x00, 0x01, 0x02, 0x03,
    0x04, 0x05, 0x06, 0x07
  }
};
```

## Task Architecture

The library creates two dedicated FreeRTOS tasks:

- **RX Task** - Continuously monitors the CAN bus and calls the RX callback for each message received
- **TX Task** - Processes queued messages from the internal queue and transmits them

Both tasks are pinned to Core 0 and run with adjustable priorities (default: RX=10, TX=9).

## Supported Baudrates

- 125 kbps
- 250 kbps
- 500 kbps (default)
- 1000 kbps

Other baudrates will default to 500 kbps.

## GPIO Pinout (Example)

```
ESP32-WROOM-32          CAN Transceiver
GPIO 15 (TX)    ------>  TX (input to transceiver)
GPIO 13 (RX)    <------  RX (output from transceiver)
GND             ------>  GND
```

Common CAN transceiver modules:
- MCP2551 (3.3V-tolerant)
- TJA1050 (3.3V-tolerant)

## Troubleshooting

**Library initialization fails:**
- Verify GPIO pins are not in use by other peripherals
- Check CAN transceiver is powered and connected
- Ensure baudrate matches other devices on bus

**No messages received:**
- Check CAN transceiver connections
- Verify RX callback is registered
- Monitor serial output for initialization messages

**TX queue full:**
- Reduce message transmission rate
- Increase TX task priority
- Ensure RX callback doesn't block

## Version History

- **0.0.1** - Initial release for ESP32-WROOM-32

## License

MIT License
