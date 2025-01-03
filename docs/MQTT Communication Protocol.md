# MQTT Communication Protocol

## Table of Contents

- [MQTT Communication Protocol](#mqtt-communication-protocol)
  - [Table of Contents](#table-of-contents)
  - [Overview](#overview)
  - [Design Architecture](#design-architecture)
    - [Device Information](#device-information)
    - [Message Structure](#message-structure)
      - [Status Byte](#status-byte)
      - [Example Response](#example-response)
    - [Command Handling](#command-handling)
  - [Core Components](#core-components)
    - [MQTT Message Schema](#mqtt-message-schema)
      - [Header](#header)
      - [Base Message Class](#base-message-class)
    - [Telemetry Commands](#telemetry-commands)
    - [Message Serialization](#message-serialization)
    - [Usage](#usage)
    - [Examples](#examples)
      - [Sending Device Boot Info](#sending-device-boot-info)
      - [Parsing Incoming Messages](#parsing-incoming-messages)

---

## Overview

We have developed an MQTT-based protocol that facilitates efficient and reliable communication between devices and cloud services. Key features of the protocol include:

- **Device Identification**: Utilizes a unique "PSN" (Product Serial Number) to accurately identify device-specific messages.
- **Optimized Message Serialization**: Implements binary serialization to significantly enhance communication efficiency and reduce payload size.
- **Flexible Structured Design**: Features a structured format for message headers and payloads, enabling seamless support for diverse telemetry data and control commands.

For a practical implementation, refer to [`mqtt_client.py`](../python/mqtt_client.py). This file contains the complete implementation of the mqtt communication protocol, and you can start your development by editing this Python file.


---

## Design Architecture

### Device Information

Each device is identified using a `PSN` (Product Serial Number), stored in the ESP32's `protected_data` partition under the key `serial_number`. This ensures unique identification for each device during communication.

---

### Message Structure

- **Header**: Includes service ID and payload length for efficient parsing.
- **Payload**: Varies based on command and data type.

#### Status Byte

- `0x00`: Success
- `0x01`: Failure
- Other bytes represent specific API-defined data.

#### Example Response

For the command `GET_POWER_SUPPLY_STATUS`:
- `0x00`: Indicates success.
- `0x1F`: Binary `11111` means 5 ports are active.

---

### Command Handling

The system supports unified handlers for both BLE and MQTT commands. This ensures consistent behavior across communication protocols.

---

## Core Components

### MQTT Message Schema

Header and payload structures are implemented in `mqtt_message.h` to define and serialize MQTT messages.

#### Header

```cpp
struct MQTTMessageHeader {
  uint16_t service;
  uint16_t message_id;
};
```

#### Base Message Class

```cpp
struct MQTTMessage {
  MQTTMessageHeader header;
  virtual ~MQTTMessage() = default;
  virtual std::vector<uint8_t> serialize() const;
};
```

### Telemetry Commands
Defined commands handle various telemetry and control tasks:

```cpp
enum TelemetryServiceCommand : uint16_t {
  STREAM_PORTS_STATUS = 0x0080,
  STREAM_DEVICE_STATUS = 0x0081,
  PING = 0x0100,
  DEVICE_BOOT_INFO = 0x0110,
  POWER_HISTORICAL_DATA = 0x0130,
  // Additional commands...
};
```

### Message Serialization
All message structures provide a serialize() method to convert their fields into a byte array for transmission.

Example: `StreamPortStatus`

```cpp
struct StreamPortStatus : MQTTMessage {
  uint8_t status;
  uint8_t port_status_map;
  PortStatusData ports[8];

  std::vector<uint8_t> serialize() const override {
    std::vector<uint8_t> payload = header.serialize();
    payload.emplace_back(status);
    payload.emplace_back(port_status_map);
    // Serialize each port's data
    for (const auto &port : ports) {
      const uint8_t *portData = reinterpret_cast<const uint8_t *>(&port);
      payload.insert(payload.end(), portData, portData + sizeof(PortStatusData));
    }
    return payload;
  }
};
```

### Usage
To integrate this framework into your project:

1. Include the mqtt_message.h header file in your project.
2. Define your specific telemetry or control commands.
3. Use the serialize() methods to prepare payloads for MQTT transmission.

### Examples
#### Sending Device Boot Info

```cpp
DeviceBootInfo bootInfo = { /* Initialize fields */ };
std::vector<uint8_t> payload = bootInfo.serialize();
// Send payload over MQTT
```

#### Parsing Incoming Messages

```cpp
void handleIncomingMessage(const std::vector<uint8_t>& payload) {
  // Deserialize based on header service ID
}
```