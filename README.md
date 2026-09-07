# CAN-Networked Electrical Control Unit — Architecture

## Overview

A single-node automotive electrical control unit using CAN (Controller Area Network) as the telemetry and actuation reporting bus. The node (**ECU**) reads a BME280 environmental sensor over I2C, makes a threshold-based control decision with hysteresis, actuates a relay to drive a fan, and broadcasts all sensor data and actuator state on the CAN bus at 500 kbps. The design covers four layers: **power distribution**, **network/communication**, **signal actuation**, and **requirements verification**.

---

## Architecture Diagram

<!-- Insert diagram image here, e.g. ![Architecture Diagram](./diagram.png) -->

---

## 1. Power Distribution Layer

| Element | Role |
|---|---|
| 12V Supply | Primary power source |
| USB (5V) | Powers STM32 Nucleo board via ST-LINK |
| +12V Rail | Powers fan load through relay |
| +5V Rail | Powers relay module coil |
| +3.3V Rail | Powers BME280 and SN65HVD230 transceiver (sourced from Nucleo 3.3V pin) |
| Common Ground (GND) | Shared return path for all nodes — critical for CAN's differential signaling reference and relay control signal reference |

### Load Analysis

| Component | Measured/Rated Current | Notes |
|---|---|---|
| STM32F446RE Nucleo | ~80 mA | Active mode, USB powered |
| BME280 breakout | ~1 mA | Negligible |
| SN65HVD230 transceiver | ~10 mA | Active transmit |
| Relay module coil | See relay module datasheet | When energized |
| 12V DC fan | See fan datasheet | Load dependent |
| **Total 12V rail draw** | **Fan + relay coil combined** | Verify against your specific components |
| **Total USB draw** | **~160 mA** | STM32 + transceiver + BME280 |

**Fuse sizing:** Size the fuse based on your measured worst-case fan and relay current draw, with at least 3x margin to accommodate motor inrush current at startup while protecting the harness.

**Wire gauge:** For bench-scale runs under 0.5m at <600 mA, 22 AWG is comfortably rated. For longer harness runs, wire gauge should be re-evaluated against voltage drop over distance in addition to ampacity.

---

## 2. Node Architecture

### ECU Node

| Block | Function |
|---|---|
| BME280 Sensor | Reads temperature, humidity, and barometric pressure over I2C |
| STM32F446RE MCU | Runs control loop, makes threshold decision, drives relay, transmits CAN frames |
| SN65HVD230 Transceiver | Converts 3.3V CAN TX/RX logic ↔ 12V differential CANH/CANL bus signals |
| Opto-isolated Relay Module | Electrically isolates 3.3V GPIO control from 12V fan switching circuit |
| 12V DC Fan | Physical actuator — the system's output |
| 120Ω Termination Resistors ×2 | One at each physical end of the CAN bus — required to prevent signal reflections |

### Debug / Monitoring Node (Optional)

| Block | Function |
|---|---|
| USB-CAN Adapter (e.g. CANable) | Second bus node — provides ACK and allows frame capture on a host PC |
| SavvyCAN / CANoe | Decodes frames in real time using the DBC message database |

---

## 3. Signal Flow

**Sensor path:** The BME280 is polled every 250ms over I2C1 (PB8/PB9). The STM32 sends the register address, reads 8 bytes of raw ADC data, and applies Bosch's integer compensation formulas to produce calibrated temperature (×100 °C), humidity (×100 %RH), and pressure (Pa). Temperature compensation runs first because it produces the `t_fine` intermediate value that pressure and humidity compensation both depend on.

**Control path:** After each successful sensor read, the firmware compares the compensated temperature against two thresholds. If temperature exceeds `TEMP_THRESHOLD_ON` (28.00°C), PB0 is driven high, energizing the relay coil and closing the fan circuit. If temperature drops below `TEMP_THRESHOLD_OFF` (26.00°C), PB0 is driven low, de-energizing the relay. The 2°C hysteresis gap prevents relay chatter when temperature hovers near a single threshold.

**CAN path:** Every 500ms, the firmware packs sensor readings and fan state into two CAN frames and transmits them via the bxCAN peripheral. Every 1000ms, a heartbeat frame is transmitted carrying uptime in seconds. The SN65HVD230 transceiver converts the STM32's single-ended TX/RX signals to differential CANH/CANL bus signaling.

**Power path:** The 12V supply feeds the fan through the relay's normally-open contact. The relay coil is driven from the Nucleo's 5V pin through the opto-isolated relay module. The 3.3V rail from the Nucleo powers the BME280 and SN65HVD230 directly.

---

## 4. CAN Network Configuration

| Parameter | Value |
|---|---|
| Bus standard | Classical CAN (ISO 11898) |
| Baud rate | 500 kbps |
| APB1 clock | 45 MHz (180 MHz SYSCLK, /4 prescaler) |
| Bit timing prescaler | 5 |
| BS1 | 15 TQ |
| BS2 | 2 TQ |
| Total TQ per bit | 18 |
| Sample point | 88.9% (16/18) |
| Termination | 120Ω at each physical bus end |
| Checksum | CRC-15 (handled by bxCAN peripheral) |

### CAN Message Schedule

| ID | Name | DLC | Rate | Payload |
|---|---|---|---|---|
| 0x010 | Heartbeat | 4 bytes | 1 Hz | Uptime seconds (uint32, LE) |
| 0x100 | SensorData | 8 bytes | 2 Hz | Temp×100 (int16 LE), Humidity×100 (uint16 LE), Pressure Pa (uint32 LE) |
| 0x101 | ActuatorState | 2 bytes | 2 Hz | Fan state (0x00=OFF, 0x01=ON), Reserved |

**Timing budget:** With a 500ms broadcast period, end-to-end latency from sensor reading to CAN frame on the bus is under 500ms — dominated by the sensor poll interval, not the CAN transmission time (a single 8-byte frame at 500 kbps takes ~144µs).

---

## 5. Pin Assignments (NUCLEO-F446RE)

| Signal | Pin | Header | Notes |
|---|---|---|---|
| I2C1_SCL | PB8 | Arduino D15 | Requires 4.7kΩ pull-up to 3.3V on SDA and SCL |
| I2C1_SDA | PB9 | Arduino D14 | |
| CAN1_RX | PA11 | Morpho CN10 | Avoids conflict with I2C1 alternate function on PB8/PB9 |
| CAN1_TX | PA12 | Morpho CN10 | |
| Fan relay GPIO | PB0 | Morpho CN10 | Active high — energizes relay coil |
| Status LED | PA5 | Onboard LD2 | Blinks at 1 Hz during normal operation |

---

## 6. Design Rationale

- **CAN over LIN:** CAN's multi-master differential signaling provides higher noise immunity and fault tolerance than LIN's single-wire approach. For a control unit reporting to a potential network of diagnostic tools, CAN is the appropriate automotive bus choice.
- **Transceiver required:** The STM32 bxCAN peripheral outputs single-ended 3.3V TX/RX logic. The CAN bus standard requires differential CANH/CANL signaling at 12V levels. The SN65HVD230 bridges this voltage domain gap in both directions.
- **120Ω termination at both ends:** CAN is a differential transmission line. Without matched termination at each physical end, signal reflections corrupt bus logic levels. Even on a short bench harness, omitting termination causes bit errors.
- **PA11/PA12 for CAN instead of PB8/PB9:** The F446RE's CAN1 peripheral has alternate functions on both pin pairs. PB8/PB9 conflict with I2C1 which is already assigned to the BME280. PA11/PA12 are clean.
- **Hysteresis on control threshold:** A single threshold causes relay chatter when temperature oscillates around the setpoint due to sensor noise. The 2°C gap between ON and OFF thresholds ensures the relay only changes state when the temperature has moved meaningfully — standard bang-bang control practice.
- **Opto-isolated relay module:** Electrically separates the 3.3V STM32 GPIO domain from the 12V fan switching circuit. Protects the MCU from inductive kickback and voltage transients when the fan motor switches.
- **Integer scaling in CAN payload:** Automotive CAN databases use integer scaling rather than raw IEEE 754 floats. Temperature is transmitted as int16 × 100 (e.g. 2350 = 23.50°C), humidity as uint16 × 100, pressure as uint32 in Pa. This matches industry convention, avoids endianness ambiguity of floats, and maps directly to DBC signal definitions.
- **ctrl_hum written before ctrl_meas:** BME280 hardware quirk documented in the Bosch datasheet — the humidity oversampling register only takes effect when ctrl_meas is subsequently written. Writing them in the wrong order silently disables humidity measurement.

---

## 7. Requirements Traceability

| Requirement | Design Decision | Verification Method |
|---|---|---|
| ECU shall read sensor data every 250ms | HAL_GetTick() interval check in main loop with 250ms period | Serial debug print confirms timestamp between readings |
| ECU shall actuate fan when temperature exceeds 28.00°C | TEMP_THRESHOLD_ON = 2800 (×100 format), GPIO set on crossing | Apply heat to sensor, confirm relay clicks and fan starts at correct temperature |
| ECU shall de-actuate fan when temperature drops below 26.00°C | TEMP_THRESHOLD_OFF = 2600, 2°C hysteresis gap | Remove heat, confirm relay opens only after temperature drops below 26.00°C, not at 28.00°C |
| ECU shall not chatter relay at threshold boundary | Hysteresis gap between ON and OFF thresholds | Hold temperature steady between 26–28°C, confirm relay does not toggle |
| ECU shall broadcast sensor telemetry at 2 Hz | HAL_GetTick() interval check with 500ms period | USB-CAN adapter confirms frame rate in SavvyCAN |
| ECU shall broadcast heartbeat at 1 Hz | Separate 1000ms interval check | Heartbeat ID 0x010 visible at 1 Hz in SavvyCAN |
| CAN frames shall decode correctly against DBC | Integer scaling matches DBC signal definitions | Load ecu_messages.dbc in SavvyCAN, confirm decoded values match serial debug output |
| Bus shall maintain defined idle state | 120Ω termination at each physical bus end | Oscilloscope measurement of CANH/CANL during idle — should sit at ~2.5V differential |
| System shall detect BME280 absence | Chip ID register checked on init, HAL_ERROR returned on mismatch | Disconnect BME280, confirm fast LED blink fault indication on power-up |
