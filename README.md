# CH592 BLE to Hardware UART Bridge (with Hardware CTS/RTS Flow Control)

A high-performance, transparent bidirectional bridge between **Bluetooth Low Energy (Nordic NUS)** and physical **UART0** with **Hardware CTS/RTS Auto-Flow Control** on the **WCH CH592** microcontroller.

```mermaid
flowchart LR
    Device["🔌 External Serial Device<br/><code>(MCU / Sensor / Host)</code>"] <-->|"Hardware UART0 (with CTS/RTS)<br/>TX:PB7, RX:PB4, CTS:PB0, RTS:PB6"| CH592["⚡ WCH CH592 MCU<br/><code>(UART0 HW Flow + Nordic NUS)</code>"]
    CH592 <-->|"BLE 2.4 GHz Nordic NUS<br/>(LE 2M PHY • 145 KB/s)"| Client["📱 BLE Central Client<br/><code>(Phone / PC / Web BLE)</code>"]
```

---

## Features

- **Standard Nordic UART Service (NUS)**:
  - Universally recognized by mobile apps (*nRF Connect*, *Serial Bluetooth Terminal*) and Web Bluetooth.
  - Advertises as `CH592_BLE_UART`.
  - Supports **LE 2M PHY** with up to **145 KB/s** application throughput.
- **Hardware UART0 with Auto-Flow Control (CTS/RTS)**:
  - **Hardware CTS (`PB0`)**: UART hardware automatically halts transmission when the remote device deasserts CTS.
  - **Hardware RTS (`PB6`)**: Automatically throttles remote transmission when the ring buffer reaches high-water mark, resuming when drained.
  - **Zero-Wait-State SRAM ISR**: Vectored `UART0_IRQHandler` running from `.highcode` with 8-byte hardware FIFO and 1024-byte circular buffers.
- **High-Throughput Batch Pumping**:
  - Pushes up to 8 packets per cycle into the BLE controller TX queue to saturate the 15 ms connection event.
- **On-Demand Diagnostics via Pad 2 (PB22)**:
  - Grounding Pad 2 prints real-time link status (MTU, PHY, Notifications state, byte counters) directly to UART0.

---

## Pinout Mapping

| Function | Pin | Default Direction | Description |
|:---|:---:|:---:|---|
| **TXD0** | `PB7` | Output (PP 5mA) | CH592 Transmit -> Remote RX |
| **RXD0** | `PB4` | Input (Pull-Up) | CH592 Receive <- Remote TX |
| **CTS0** | `PB0` | Input (Pull-Up) | Clear-to-Send <- Remote RTS |
| **RTS0** | `PB6` | Output (PP 5mA) | Ready-to-Receive -> Remote CTS |
| **BOOT / Diag** | `PB22` | Input (Pull-Up) | Ground to enter ISP flash; tap for link status |
| **GND** | `GND` | Ground | Common Ground |
| **3V3** | `VIO33` | Power Input | **+3.3V Only** *(Do not connect 5V!)* |

---

## Quick Start

### 1. Build
```bash
cd ch592_ble_uart
make
```

### 2. Flash
1. Ground **PB22 (BOOT)** to **GND**.
2. Connect USB/power.
3. Flash:
   ```bash
   ./flash.sh
   ```
4. Release the BOOT bridge.

### 3. Connect & Communicate
- Connect your external MCU or FTDI / USB-to-UART adapter to `PB7 (TX)`, `PB4 (RX)`, `PB0 (CTS)`, `PB6 (RTS)` and `GND`.
- Set baud rate to **115,200 bps** (8N1, RTS/CTS flow control enabled).
- Connect via **nRF Connect** or **Serial Bluetooth Terminal** to `CH592_BLE_UART`.
- Enable notifications on characteristic `6E400003-B5A3-F393-E0A9-E50E24DCCA9F`.
- All bytes sent from the UART side appear instantly in the BLE app, and vice versa.
