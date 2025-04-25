# Wearable F411RE Locator

This is an embedded C++ project for a wearable locator system based on the STM32F411RE microcontroller. The system uses an IMU (MPU-9250) and RSSI from LoRa modules to estimate the relative position between two devices. It runs on FreeRTOS and includes custom sensor fusion, calibration routines, and LoRa-based distance estimation.

---

## 🛠️ Requirements

To build and run the project, you need the following:

- **VS Code with STM32VSCode Extension**
- **STM32CubeCLT v1.17.0** – for compatibility with STM32 build tools.
- **STM32CubeMX** – for pin configuration and code generation.
- **CMake** – as the project build system (minimum 3.22).
- **ST-Link driver** – for flashing and debugging.
- **Serial terminal app** (e.g., **PuTTY**) – to monitor device output via the COM port (USB to UART2 through ST-Link).

---

## ⚙️ How to Build & Run

1. Open the project folder using **VS Code**.
2. Ensure system paths are configured for CubeCLT and CMake.
3. Go to the **Run and Debug** tab.
4. Select **Build & Debug Microcontroller - ST-Link**.
5. Hit **Run**.

---

## 🧪 Calibration Flow

1. **IMU Calibration**:
   - Lay the device flat until accelerometer and gyroscope calibration completes.

2. **Magnetometer Calibration**:
   - Rotate the device in multiple directions until completion.

3. **Path Loss Exponent**:
   - Choose a path loss exponent based on your environment (e.g., open field, urban).

4. **RSSI Calibration**:
   - Hold two devices **1 meter apart, facing each other** until calibration finishes.

---

## 🔄 FreeRTOS Task Structure

The system uses FreeRTOS for task scheduling. Task creation and flow depend on whether the device is configured as a transmitter or receiver.

### 📡 Transmit Device Flow

1. **`rssi_calibration_tx`** – Performs 1-meter RSSI calibration.  
2. After completion, it spawns:
   - **`quaternion_update`** – Continuously updates orientation using IMU data.
   - **`LoRa_task_send`** – Sends direction and distance via LoRa.
3. `rssi_calibration_tx` then deletes itself to free resources.

### 📥 Receive Device Flow

1. **`rssi_calibration_rx`** – Performs 1-meter RSSI calibration.  
2. After completion, it spawns:
   - **`quaternion_update`** – Continuously updates orientation.
   - **`LoRa_task_receive`** – Receives data from the transmitting node.
   - **`display_task`** – Outputs relative direction and distance to the terminal.
3. `rssi_calibration_rx` then deletes itself to free resources.
---

## 📁 Project Structure

### 📂 Core

Main application logic is implemented here, split into headers (`Inc/`) and implementations (`Src/`):

#### **Inc/**
- `compute_direction.h` – Header for computing relative direction between two devices using IMU data.
- `custom_printf.h` – Lightweight formatted print utility for outputting data over UART2 (ST-Link COM port).
- `FreeRTOSConfig.h` – RTOS configuration file for task scheduling and memory.
- `IMU_9250.h` – Interface for initializing and reading from the MPU-9250 IMU.
- `LoRa.h` – LoRa communication interface, includes calibration task logic.
- `MadgwickAHRS.h` – Madgwick sensor fusion algorithm header.
- `main.h` – Function declarations and main setup routines.
- `quaternion.h` – Definitions and examples for quaternion math and operations.
- `stm32f4xx_hal_conf.h`, `stm32f4xx_it.h` – STM32 HAL and interrupt configuration.

#### **Src/**
- `compute_direction.cpp` – Contains `compute_direction` FreeRTOS task that calculates direction and prints it via terminal.
- `custom_printf.c` – Sends formatted data to the terminal over USB (UART2).
- `freertos.c` – FreeRTOS initialization and system startup logic.
- `IMU_9250.c` – Handles reading IMU sensor data (accel, gyro, mag).
- `LoRa.cpp` – Manages LoRa communication, including distance estimation and calibration for both TX and RX.
- `MadgwickAHRS.cpp` – Implements sensor fusion filter using Madgwick's algorithm.
- `main.cpp` – System entry point, initializes peripherals and starts the RTOS scheduler.
- `quaternion.cpp` – Contains the `quaternion_update` task for real-time orientation tracking.
- `stm32f4xx_hal_*.c`, `syscalls.c`, `sysmem.c`, `system_stm32f4xx.c` – HAL interface and system-level configuration for STM32F4.

---

### 📂 Drivers

Includes CMSIS and STM32 HAL drivers for hardware abstraction.

### 📂 Middlewares

Contains **FreeRTOS** kernel source files and configuration for multitasking.

### 📂 RadioLib

Houses the [RadioLib](https://github.com/jgromes/RadioLib) library for LoRa communication, adapted to use STM32 HAL SPI API.

---

## 🔧 Build Files

- `CMakeLists.txt` – CMake build script.
- `startup_stm32f411xe.s` – Assembly-level boot code for the MCU.
- `STM32F411RETx_FLASH.ld` – Linker script defining memory layout.
- `.mxproject`, `.project` – STM32CubeMX project metadata.
- `compile_commands.json`, `CMakePresets.json` – Settings for code analysis and CMake build presets.

---

## 📌 Notes

- Ensure **CubeCLT v1.17.0** is used to avoid version mismatches.
- Use **PuTTY** or a similar tool to monitor output via ST-Link COM port.
- During RSSI calibration, maintain stable device orientation for accurate results.

---
