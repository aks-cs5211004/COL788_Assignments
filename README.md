# COL788 Assignments

This repository contains the completed assignments for the course **COL788** under **Prof. Rijurekha Sen**.

## Repository Structure

The repository is divided into 5 subfolders:

### 1. **Assignment1/**
Includes:
- Complete source code located in the `code/` subfolder.
- `.ioc` files:
  - `Assignment1/code/blinky/blinky3.ioc`
  - `Assignment1/code/bmp-sensor/sensor.ioc`
- `main.c` files:
  - `Assignment1/code/bmp-sensor/Core/Src/main.c`
  - `Assignment1/code/blinky/Core/Src/main.c`

---

### 2. **Assignment2/**
Includes:
- Complete source code located in the `code/` subfolder, including `main.c`.
- Manual modifications made to `main.c` and the `.ioc` file.
- `.ioc` file:  
  - `Assignment2/code/laad.ioc`
- `main.c` file:  
  - `Assignment2/code/Core/Src`
- Files in `Assignment2/code/Core/Inc`:
  - `max30102_for_stm32_hal.h`
  - `max_processing.h`
- Files in `Assignment2/code/Core/Src`:
  - `max30102_for_stm32_hal.c`
  - `max_processing.c`
  - `stm32l0xx_it.c` (modified to handle temperature interrupts).
  

---

### 3. **Assignment3/**
Includes:
- Complete source code located in the `code/` subfolder, including `main.c`.
- Manual modifications made to `main.c` and the `.ioc` file to integrate the `FATFS` library.
- `.ioc` file:  
  - `Assignment3/code/SD_card.ioc`
- `main.c` file:  
  - `Assignment3/code/Core/Src`

---

### 4. **Assignment4/**
Includes:
- Complete source code located in the `code/` subfolder, including `main.c`.
- Clean, handwritten diagram of the hardware connections: `Assignment4/Connections_Drawing.jpeg`.
- Picture of the actual hardware setup: `Assignment4/Connections.jpeg`.
- Manual modifications made to `main.c` and the `.ioc` file to integrate the SD Card and Sensor.
- `.ioc` file:  
  - `Assignment4/code/SD_card.ioc`
- `main.c` file:  
  - `Assignment4/code/Core/Src`

---

### 5. **Assignment5/**
Includes:
- Complete source code located in the `code/` subfolder, including `main.c`.
- Clean, handwritten diagram of the hardware connections: `Assignment5/Connections_Drawing.jpeg`.
- Picture of the actual hardware setup: `Assignment5/Connections.jpeg`.
- Logs for Heart Rate and Temperature data written to the SD Card: `Assignment5/heart_rate_log.txt` and `Assignment5/temperature_log.txt`
- Manual modifications made to `main.c` and the `.ioc` file to include the `FreeRTOS` library.
- `.ioc` file:  
  - `Assignment5/code/assignment5.ioc`
- `main.c` file:  
  - `Assignment5/code/Core/Src`

---

## Hardware Requirements

1. **STM32 L073RZ Nucleo Board**
2. **BMP280 Sensor** (used in Assignment 1)
3. **MAX30102 Sensor** (used in Assignments 2, 4, and 5)
4. **SD Card Reader and SD Card (FAT32-formattable)** (used in Assignments 3, 4, and 5)

---

# Git Issues

Around 8 to 9 Git Issues have been raised. Issues marked as resolved are left open for reference in case others encounter similar errors.
