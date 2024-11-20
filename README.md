# COL788 Assignments

The repository contiains 5 assignments done under the course COL788 under Prof. Rijurekha Sen.
## Structure  
The repository is divided into 5 subfolders:  

1. **Assignment1/**  
   Contains whole source code under code subfolder and a main.c. 

2. **Assignment2/**
   Includes: 
   - Contains whole source code under code subfolder and a main.c.
   - Manual changes made to `main.c` and `.ioc` file 
   - Files included in `Assignment2/code/Core/Inc` are `max30102_for_stm32_hal.h`, `max_processing.h`.
   - Files included in `Assignment2/code/Core/Src` are `ax30102_for_stm32_hal.c`, `max_processing.c`. `stm32l0xx_it.c` was changed to handle Temperaure interupts

4. **Assignment3/**  
   Includes: 
   - Contains whole source code under code subfolder and a main.c.
   - Manual changes made to `main.c` and `.ioc` file to integrate `FATFS` Library.
   
5. **Assignment4/**
   Includes:
   - Contains whole source code under code subfolder and a main.c.
   - Clean handwritten diagram of hardware connections.  
   - Picture of the actual hardware setup.
   - Manual changes made to `main.c` and `.ioc` file  to integrate SD Card and Sensor.

7. **Assignment5/**  
   Includes:  
   - Clean handwritten diagram of hardware connections.  
   - Picture of the actual hardware setup.
   - Contains whole source code under code subfolder and a main.c.
   - Logs for HeartRate and Temperature as written on the SD_Card.
   - Manual changes made to `main.c` and `.ioc` file to include `FreeRTOS` library.
---

## Hardware Requirements  
1. **STM32 L073RZ Nucleo Board**  
2. **BMP280 Sensor** (For assignment 1)
3. **MAX30102 Sensor* (For Assignment 2,4 and 45)
4. **SD Card reader and SD Card(FAT32 formattable)** (For Assignments 3,4, and 5)
