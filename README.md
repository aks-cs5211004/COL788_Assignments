# COL788 Assignments

The repository contiains 5 assignments done under the course COL788 under Prof. Rijurekha Sen.
## Structure  
The repository is divided into 5 subfolders:  

1. **Assignment1/**  
   Includes: 
   - Contains whole source code under code subfolder.
   - `.ioc` file can be found at `Assignment1/code/blinky/blinky3.ioc` and `Assignment1/code/bmp-sensor/sensor.ioc`.
   - `main.c` can be found at `Assignment1/code/bmp-sensor/Core/Src/main.c` and `Assignment1/code/blinky/Core/Src/main.c`.

2. **Assignment2/**
   Includes: 
   - Contains whole source code under code subfolder and the `main.c`.
   - Manual changes made to `main.c` and `.ioc` file
   - `.ioc` file can be found at `Assignment2/code/laad.ioc`
   - Files included in `Assignment2/code/Core/Inc` are `max30102_for_stm32_hal.h` and  `max_processing.h`. These files can be found at `Assignment2/code/Core/Inc`
   - Files included in `Assignment2/code/Core/Src` are `ax30102_for_stm32_hal.c` and `max_processing.c`. `stm32l0xx_it.c` was changed to handle Temperaure interupts. These files can be found at `Assignment2/code/Core/Src`

4. **Assignment3/**  
   Includes: 
   - Contains whole source code under code subfolder and the `main.c`.
   - Manual changes made to `main.c` and `.ioc` file to integrate `FATFS` Library.
   -  `.ioc` file can be found at `Assignment3/code/SD_card.ioc`
   
5. **Assignment4/**
   Includes:
   - Contains whole source code under code subfolder and the `main.c`.
   - Clean handwritten diagram of hardware connections.  
   - Picture of the actual hardware setup.
   - Manual changes made to `main.c` and `.ioc` file  to integrate SD Card and Sensor.
   - `.ioc` file can be found at `Assignment4/code/SD_card.ioc`

7. **Assignment5/**  
   Includes:  
   - Clean handwritten diagram of hardware connections.  
   - Picture of the actual hardware setup.
   - Contains whole source code under code subfolder and the `main.c`.
   - Logs for HeartRate and Temperature as written on the SD_Card.
   - Manual changes made to `main.c` and `.ioc` file to include `FreeRTOS` library.
   - `.ioc` file can be found at `Assignment5/code/assignment5.ioc`
---

## Hardware Requirements  
1. **STM32 L073RZ Nucleo Board**  
2. **BMP280 Sensor** (For assignment 1)
3. **MAX30102 Sensor** (For Assignment 2,4 and 45)
4. **SD Card reader and SD Card(FAT32 formattable)** (For Assignments 3,4, and 5)
