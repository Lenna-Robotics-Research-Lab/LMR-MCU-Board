# LMR Bardia MCU Board

## Overview

The Bardia Board, designed as an open-source hardware by Lenna Robotics Lab, is the MCU board for the LMR v1.1. The Schematics and PCB of this board is free to use under open-source licensing. 

![Alt text](https://github.com/Lenna-Robotics-Research-Lab/LMR-MCU-Board/blob/main/images/Lenna_Board_2.png "LMR v1.1 Bardia Board")
</br>

Some highlights on the board are listed below:

1. STM32F407VGT6 ARM Microcontroller 
2. Ethernet Support (DP83848 Physical Layer) 
3. On-board L298 Motor Driver 
4. USB to Serial (CH340G) 
5. Simultaneous USB and Battery Power Support 
6. Dedicated ADC for Reading Battery Level
7. Dedicated Fan Slot  
8. Ports(1x I2C Bus, 1x UART, 1x SPI, 1x CAN, 20x GPIO)
9. Dedicated SRF-04 Ultrasonic Sensor Slots (4x)
 
## Microcontroller 

<p align="justify">
The microcontroller unit (MCU) is a critical component in the design, as it serves as the central processing unit for the robot. Selecting the right MCU was crucial, and our primary considerations were the available peripherals and their compatibility with various subsystems of the robot, as well as the local availability of the MCU in our region.

After careful evaluation, STM32F407VGT6 was chosen as the MCU for this project. This powerful microcontroller offers a wealth of features that make it an ideal choice, including multiple of each communication protocols like I2C, SPI, and UART, as well as Ethernet support. Moreover, its affordability and widespread availability further solidified its suitability for this project's requirements. <p>


