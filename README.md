# LMR Bardia MCU Board

## Overview

The Bardia Board, designed as an open-source hardware by Lenna Robotics Lab, is the MCU board for the LMR v1.1. The Schematics and PCB of this board is free to use under open-source licensing. 

![Alt text](https://github.com/Lenna-Robotics-Research-Lab/LMR-MCU-Board/blob/main/images/Lenna_Board_2.png "LMR v1.1 Bardia Board")
</br>

Some highlights on the board are listed below:

1. STM32F407VGT6 ARM Microcontroller 
2. On-board L298 Motor Driver 
3. USB to Serial (CH340G) 
4. Simultaneous USB and Battery Power Support 
5. Dedicated ADC for Reading Battery Level
6. Dedicated Fan Slot  
7. Ports(1x I2C Bus, 1x UART, 1x SPI, 1x CAN, 20x GPIO)
8. Dedicated SRF-04 Ultrasonic Sensor Slots (4x)
9. Ethernet Support (DP83848 Physical Layer) 
 
## Microcontroller 

<p align="justify">
The microcontroller unit (MCU) is a critical component in the design, as it serves as the central processing unit for the robot. Selecting the right MCU was crucial, and our primary considerations were the available peripherals and their compatibility with various subsystems of the robot, as well as the local availability of the MCU in our region.<p>
<p align="justify">
After careful evaluation, STM32F407VGT6 was chosen as the MCU for this project. This powerful microcontroller offers a wealth of features that make it an ideal choice, including multiple of each communication protocols like I2C, SPI, and UART, as well as Ethernet support. Moreover, its affordability and widespread availability further solidified its suitability for this project's requirements. <p>

## L298 Driver Motor

<p align="justify">
The on-board L298 H-bridge motor driver simplifies the motor control process. To get started, simply connect the motors correctly and begin using the board. When using two DC motors, the on-board driver is sufficient. However, for projects requiring more motors or higher precision, we recommend using a dedicated motor driver board to avoid potential noise interference with other devices.<p>

### Noise Prevention Measures

To mitigate noise generated by the motor driver, the following precautions have been implemented:
- ***Ground Connections***: The driver ground and circuit ground are connected at a single node using a ferrite bead.
- ***EMI/EMC Reduction***: Stitching is utilized on the driver part of the board to further reduce electromagnetic interference (EMI).

## USB to Serial Interface 

<p align="justify">
The LMR Bardia Board features a USB type-B port, enabling direct connection to your computer. This straightforward setup supports seamless integration with environments like MATLAB and OCTAVE for monitoring and data processing. The board leverages the CH340G IC for USB communication and can be powered through the USB's 5V supply. <p>






