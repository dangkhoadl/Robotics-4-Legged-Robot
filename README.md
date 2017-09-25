# Four legged walking Robot
My project to complete the Bachelor of Engineering degree

## Hardwares Resources
* 1 STM32F4 Microcontroller development board (ARM Cortex M4)
	- 12 PWM channels
	- 4 timer interrupts
	- 2 external event interrupts
	- 1 SPI channel
	- 1 UART channel
* 12 step motors with PWM control
	- 3 motor for 1 leg
* 1 MPU-6050 Gyroscope
	- 10-axis gyroscope
	- Interface with the microcontroller through SPI
* 1 Xbox360 Controller
	- Interface with Microcontroller through a wireless module

## Features
* Path planning and motion planning
	- 3 seleted gaits for traversing through different terrains
	- Creeping, Walking, Trotting
	- Direction of movements controlled by the Xbox 360 Controller
* Fuzzy Logic Controller
	- Implemented in the ARM CPU
	- Realtime response to terrain conditions
	- Input data: From 10 different gyroscope channels
	- Output: Signals to configre the gait parameter vector to adapt to the new environment
* Kalman filter
	- To eliminate Input noises from input data

## Demo
* [Creeping Gait](https://www.youtube.com/watch?v=fn-V60laoiw)
* [Walking Gait](https://www.youtube.com/watch?v=Subcgy-ac0Q)
* [Walking and Trotting Gaits](https://www.youtube.com/watch?v=GGXLdF0h1R4)
* [Traversing through different terrains](https://www.youtube.com/watch?v=WlVCievpWfg)

## How to run
* Use [ARM Keil IDE ](http://www2.keil.com/mdk5/)  to build and compile project files into .bin object
* Use the built-in debugger to load the object file into the chip ROM
