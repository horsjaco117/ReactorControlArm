# Project Name: AGS-201 Control rod interface software and wiring diagrams

## Overview
All the software, connections, and Troubleshooting procedures are contained here. 

Features
* **Real-time Monitoring** Read all the button inputs and analog position data for the AGS-201
* **Hardware Interfacing** A basic Arduino Microcontroller is able to interface with the control panel and read data. 
* **Optional User Interfaces** LabView, Raspberry Pi python code, and Arduino code available to interface with control panel depending on configuration

## Hardware Requirements to run software

* Arduino Uno style microcontroller or Arduino Giga 
* Raspberry Pi (If utilizing the Nuclear Reactor code interface)
* Laptop or Desktop Computer for running the packaged LabView.exe files
* College of Technology PCB with built in Ethernet adapter
* Optional Screen shield for the Arduino Giga

Requirements for modifying code 
* Arduino IDE for Microcontroller
* USB-A to USB-C data transfer cable
* Laptop or Desktop computer (Compatible mostly with Windows. Other OS's haven't been tested.)
* Geany code reader for Raspberry Pi
* Raspberry Pi
* Ethernet cable
* Stepper motor driver with Stepper motor

## Quick Setup for Arduino
1. Click on the Green code button on the main page of github. 
2. In the menu that appears click on the download ZIP file. 
3. All files for this project will be in the ZIP file. 
4. To download the code on the arduino open the Arduino code inside the CodeFolder.
5. There are two main sets of code that are working for the main projects. Select Control Arm for Giga if using the Arduino Giga. Select UnoR4Working if using the Arduino Uno R4. 
6. Ensure the Arduino IDE is installed from the official website onto your device.
7. Connect your Arduino via USB and ensure it's connected
8. Click verify and if the output message is good click the upload button to install the code onto the board. Code can be reWritten for further iterations of code. 
9. The Arduino IDE will return a successfuly upload 
10. Enjoy the software

## Quick guide to LabView exe files
1. Enter the codefolder and select LabViewCode
2. Got to the EXEFolder
3. Select the desired LabView test folder then double click the .exe file

## Quick guide to the picode files
1. Upload the desired picode onto a USB flash drive
2. Mount the USB drive on you Pi and open with a code editor
3. Let code run on the Pi. 
