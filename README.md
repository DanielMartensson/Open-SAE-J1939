# Open SAE J1939

SAE J1939 is a protocol for shaping the CAN-bus message in a specific way that suits industrial vehicles such as tractors, machinery, trucks and more.

SAE J1939 is a very easy protocol to use, but there is a lack of information about SAE J1939, due to the cost of the protocol document, available how to shape a CAN-bus message according to SAE J1939 protocol standard. So therefore I’m writing a SAE J1939 protocol available for free to use on any embedded systems such as STM32, Arduino, AVR, PIC etc.

To learn to build on this project, you need first to understand SAE J1939. I have written this project in C language because C is an industry standard. The C language dialect I have chosen is C99 and I don’t use dynamical memory allocation in this library. So it will work with MISRA C standard.

With this library, you can communicate with valves, engines, actuators, hardware and all other things that are suitable for heavy industrial mobile applications. I have build up a basic structure of the project and I hope that other users will send pull request of their C code for extra functionality to SAE J1939 standard because SAE J1939 is a huge standard. 

# Getting started

The first thing you need to know is to read the document `Open SAE J1939.pdf` inside the `Documention` folder.
Learn the structure of the project, else you won't be able to understand SAE J1939. 
After you have got a basic understanding of the project, you are able to build on it. Keep it simple and follow the 
SAE J1939 standard!

After you have understand the structure of the project, then select processor choice in `CAN_Netork.c` file.
Here you can select for example `STM32`, `Arduino`, `PIC`, `AVR` etc.

# The structure of the project

![a](https://raw.githubusercontent.com/DanielMartensson/Open-SAE-J1939/main/Documention/Pictures/Project%20structure.png)

# SAE J1939 functionality
 - SAE J1939:21 Transport Layer
 	- Acknowledgement
 	- Request
 	- Transport Protocol Connection Management
 	- Transport Protocol Data Transfer
 - SAE J1939:71 Application Layer
 	- Request Component Identification
 	- Request ECU Identification
 	- Request Software Identification
 - SAE J1939:73 Diagnostics Layer
 	- DM1
 	- DM2
 	- DM3
 	- DM14
 	- DM15
 	- DM16
 - SAE J1939:81 Network Management Layer
 	- Address Claimed
 	- Commanded Address
 
# Extra functionality
 - ISO 11783 Tractors And Machinery For Agriculture And Forestry
 	- ISO 11783-7 Implement messages application layer
 		- Auxiliary Valve Command
 		- Auxiliary Valve Estimated Flow
 		- Auxiliary Valve Measured Position
 		- General Purpose Valve Command
 		- General Purpose Estimated Flow
 	