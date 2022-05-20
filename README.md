# Open SAE J1939

SAE J1939 is a protocol for shaping the CAN-bus message in a specific way that suits industrial vehicles such as tractors, machinery, trucks and more.

SAE J1939 is a very easy protocol to use, but there is a lack of information about SAE J1939, due to the cost of the protocol document, available how to shape a CAN-bus message according to SAE J1939 protocol standard. So therefore I’m writing a SAE J1939 protocol available for free to use on any embedded systems such as STM32, Arduino, AVR, PIC etc or PC.

To learn to build on this project, you need first to understand SAE J1939. I have written this project in `C` language because C is an industry standard. The `C` language dialect I have chosen is `C99` and I don't use dynamical memory allocation in this library. So it will work with `MISRA C` standard.

With this library, you can communicate with valves, engines, actuators, machinery, hardware and all other things that are suitable for heavy industrial mobile applications. I have build up a basic structure of the project and I hope that other users will send pull request of their `C` code for extra functionality to SAE J1939 standard because SAE J1939 is a huge standard. 

Looking for CANopen library?
https://github.com/DanielMartensson/Easy-CANopen/

Looking for a QT C++ framework implementation with Open SAE J1939 over USB? 
https://github.com/DanielMartensson/OpenSourceLogger

# Getting started

The first thing you need to know is to read the document `Open SAE J1939.pdf` inside the `Documentation` folder.
Learn the structure of the project, else you won't be able to understand SAE J1939. 
After you have got a basic understanding of the project, you are able to build on it. Keep it simple and follow the 
SAE J1939 standard!

After you have understand the structure of the project, then select processor choice in `Hardware -> Hardware.h` file.
Here you can select for example `STM32`, `Arduino`, `PIC`, `AVR` etc. or if you want to run it on PC first, then select `PROCESSOR_CHOICE 0` and run some examples.
That's the debugging mode for internal CAN feedback.

# How to use the project

 - Step 1: Download this repository
 - Step 2: Go to `Hardware -> Hardware.h` and select your processor, if it's not available, please write code for it and send me a pull request
 - Step 3: Copy over the `Src` folder to your project folder inside your IDE. Rename `Src` to for example `Open SAE J1939`. That's a good name.
 - Step 4: Use the `Examples -> Open SAE J1939 -> Startup.txt` example as your initial starting code for a SAE J1939 project.
 
```c
#include <stdlib.h>
#include <stdio.h>

/* Include Open SAE J1939 */
#include "Open_SAE_J1939/Open_SAE_J1939.h"

/* Include ISO 11783 */
#include "ISO_11783/ISO_11783-7_Application_Layer/Application_Layer.h"

int main() {

	/* Create our J1939 structure */
	J1939 j1939 = {0};

	/* Load your ECU information */
	Open_SAE_J1939_Startup_ECU(&j1939);

	while(1) {
		/* Read incoming messages */
		Open_SAE_J1939_Listen_For_Messages(&j1939);
		/* Your application code here */

	}

	return 0;
}
```
See the examples in `Examples -> SAE J1939` how to change the address, NAME or identifications for your ECU.

# The structure of the project

![a](https://raw.githubusercontent.com/DanielMartensson/Open-SAE-J1939/main/Src/Documentation/Pictures/Project%20structure.png)

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
 	- Address Not Claimed
 	- Delete Address
 
# Extra functionality
 - ISO 11783 Tractors And Machinery For Agriculture And Forestry
 	- ISO 11783-7 Implement Messages Application Layer
 		- Auxiliary Valve Command
 		- Auxiliary Valve Estimated Flow
 		- Auxiliary Valve Measured Position
 		- General Purpose Valve Command
 		- General Purpose Valve Estimated Flow
 	
# Questions and answers
 - Q: Can this library be used with `C++`?
 	- A: Yes it can be used with `C++`
 - Q: I want to build on this library, what should I do?
 	- A: First you need to know `C99` and bitwise operations. Then you need to understand the `SAE J1939:21 Transport Layer` structure. Don't forget to update the PDF with your new functionality.
 - Q: Can I use this on my Arduino?
 	- A: Yes, this `C` code is 100% pure `C` code and only using `C` standard library and also the code does not take account of what hardware you are using.
 - Q: Do I need to install the library for to use the library?
 	- A: No, just copy over the `.c` and `.h` files to your project and compile. I have used this with QT framework.
 - Q: This project is quite old now and not so much updates, is it still worth to use it?
 	- A: Yes, this library only updates when I or other includes more functionality from SAE J1939. The reason why I wrote this in `C99` is because it's an industry standard and will you will always be able to compile this library and use it.
 - Q: What is your plan with the library?
 	- A: To make SAE J1939 available for everybody
 - Q: I don't have CAN-bus, but can I use this library anyway with UART, USB, WiFi etc?
 	- A: Yes. This is only a way to shape a message in a specific way.
 - Q: Can I send data with this library, even if I don't have CAN-bus?
 	- A: Yes. There are something called DM14 transmit request, DM15 status response and DM16 binary transfer. Use that if you want to transfer data in an industrial way.
 - Q: Can I send multi package messages from from multiple ECU:s to one ECU at the same time?
 	- A: No. If you starting to send multipackages from multiple ECU:s to another ECU, then that ECU cannot understand the message. Transmit only multipackage messages one at the time if the destination address is the same.
# Issues and answers

- I: I cannot compile this library. I'm using `Keil Microvision`.
	- A: `Keil Microvision` cannot handle binary numbers such as `0b010101`. Try `STM32CubeIDE` instead because `Open SAE J1939` is made in `STM32CubeIDE`
- I: Can you provide us with some hardware examples for example `STM32`?
	- A: Yes! There is a STM32 example how to get connection with CAN-bus including an interrupt listener for messages. Go to `Examples -> Hardware` folder at look for `CAN_STM32.txt`. Also there is a `USB` example as well for `QT C++`.
