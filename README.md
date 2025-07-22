# Open SAE J1939

SAE J1939 is a protocol for shaping the CAN-bus message in a specific way that suits industrial vehicles such as tractors, machinery, trucks and more.

SAE J1939 is a very easy protocol to use, but there is a lack of information about SAE J1939, due to the cost of the protocol document, available how to shape a CAN-bus message according to SAE J1939 protocol standard. So therefore I’m writing a SAE J1939 protocol available for free to use on any embedded systems such as STM32, Arduino, AVR, PIC etc or PC.

To learn to build on this project, you need first to understand SAE J1939. I have written this project in `C` language because C is an industry standard. The `C` language dialect I have chosen is `ANSI C (C89)` and I don't use dynamical memory allocation in this library. So it will work with `MISRA C` standard.

With this library, you can communicate with valves, engines, actuators, machinery, hardware and all other things that are suitable for heavy industrial mobile applications. I have build up a basic structure of the project and I hope that other users will send pull request of their `C` code for extra functionality to SAE J1939 standard because SAE J1939 is a huge standard. 

Looking for C CANopen library for embedded systems?
https://github.com/DanielMartensson/Easy-CANopen

Looking for a C++ GUI framework that uses Open SAE J1939 over the USB?
https://github.com/DanielMartensson/GoobySoft

Looking for a C STM32 project with Open SAE J1939?
https://github.com/DanielMartensson/STM32-PLC

# Getting started

The first thing you need to know is to read my own PDF document of [Open SAE J1939](https://github.com/DanielMartensson/Open-SAE-J1939/blob/main/Src/Documentation/Open%20SAE%20J1939.pdf) inside the `Documentation` folder.
Learn the structure of the project, else you won't be able to understand SAE J1939. 
After you have got a basic understanding of the project, you are able to build on it. Keep it simple and follow the 
SAE J1939 standard!

After you have understand the structure of the project, then select processor choice in `Hardware -> Hardware.h` file.
Here you can select for example `STM32`, `Arduino`, `PIC`, `AVR` etc. or if you want to run it on PC first, then build with the define `OPENSAE_J1939_TARGET_PLATFORM` and run some examples.
That's the debugging mode for internal CAN feedback.

## Building with CMake

This project can be built with CMake. The build can be configured to target different hardware platforms.
Per default, no specific hardware platform is selected and all CAN messages are sent and received locally
over an internal memory bus. However, if you want to send messages on a specific platform, you can set the
'OPENSAE_J1939_TARGET_PLATFORM' compile definition during build. Here is an example of how to build the project using
SOCKETCAN as a targeted platform.

```bash
git clone https://github.com/DanielMartensson/Open-SAE-J1939.git
cd Open-SAE-J1939
mkdir build
cmake -B build -DOPENSAE_J1939_TARGET_PLATFORM=SOCKETCAN .
cmake --build build
```

Please feel free to contribute to this project if you add platform support for another target.

### Including the project as a library

In CMake the library is named opensaej1939.
To link it with your executable use it with
the target_link_libraries command, as an example:

```cmake
# Fetch all dependencies from git
include(FetchContent)
# Fetch Open-SAE-J1939
FetchContent_Declare(
    opensaej1939
    GIT_REPOSITORY https://github.com/DanielMartensson/Open-SAE-J1939.git
    GIT_TAG main
)
# This line actually downloads and adds the project
FetchContent_MakeAvailable(opensaej1939)

# Your executable
add_executable(your_executable main.c)

# Link against the Open SAE J1939 library
target_link_libraries(your_executable PRIVATE opensaej1939)
```

If you instead work with Git submodules in your repository, you can
include the CMakeLists.txt from this project using the add\_subdirectory command like this:

```cmake
# Add the Open-SAE-J1939 submodule as a subdirectory
add_subdirectory(external/Open-SAE-J1939)

# Your executable
add_executable(your_executable main.c)

# Link against the Open SAE J1939 library
target_link_libraries(your_executable PRIVATE opensaej1939)
```

The build can also be configurable. You can set the target platform
with the OPENSAE_J1939_TARGET_PLATFORM define, as established above.
You can also set some more detailed behaviour of the inner
workings of the stack with compile time definitions.
The project follows an embedded-target philosophy, where memory
is statically allocated at startup and does not grow dynamically at runtime.
This has a consequence when working with data transfer support.
We don't want to allocate more memory than necessary.
You can configure the number of data transfer PGNS as well as
their data sizes accordingly:


| Definition               | Description                                                                 |
|--------------------------|-----------------------------------------------------------------------------|
| `MAX_PROPRIETARY_A`      | The maximum size (in bytes) of the Proprietary A data.                     |
| `MAX_PROPRIETARY_B`      | The maximum supported size (in bytes) of a single Proprietary B data unit. |
| `MAX_PROPRIETARY_B_PGNS` | The maximum number of distinct Proprietary B PGNs that can be handled.     |

If not specifically set during the cmake build setup stage, they will attain these default values:

| Definition               | Default Value |
|--------------------------|---------------|
| `MAX_PROPRIETARY_A`      | 15U           |
| `MAX_PROPRIETARY_B`      | 60            |
| `MAX_PROPRIETARY_B_PGNS` | 2             |

As an example, this command would set up your project to use the SocketCAN backend
as well as setting the maximum number of distinct Proprietary B PGNs to 124 for your project.

```bash
cmake -B build -DOPENSAE_J1939_TARGET_PLATFORM=SOCKETCAN -DMAX_PROPRIETARY_B_PGNS=124.
```

# How to use the project

 - Step 1: Download this repository
 - Step 2: Go to `Hardware -> Hardware.h` and select your processor, if it's not available, please write code for it and send me a pull request
 - Step 3: Copy over the `Src` folder to your project folder inside your IDE. Rename `Src` to for example `Open SAE J1939`. That's a good name.
 - Step 4: Use the `Examples -> Open SAE J1939 -> Main.txt` example as your initial starting code for a SAE J1939 project.
 
```c
/*
 * Main.c
 *
 *  Created on: 16 juli 2021
 *      Author: Daniel Mårtensson
 */

#include <stdio.h>

 /* Include Open SAE J1939 */
#include "Open_SAE_J1939/Open_SAE_J1939.h"

/* Include ISO 11783 */
#include "ISO_11783/ISO_11783-7_Application_Layer/Application_Layer.h"

void Callback_Function_Send(uint32_t ID, uint8_t DLC, uint8_t data[]) {
	/* Apply your transmit layer here, e.g:
	 * uint32_t TxMailbox;
	 * static CAN_HandleTypeDef can_handler;
	 * This function transmit ID, DLC and data[] as the CAN-message.
	 * HardWareLayerCAN_TX(&can_handler, ID, DLC, data, &TxMailbox);
	 *
	 * You can use TCP/IP, USB, CAN etc. as hardware layers for SAE J1939
	 */
}

void Callback_Function_Read(uint32_t* ID, uint8_t data[], bool* is_new_data) {
	/* Apply your receive layer here, e.g:
	 * CAN_RxHeaderTypeDef rxHeader = {0};
	 * static CAN_HandleTypeDef can_handler;
	 * This function read CAN RX and give the data to ID and data[] as the CAN-message.
	 * if (HardWareLayerCAN_RX(can_handler, &rxHeader, ID, data) == STATUS_OK){
	 *	*is_new_data = true;
	 * }
	 *
	 * You can use TCP/IP, USB, CAN etc. as hardware layers for SAE J1939
	 */
}

/* This function reads the CAN traffic */
void Callback_Function_Traffic(uint32_t ID, uint8_t DLC, uint8_t data[], bool is_TX) {
	/* Print if it is TX or RX */
	printf("%s\t", is_TX ? "TX" : "RX");

	/* Print ID as hex */
	printf("%08X\t", ID);

	/* Print the data */
	uint8_t i;
	for (i = 0U; i < DLC; i++) {
		printf("%X\t", data[i]);
	}

	/* Print the non-data */
	for (i = DLC; i < 8U; i++) {
		printf("%X\t", 0U);
	}

	/* New line */
	printf("\n");
}

/* Apply your delay here */
void Callback_Function_Delay(uint8_t delay){
	/* Place your hardware delay here e.g HAL_Delay(delay); for STM32 */
}

int main() {

	/* Create our J1939 structure */
	J1939 j1939 = { 0 };

	/*
	 * Callbacks can be used if you want to pass a specific CAN-function into the hardware layer.
	 * All you need to do is to enable INTERNAL_CALLLBACK inside hardware.h
	 * If you don't want to have the traffic callback, just set the argument as NULL.
	 * If you don't want any callback at all, you can write your own hardware layer by selecting a specific processor choice at hardware.h
	 */
	CAN_Set_Callback_Functions(Callback_Function_Send, Callback_Function_Read, Callback_Function_Traffic, Callback_Function_Delay);

	/* Load your ECU information */
	Open_SAE_J1939_Startup_ECU(&j1939);

	/* SAE J1939 process */
	bool run = true;
	while (run) {
		/* Read incoming messages */
		Open_SAE_J1939_Listen_For_Messages(&j1939);

		/* Your application code here */

	}

	/* Save your ECU information */
	Open_SAE_J1939_Closedown_ECU(&j1939);

	return 0;
}
```
See the examples in `Examples -> SAE J1939` how to change the address, NAME or identifications for your ECU.

# The structure of the project

![a](https://raw.githubusercontent.com/DanielMartensson/Open-SAE-J1939/main/Src/Documentation/Pictures/Project%20structure.png)

# A Working example how the structure of Open SAE J1939 is done

This flow chart in code how Open SAE J1939 library is working. This example demonstrates how to send a request and get an answer.

- Step 1: `ECU X` is going to send a `PGN` to `ECU Y`. Interpret `PGN` as a function code.
  https://github.com/DanielMartensson/Open-SAE-J1939/blob/4297cff44107e5278f120243cb9a611eafe8c42f/Src/SAE_J1939/SAE_J1939-71_Application_Layer/Request_ECU_Identification.c#L18-L20
- Step 2: `ECU Y` is going to read that `PGN` message from `ECU X`.
  https://github.com/DanielMartensson/Open-SAE-J1939/blob/4297cff44107e5278f120243cb9a611eafe8c42f/Src/Open_SAE_J1939/Listen_For_Messages.c#L18-L32
- Step 3: The `PGN` function code will be interpreted by `ECU Y`.
  https://github.com/DanielMartensson/Open-SAE-J1939/blob/678473c13cb7eb5fe46d5aac30a53efad4ccefd9/Src/SAE_J1939/SAE_J1939-21_Transport_Layer/Request.c#L20-L52
- Step 4: The `PGN` function code is now interpreted as `ECU Identification` by `ECU Y`. Then `ECU Y` is going to broadcast the `ECU Identification` to all `ECUs`.
  - Step 4.1.1: For 1 package message, `ECU Y` is going to broadcast the `ECU Identification`. 
    https://github.com/DanielMartensson/Open-SAE-J1939/blob/678473c13cb7eb5fe46d5aac30a53efad4ccefd9/Src/SAE_J1939/SAE_J1939-71_Application_Layer/Request_ECU_Identification.c#L26-L42
  - Step 4.1.2: `ECU X` read the response from `ECU Y` because the `ECU Identification` is broadcasted.
    https://github.com/DanielMartensson/Open-SAE-J1939/blob/8faf1a542b291cb2ff71f20fceb941bb6113127b/Src/Open_SAE_J1939/Listen_For_Messages.c#L61-L62
  - Step 4.2.1: For Multi Package Message, the control byte can either be BAM or RTS. BAM is only used if you send to all `ECUs` e.g address `0xFF = 255`. But if the control byte is RTS, e.g address is not `0xFF`, then `ECU Y` is going to send a RTS and listen for a CTS response by `ECU X`. RTS is a question for "Let me know when I can transmit the message?" and CTS is response `Now you can transmit the message to me`. 
    https://github.com/DanielMartensson/Open-SAE-J1939/blob/678473c13cb7eb5fe46d5aac30a53efad4ccefd9/Src/SAE_J1939/SAE_J1939-71_Application_Layer/Request_ECU_Identification.c#L44-L58
    - Step 4.2.2: If `ECU Y` is sending a RTS, then `ECU X` will read the RTS and response with CTS back to `ECU Y`
      https://github.com/DanielMartensson/Open-SAE-J1939/blob/678473c13cb7eb5fe46d5aac30a53efad4ccefd9/Src/SAE_J1939/SAE_J1939-21_Transport_Layer/Transport_Protocol_Connection_Management.c#L22-L26
    - Step 4.2.3: Once `ECU Y` has received the CTS, then it going to transmit the data, in this case `ECU Identification`, back to `ECU X`.
      https://github.com/DanielMartensson/Open-SAE-J1939/blob/678473c13cb7eb5fe46d5aac30a53efad4ccefd9/Src/SAE_J1939/SAE_J1939-21_Transport_Layer/Transport_Protocol_Connection_Management.c#L29-L31
    - Step 4.2.3: Once `ECU Y` is sending package after package...
      https://github.com/DanielMartensson/Open-SAE-J1939/blob/8faf1a542b291cb2ff71f20fceb941bb6113127b/Src/SAE_J1939/SAE_J1939-21_Transport_Layer/Transport_Protocol_Data_Transfer.c#L84-L105
    - Step 4.2.4: Then `ECU X` is recieving each package and building up the message by knowing the `PNG` function code.
      https://github.com/DanielMartensson/Open-SAE-J1939/blob/8faf1a542b291cb2ff71f20fceb941bb6113127b/Src/SAE_J1939/SAE_J1939-21_Transport_Layer/Transport_Protocol_Data_Transfer.c#L19-L48
      And then finally implement the message.
      https://github.com/DanielMartensson/Open-SAE-J1939/blob/8faf1a542b291cb2ff71f20fceb941bb6113127b/Src/SAE_J1939/SAE_J1939-21_Transport_Layer/Transport_Protocol_Data_Transfer.c#L66-L68

# SAE J1939 functionality
 - SAE J1939:21 Transport Layer
 	- Acknowledgement
 	- Request
 	- Transport Protocol Connection Management with BAM, CTS, RTS and EOM
 	- Transport Protocol Data Transfer
 - SAE J1939:71 Application Layer
 	- Request Component Identification
 	- Request ECU Identification
 	- Request Software Identification
	- Request Proprietary A
	- Request Proprietary B
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
 	- A: First you need to know `ANSI C (C89)` and bitwise operations. Then you need to understand the `SAE J1939:21 Transport Layer` structure. Don't forget to update the PDF with your new functionality.
 - Q: Can I use this on my Arduino?
 	- A: Yes, this `C` code is 100% pure `C` code and only using `C` standard library and also the code does not take account of what hardware you are using.
 - Q: Do I need to install the library for to use the library?
 	- A: No, just copy over the `.c` and `.h` files to your project and compile. I have used this with QT framework.
 - Q: This project is quite old now and not so much updates, is it still worth to use it?
 	- A: Yes, this library only updates when I or other includes more functionality from SAE J1939. The reason why I wrote this in `ANSI C (C89)` is because it's an industry standard and you will always be able to compile this library and use it on all systems.
 - Q: What is your plan with the library?
 	- A: To make SAE J1939 available for everybody
 - Q: I don't have CAN-bus, but can I use this library anyway with UART, USB, WiFi etc?
 	- A: Yes. This is only a way to shape a message in a specific way.
 - Q: Can I send data with this library, even if I don't have CAN-bus?
 	- A: Yes. There are something called DM14 transmit request, DM15 status response and DM16 binary transfer. Use that if you want to transfer data in an industrial way.
 - Q: Can I send multi package messages from multiple ECU's to one ECU at the same time?
 	- A: No. If you starting to send multipackages from multiple ECU's to another ECU, then that ECU cannot understand the message. Transmit only multipackage messages one at the time if the destination address is the same.
 - Q: I don't want to use `ANSI C (C89)` with Open SAE J1939. Can I use the latest C standard with Open SAE J1939?
    - Yes, you can use the latest C standard with this library.
 - Q: Is it possible to compile this library onto a Windows MS-DOS or Windows 95 machine?
    - A C89 compatible compiler and an IDE and it should not be any problem
 - Q: Can I adjust the memory occupied by the program?
    - A: Yes, you can adjust the defines `MAX_PROPRIETARY_A`, `MAX_PROPRIETARY_B` and `MAX_PROPRIETARY_B_PGNS` in the file `Structs.h` according to your use case. A 'sane' default value is provided, but you can set them to the minimum if proprietary PGNs support is not needed or increase it if more PGNs need to be used.
# Issues and answers

- I: I cannot compile this library. I'm using `Keil Microvision`.
	- A: `Keil Microvision` cannot handle binary numbers such as `0b010101`. Try `STM32CubeIDE` instead because `Open SAE J1939` is made in `STM32CubeIDE`
- I: Can you provide us with some hardware examples for example `STM32`?
	- A: Yes! There is a STM32 example how to get connection with CAN-bus including an interrupt listener for messages. Go to `Examples -> Hardware` folder at look for `CAN_STM32.txt`. Also there is a `USB` example as well for `QT C++`.
