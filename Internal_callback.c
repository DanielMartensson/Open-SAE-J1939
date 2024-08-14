/*-----------------------------------------------------------------------------------------------------*/

 /*
  * Main.c
  *
  *  Created on: 16 July 2021
  *      Author: Daniel Mårtensson
  */

 #include "driverlib.h"
 #include "device.h"  // Include this for the DELAY_US function on TI microcontrollers
 #include <stdio.h>
 #include <math.h>
 //#include <inttypes.h>

 /* Include Open SAE J1939 */
 #include "Open_SAE_J1939/Open_SAE_J1939.h"

 #define PI 3.14159265
 #define SAMPLING_RATE 100 // Samples per second
 #define AMPLITUDE 100      // Max amplitude of the sine wave
 #define FREQUENCY 1        // Frequency of the sine wave in Hz

 /* Every time we want to send a CAN message, this function will be called */
 void Callback_Function_Send(uint32_t ID, uint8_t DLC, uint8_t data[]) {
     printf("Callback_Function_Send called\n");
     printf("ID = 0x%X\n", ID);
     printf("DLC = %i\n", DLC);
     uint8_t i;
     for (i = 0; i < DLC; i++) {
         printf("data[%i] = 0x%X\n", i, data[i]);
     }
     printf("\n");
 }

 /* Every time we want to read a CAN message, this function will be called */
 void Callback_Function_Read(uint32_t* ID, uint8_t data[], bool* is_new_data) {
     printf("Callback_Function_Read called\n");
     *ID = 0xFF;
     uint8_t i;
     for (i = 0; i < 8; i++) {
         data[i] = 0xFF;
     }
     *is_new_data = true;
 }

 /* Apply your delay here */
 void Callback_Function_Delay(uint8_t delay){
     // Convert the delay to microseconds (assuming delay is in milliseconds)
     uint32_t delay_in_us = (uint32_t)delay * 1000;

     // Use the DELAY_US function provided by TI libraries
     DELAY_US(delay_in_us);
 }

 /* Function to generate and return a sinusoidal value */
 uint8_t Generate_Sinusoidal_Signal(uint8_t time) {
     return AMPLITUDE * sin(2 * PI * FREQUENCY * time);
 }

 uint8_t main() {

     // Initialize device clock and peripherals
     //Device_init();

     // Initialize PIE and clear PIE registers. Disables CPU interrupts.
     //Interrupt_initModule();

     // Initialize the PIE vector table with pointers to the shell Interrupt Service Routines (ISR).
     //Interrupt_initVectorTable();

     // Enable Global Interrupt (INTM) and realtime interrupt (DBGM)
     //
     //EINT;
     //ERTM;

     /* Create our J1939 structure with two ECU */
     J1939 j1939_1 = {0};

     /* !!!!! DON'T FORGET TO CHANGE THE PROCESSOR_CHOICE to INTERNAL_CALLBACK in Hardware.h !!!!! */

     /* Important to set all non-addresses to 0xFF - Else we cannot use ECU address 0x0 */
     uint8_t i;
     for (i = 0; i < 255; i++) {
         j1939_1.other_ECU_address[i] = 0xFF;
     }

     /* Set the ECU address - You can use all numbers between 0 and 253 */
     j1939_1.information_this_ECU.this_ECU_address = 30; /* From 0 to 253 because 254 = error address and 255 = broadcast address */

     /* Set the callback functions for the hardware */
     CAN_Set_Callback_Functions(Callback_Function_Send, Callback_Function_Read, NULL, Callback_Function_Delay);

     /* Main loop */
     uint8_t time = 0;
     while (1) {
         /* Generate sinusoidal signal value */
         uint8_t signal_value = Generate_Sinusoidal_Signal(time);

         /* Pack the signal value into an 8-byte J1939 message */
         uint8_t data[2] = {0};
         uint8_t number_of_occurrences = 2;
         uint16_t packed_value = (uint16_t)(signal_value + AMPLITUDE); // Offset to ensure positive values
         data[0] = (packed_value >> 8) & 0xFF; // High byte
         data[1] = packed_value & 0xFF;        // Low byte

         /* Send the CAN message using J1939 protocol */
         uint8_t DA = 50; // address of the other ECU that is going to receive the message
         SAE_J1939_Send_Binary_Data_Transfer_DM16(&j1939_1, DA, number_of_occurrences, data);

         /* Increment time for the next sample */
         time += 1.0 / SAMPLING_RATE;

         /* Apply a delay to match the sampling rate */
         Callback_Function_Delay(1);
     }

     // return 0;  // Typically not needed in embedded main functions
 }
