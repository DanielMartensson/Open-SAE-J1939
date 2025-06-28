/*
 * SocketCAN_Transmit_Receive.c
 *
 *  Created on: 24 june 2025
 *      Author: Rickard Hallerbäck
 */


#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <errno.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>

#include "SocketCAN.h"

// Global variables
int can_socket = -1;
bool can_initialized = false;

// Function to initialize SocketCAN
int socketcan_setup(const char *ifname) {
    struct ifreq ifr;
    struct sockaddr_can addr;

    if (can_initialized) {
        return 0; // Already initialized
    }

    // Create the socket
    can_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_socket < 0) {
        perror("Error while opening CAN socket");
        return -1;
    }

    // Disable reception of own messages
    int disable_loopback = 0;
    if (setsockopt(can_socket, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &disable_loopback, sizeof(disable_loopback)) < 0) {
        perror("Error disabling loopback");
        close(can_socket);
        return -1;
    }

    // Specify the CAN interface
    strcpy(ifr.ifr_name, ifname);
    if (ioctl(can_socket, SIOCGIFINDEX, &ifr) < 0) {
        fprintf(stderr, "Error getting interface index for '%s'", ifname);
        close(can_socket);
        can_socket = -1;
        return -1;
    }

    // Bind the socket to the CAN interface
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(can_socket, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        fprintf(stderr, "Error binding CAN socket for '%s'", ifname);
        close(can_socket);
        can_socket = -1;
        return -1;
    }

    can_initialized = true;
    return 0;
}

bool socketcan_is_initialized() {
  return can_initialized;
}

// Transmit a CAN message
int socketcan_transmit(uint32_t ID, uint8_t data[], uint8_t DLC) {
    if (!socketcan_is_initialized()) {
      socketcan_setup(SOCKETCAN_IFNAME);
    }

    if (!can_initialized || can_socket < 0) {
        return -1;
    }

    struct can_frame frame;
    memset(&frame, 0, sizeof(frame));

    frame.can_id = ID | CAN_EFF_FLAG;
    frame.can_dlc = DLC;

    memcpy(frame.data, data, DLC > 8 ? 8 : DLC);

    int bytes_sent = write(can_socket, &frame, sizeof(struct can_frame));

    return (bytes_sent == sizeof(frame)) ? 0 : -1;
}

// Receive a CAN message (non-blocking recommended)
int socketcan_receive(uint32_t* ID, uint8_t data[], bool* is_new_message) {
    if (!socketcan_is_initialized()) {
        socketcan_setup(SOCKETCAN_IFNAME);
    }

    if (!can_initialized || can_socket < 0) {
        *is_new_message = false;
        return -1;
    }

    struct can_frame frame;
    ssize_t bytes_read = read(can_socket, &frame, sizeof(struct can_frame));

    if (bytes_read < (ssize_t)sizeof(struct can_frame)) {
        *is_new_message = false;
        return -1;
    }

    // Extract the ID, masking out flags
    if (frame.can_id & CAN_EFF_FLAG) {
        *ID = frame.can_id & CAN_EFF_MASK;  // 29-bit ID
    } else {
        *ID = frame.can_id & CAN_SFF_MASK;  // 11-bit ID
    }

    // Copy up to 8 bytes of data
    uint8_t len = (frame.can_dlc <= 8) ? frame.can_dlc : 8;
    memcpy(data, frame.data, len);

    *is_new_message = true;
    return 0;
}
