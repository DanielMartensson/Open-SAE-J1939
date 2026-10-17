#include <stdio.h>
#include <string.h>
#include "Open_SAE_J1939/Open_SAE_J1939.h"

/* A synchronous, in-memory link. No CAN interface or device is opened. */
typedef struct {
    uint32_t id;
    uint8_t data[8];
} Frame;

static Frame frames[16];
static unsigned int sent;
static unsigned int received;

static void send_frame(uint32_t id, uint8_t dlc, uint8_t data[]) {
    frames[sent].id = id;
    memset(frames[sent].data, 0, 8);
    memcpy(frames[sent].data, data, dlc);
    sent++;
}

static void read_frame(uint32_t *id, uint8_t data[], bool *is_new) {
    *is_new = received < sent;
    if (*is_new) {
        *id = frames[received].id;
        memcpy(data, frames[received].data, 8);
        received++;
    }
}

static void delay_ms(uint8_t milliseconds) {
    (void)milliseconds;
}

int main(void) {
    J1939 requester = {0};
    J1939 responder = {0};
    const uint8_t software[15] = "SAE J1939!!!";
    const uint8_t expected_eom[8] = {0x13, 16, 0, 3, 0xFF, 0xDA, 0xFE, 0};
    unsigned int i;

    requester.information_this_ECU.this_ECU_address = 0x90;
    responder.information_this_ECU.this_ECU_address = 0xA2;
    responder.information_this_ECU.this_identifications.software_identification.number_of_fields = 15;
    memcpy(responder.information_this_ECU.this_identifications.software_identification.identifications, software, 15);
    CAN_Set_Callback_Functions(send_frame, read_frame, NULL, delay_ms);

    SAE_J1939_Send_Request_Software_Identification(&requester, 0xA2);
    Open_SAE_J1939_Listen_For_Messages(&responder); /* request -> RTS */
    Open_SAE_J1939_Listen_For_Messages(&requester); /* RTS -> CTS */
    for (i = 0; i < 3; i++) {
        Open_SAE_J1939_Listen_For_Messages(&responder); /* CTS -> DT */
        Open_SAE_J1939_Listen_For_Messages(&requester); /* DT -> CTS/EOM */
    }

    if (requester.from_other_ecu_identifications.software_identification.number_of_fields != 15 ||
        memcmp(requester.from_other_ecu_identifications.software_identification.identifications, software, 15) != 0) {
        fprintf(stderr, "Software identification was not reassembled\n");
        return 1;
    }

    /* The final DT must produce EOM, not overwrite its totals with abort fields. */
    if (sent != received + 1 || frames[received].id != 0x1CECA290UL ||
        memcmp(frames[received].data, expected_eom, 8) != 0) {
        fprintf(stderr, "EOM must acknowledge 16 bytes in 3 packets\n");
        return 1;
    }

    Open_SAE_J1939_Listen_For_Messages(&responder);
    if (responder.from_other_ecu_tp_cm.total_number_of_bytes_received != 16 ||
        responder.from_other_ecu_tp_cm.total_number_of_packages_received != 3) {
        fprintf(stderr, "Sender received incorrect completion totals\n");
        return 1;
    }
    return 0;
}
