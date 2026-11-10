#include <stdio.h>
#include <string.h>
#include "Open_SAE_J1939/Open_SAE_J1939.h"

/* A synchronous, in-memory link. No CAN interface or device is opened. */
typedef struct {
    uint32_t id;
    uint8_t data[8];
} Frame;

static Frame frames[64];
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

/* A component identification that fits MAX_IDENTIFICATION still reassembles over TP after the length bounds. */
int main(void) {
    J1939 requester = {0};
    J1939 responder = {0};
    struct Component_identification *mine = &responder.information_this_ECU.this_identifications.component_identification;
    struct Component_identification *theirs = &requester.from_other_ecu_identifications.component_identification;
    const uint8_t length = 10;
    unsigned int i;

    requester.information_this_ECU.this_ECU_address = 0x90;
    responder.information_this_ECU.this_ECU_address = 0xA2;
    mine->length_of_each_field = length;
    memcpy(mine->component_product_date, "2026-11-10", length);
    memcpy(mine->component_model_name, "MODEL-0001", length);
    memcpy(mine->component_serial_number, "SN12345678", length);
    memcpy(mine->component_unit_name, "UNIT-NAME!", length);
    CAN_Set_Callback_Functions(send_frame, read_frame, NULL, delay_ms);

    SAE_J1939_Send_Request_Component_Identification(&requester, 0xA2);
    for (i = 0; i < 32 && received < sent; i++) {
        Open_SAE_J1939_Listen_For_Messages(&responder);
        Open_SAE_J1939_Listen_For_Messages(&requester);
    }

    if (theirs->length_of_each_field != length ||
        memcmp(theirs->component_product_date, "2026-11-10", length) != 0 ||
        memcmp(theirs->component_model_name, "MODEL-0001", length) != 0 ||
        memcmp(theirs->component_serial_number, "SN12345678", length) != 0 ||
        memcmp(theirs->component_unit_name, "UNIT-NAME!", length) != 0) {
        fprintf(stderr, "Component identification was not reassembled\n");
        return 1;
    }
    return 0;
}
