#ifndef OPENSAE_SOCKETCAN_H
#define OPENSAE_SOCKETCAN_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef SOCKETCAN_IFNAME
#define SOCKETCAN_IFNAME "can0"
#endif

#ifndef SOCKETCAN_RCVTIMEOUT
/* Set the read timeout of the bus in milliseconds, 0 means no timeout and it will block on read operations */
#define SOCKETCAN_RCVTIMEOUT 1
#endif

#include <stdint.h>

int socketcan_receive(uint32_t *ID, uint8_t data[], bool *is_new_message);
int socketcan_transmit(uint32_t ID, uint8_t data[], uint8_t DLC);
int socketcan_setup(const char *ifname);

#ifdef __cplusplus
}
#endif

#endif
