/*
 * SAE_J1939_Structs.h
 *
 *  Created on: 14 juli 2021
 *      Author: Daniel Mårtensson
 */

#ifndef SRC_STM32_PLC_SAE_J1939_SAE_J1939_SAE_J1939_STRUCTS_H_
#define SRC_STM32_PLC_SAE_J1939_SAE_J1939_SAE_J1939_STRUCTS_H_

#include "stdint.h"

/* PGN 00E800 - Storing the Acknowledgement from the reading process */
struct Acknowledgement {
	uint8_t control_byte;							/* This indicates the status of the requested information about PGN. See J1939_Enum_Control_Byte.h */
	uint8_t group_function_value;					/* The ECU function */
	uint8_t address;								/* Address from the ECU where the acknowledgement is comming from */
	uint32_t PGN_of_requested_info;					/* Information request about the PGN */
};

/* PGN 00EC00 - Storing the Transport Protocol Connection Management from the reading process */
struct TP_CM {
	uint8_t control_byte;							/* What type of message are we going to send. See See J1939_Enum_Control_Byte.h */
	uint16_t total_message_size;					/* Total bytes our complete message includes */
	uint8_t number_of_packages;						/* How many times we are going to send packages via TP_DT */
	uint32_t PGN_of_the_packeted_message;			/* Our message is going to activate a PGN, see J1939_Enum_PGN.h */
};

/* PGN 00EB00 - Storing the Transport Protocol Data Transfer from the reading process */
struct TP_DT {
	uint8_t sequence_number;						/* When this sequence number is the same as number_of_packages from TP_CM, then we have our complete message */
	uint8_t data[7][256];							/* Package data of 2D array where first index is data0 -> data6 and second index is sequence of the data */
};

/* PGN 00EE00 - Storing the Address claimed from the reading process */
struct Name {
	uint32_t identity_number;						/* Specify the ECU serial ID - 0 to 2097151 */
	uint16_t manufacturer_code;						/* Specify the ECU manufacturer code - 0 to 2047 */
	uint8_t function_instance;						/* Specify the ECU function number - 0 to 31 */
	uint8_t ECU_instance;							/* Specify the ECU number - 0 to 7 */
	uint8_t function;								/* Specify the ECU function - 0 to 255 */
	uint8_t vehicle_system;							/* Specify the type of vehicle where ECU is located - 0 to 127 */
	uint8_t arbitrary_address_capable;				/* Specify if the ECU have right to change address if addresses conflicts - 0 to 1 */
	uint8_t industry_group;							/* Specify the group where this ECU is located - 0 to 7 */
	uint8_t vehicle_system_instance;				/* Specify the vehicle system number - 0 to 15 */
};

/* PGN 00FECA - Storing the DM1 Active diagnostic trouble codes from the reading process */
struct DM1 {
	/* These are SAE lamps can have 1 = ON and 0 = OFF */
	uint8_t SAE_lamp_status_malfunction_indicator;
	uint8_t SAE_lamp_status_red_stop;
	uint8_t SAE_lamp_status_amber_warning;
	uint8_t SAE_lamp_status_protect_lamp;
	uint8_t SAE_flash_lamp_malfunction_indicator;
	uint8_t SAE_flash_lamp_red_stop;
	uint8_t SAE_flash_lamp_amber_warning;
	uint8_t SAE_flash_lamp_protect_lamp;

	/* Fault location, problem and codes */
	uint32_t SPN;									/* Location where the fault exist - Look at J1939_Enum_DM1.h */
	uint8_t FMI;									/* Type of problem - Look at J1939_Enum_DM1.h */
	uint8_t SPN_conversion_method;					/* If SPN_conversion_method = 1 that means Diagnostics Trouble Code are aligned using a newer conversion method. If SPN_conversion_method = 0 means one of the three Diagnostics Trouble Code conversion methods is used and ECU manufacture shall know which of the three methods is used */
	uint8_t occurence_count;						/* This tells how many times failure has occurred. Every time fault goes from inactive to active, the occurence_count is incremented by 1. If fault becomes active for more than 126 times the occurence_count remains 126 */
};

/* PGN 00D800 - Storing the DM15 response from the reading process */
struct DM15 {
	uint16_t number_of_allowed_bytes;				/* Todo: vad är detta? */
	uint8_t status;									/* Look for status in J1939_Enums.h */
	uint32_t error_indicator_EDC_parameter;			/* Vad är detta */
	uint8_t EDCP_extention;							/* Vad är detta */
	uint16_t seed;									/* Vad är detta */
};

/* PGN 00D700 - Storing the DM16 binary data transfer from the reading process */
struct DM16 {
	uint8_t number_of_occurences;					/* How many bytes we have sent */
	uint8_t raw_binary_data[256];					/* Here we store the bytes */
};

struct Auxiliary_valve_estimated_flow {
	uint8_t extend_estimated_flow_standard;			/* A measurement */
	uint8_t retract_estimated_flow_standard;		/* A measurement */
	uint8_t valve_state;							/* Look for valve_state in J1939_Enums.h */
	uint8_t fail_safe_mode;							/* Look for fail_safe_mode in J1939_Enums.h */
	uint8_t limit;									/* Look for limit in J1939_Enums.h */
	uint8_t exit_code;								/* Look for exit_code in J1939_Enums.h */
};

struct General_purpose_valve_estimated_flow {
	uint8_t extend_estimated_flow_standard;			/* A measurement */
	uint8_t retract_estimated_flow_standard;		/* A measurement */
	uint8_t valve_state;							/* Look for valve_state in J1939_Enums.h */
	uint8_t fail_safe_mode;							/* Look for fail_safe_mode in J1939_Enums.h */
	uint8_t limit;									/* Look for limit in J1939_Enums.h */
	uint16_t extend_estimated_flow_extended;		/* A measurement */
	uint16_t retract_estimated_flow_extended;		/* A measurement */
};

/* Storing the error codes from the reading process */
struct DM {
	uint8_t errors_dm1_active;						/* How many errors of DM1 we have right now */
	uint8_t errors_dm2_active;						/* How many errors of DM2 is active */
	struct DM1 dm1[256];							/* dm1 can contains multiple error messages */
	struct DM1 dm2[256];							/* dm2 contains previously active errors from dm1 */
	struct DM15 dm15;								/* dm15 is the memory access response from DM14 memory request */
	struct DM16 dm16;								/* dm16 is the binary data transfer after DM15 memory response (if it was proceeded) */
	/* Add more DM here */
};

/* Storing the software identification from the reading process */
struct Software_identification {
	uint8_t length_of_each_identification;			/* The length of each identification - Not part of J1939 standard */
	uint8_t number_of_fields;						/* How many numbers contains in the identifications array */
	uint8_t identifications[256];					/* This can be for example ASCII */
};

/* Storing the ECU identification from the reading process */
struct ECU_identification {
	uint8_t length_of_each_field;					/* The real length of the fields - Not part of J1939 standard */
	uint8_t ecu_part_number[256];					/* ASCII field */
	uint8_t ecu_serial_number[256];					/* ASCII field */
	uint8_t ecu_location[256];						/* ASCII field */
	uint8_t ecu_type[256];							/* ASCII field */
	uint8_t ecu_manufacturer[256];					/* ASCII field */
	uint8_t ecu_hardware_version[256];				/* ASCII field */
};

/* Storing the component identification from the reading process */
struct Component_identification {
	uint8_t length_of_each_field;					/* The real length of the fields - Not part of J1939 standard */
	uint8_t component_product_date[256];			/* ASCII field */
	uint8_t component_model_name[256];				/* ASCII field */
	uint8_t component_serial_number[256];			/* ASCII field */
	uint8_t component_unit_name[256];				/* ASCII field */
};

typedef struct {
	/* For information about other ECU */
	uint8_t number_of_ECU;
	uint8_t number_of_cannot_claim_address;
	uint8_t addresses_ECU[256];
	struct Acknowledgement acknowledgement[256];
	struct TP_CM tp_cm[256];
	struct TP_DT tp_dt[256];
	struct Name name[256];
	struct DM dm[256];
	struct Software_identification software_identification[256];
	struct ECU_identification ecu_identification[256];
	struct Component_identification component_identification[256];
	struct Auxiliary_valve_estimated_flow auxiliary_valve_estimated_flow[256][16];
	struct General_purpose_valve_estimated_flow general_purpose_valve_estimated_flow[256];

	/* For information about this ECU */
	struct Name this_name;
	uint8_t this_address;
	struct DM this_dm;
	struct Software_identification this_software_identification;
	struct ECU_identification this_ecu_identification;
	struct Component_identification this_component_identification;
	struct Auxiliary_valve_estimated_flow this_auxiliary_valve_estimated_flow[16];
	struct General_purpose_valve_estimated_flow this_general_purpose_valve_estimated_flow;
} J1939;

#endif /* SRC_STM32_PLC_SAE_J1939_SAE_J1939_SAE_J1939_STRUCTS_H_ */
