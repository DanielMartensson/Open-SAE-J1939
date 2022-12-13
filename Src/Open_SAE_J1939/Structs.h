/*
 * Structs.h
 *
 *  Created on: 14 juli 2021
 *      Author: Daniel Mårtensson
 */

#ifndef OPEN_SAE_J1939_OPEN_SAE_J1939_STRUCTS_H_
#define OPEN_SAE_J1939_OPEN_SAE_J1939_STRUCTS_H_

/* The C standard library */
#include <stdint.h>
#include <stdbool.h>

/* This text name follows 8.3 filename standard - Important if you want to save to SD card */
#define INFORMATION_THIS_ECU "ECUINFO.TXT"

/* This is the maximum size for transferring data */
#define MAX_TP_DT 1785
#define MAX_IDENTIFICATION 30
#define MAX_DM_FIELD 10

/* PGN: 0x00E800 - Storing the Acknowledgement from the reading process */
struct Acknowledgement {
	uint8_t control_byte;							/* This indicates the status of the requested information about PGN: */
	uint8_t group_function_value;					/* The function code that specify cause of the control byte e.g time out or aborted */
	uint8_t address;								/* Address from the ECU where the acknowledgement came from */
	uint32_t PGN_of_requested_info;					/* Information request about the PGN */
	uint8_t from_ecu_address;						/* From which ECU came this message */
};

/* PGN: 0x00EC00 - Storing the Transport Protocol Connection Management from the reading process */
struct TP_CM {
	uint8_t control_byte;							/* What type of message are we going to send */
	uint16_t total_message_size;					/* Total bytes our complete message includes - 9 to 1785 */
	uint8_t number_of_packages;						/* How many times we are going to send packages via TP_DT - 2 to 224 because 1785/8 is 224 rounded up */
	uint32_t PGN_of_the_packeted_message;			/* Our message is going to activate a PGN */
	uint8_t from_ecu_address;						/* From which ECU came this message */
};

/* PGN: 0x00EB00 - Storing the Transport Protocol Data Transfer from the reading process */
struct TP_DT {
	uint8_t sequence_number;						/* When this sequence number is the same as number_of_packages from TP_CM, then we have our complete message */
	uint8_t data[MAX_TP_DT];						/* This is the collected data we are going to send. Also we are using this as a filler */
	uint8_t from_ecu_address;						/* From which ECU came this message */
};

/* PGN: 0x00EE00 - Storing the Address claimed from the reading process */
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
	uint8_t from_ecu_address;						/* From which ECU came this message */
};

/* PGN: 0x00FECA - Storing the DM1 Active diagnostic trouble codes from the reading process */
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
	uint32_t SPN [MAX_DM_FIELD];						/* Location where the fault exist */
	uint8_t FMI [MAX_DM_FIELD];							/* Type of problem */
	uint8_t SPN_conversion_method [MAX_DM_FIELD];		/* If SPN_conversion_method = 1 that means Diagnostics Trouble Code are aligned using a newer conversion method. If SPN_conversion_method = 0 means one of the three Diagnostics Trouble Code conversion methods is used and ECU manufacture shall know which of the three methods is used */
	uint8_t occurrence_count [MAX_DM_FIELD];			/* This tells how many times failure has occurred. Every time fault goes from inactive to active, the occurence_count is incremented by 1. If fault becomes active for more than 126 times the occurence_count remains 126 */
	uint8_t from_ecu_address [MAX_DM_FIELD];			/* From which ECU came this message */
};

/* PGN: 0x00D800 - Storing the DM15 response from the reading process */
struct DM15 {
	uint16_t number_of_allowed_bytes;				/* How many bytes we are allowed to write or read to - 0 to 255 */
	uint8_t status;									/* Status of the response */
	uint32_t EDC_parameter;							/* Status code */
	uint8_t EDCP_extention;							/* Describe how we should interpret the EDC parameter as a status code or error code */
	uint16_t seed;									/* Response of the key if we need more key or no key at all */
	uint8_t from_ecu_address;						/* From which ECU came this message */
};

/* PGN: 0x00D700 - Storing the DM16 binary data transfer from the reading process */
struct DM16 {
	uint8_t number_of_occurences;					/* How many bytes we have sent */
	uint8_t raw_binary_data[255];					/* Here we store the bytes */
	uint8_t from_ecu_address;						/* From which ECU came this message */
};

/* Storing the error codes from the reading process */
struct DM {
	uint8_t errors_dm1_active;						/* How many errors of DM1 we have right now */
	uint8_t errors_dm2_active;						/* How many errors of DM2 is active */
	struct DM1 dm1;									/* dm1 can only hold 1 error message at the time, but we know how many errors exists */
	struct DM1 dm2;									/* dm2 contains previously active error from dm1 */
	struct DM15 dm15;								/* dm15 is the memory access response from DM14 memory request */
	struct DM16 dm16;								/* dm16 is the binary data transfer after DM15 memory response (if it was proceeded) */
	/* Add more DM here */
};

/* PGN: 0x00FEDA - Storing the software identification from the reading process */
struct Software_identification {
	uint8_t number_of_fields;						/* How many numbers contains in the identifications array */
	uint8_t identifications[MAX_IDENTIFICATION];	/* This can be for example ASCII */
	uint8_t from_ecu_address;						/* From which ECU came this message */
};

/* PGN: 0x00FDC5 - Storing the ECU identification from the reading process */
struct ECU_identification {
	uint8_t length_of_each_field;					/* The real length of the fields - Not part of J1939 standard, only for the user */
	uint8_t ecu_part_number[MAX_IDENTIFICATION];	/* ASCII field */
	uint8_t ecu_serial_number[MAX_IDENTIFICATION];	/* ASCII field */
	uint8_t ecu_location[MAX_IDENTIFICATION];		/* ASCII field */
	uint8_t ecu_type[MAX_IDENTIFICATION];			/* ASCII field */
	uint8_t from_ecu_address;						/* From which ECU came this message */
};

/* PGN: 0x00FEEB - Storing the component identification from the reading process */
struct Component_identification {
	uint8_t length_of_each_field;					/* The real length of the fields - Not part of J1939 standard, only for the user  */
	uint8_t component_product_date[MAX_IDENTIFICATION];	/* ASCII field */
	uint8_t component_model_name[MAX_IDENTIFICATION];	/* ASCII field */
	uint8_t component_serial_number[MAX_IDENTIFICATION];/* ASCII field */
	uint8_t component_unit_name[MAX_IDENTIFICATION];	/* ASCII field */
	uint8_t from_ecu_address;						/* From which ECU came this message */
};

/* Storing the identifications from the reading process */
struct Identifications {
	struct Software_identification software_identification;
	struct ECU_identification ecu_identification;
	struct Component_identification component_identification;
};

/* PGN: 0x00FE30 (65072) to 0x00FE3F (65087) */
struct Auxiliary_valve_command {
	uint8_t standard_flow;							/* Command flow */
	uint8_t fail_safe_mode;							/* If the user want the valve to go to neutral */
	uint8_t	valve_state;							/* Retract, Extend, Neutral, Init, Error etc */
	uint8_t from_ecu_address;						/* From which ECU came this message */
};

/* PGN: 0x00FE10 (65040) to 0x00FE1F (65055) */
struct Auxiliary_valve_estimated_flow {
	uint8_t extend_estimated_flow_standard;			/* A measurement */
	uint8_t retract_estimated_flow_standard;		/* A measurement */
	uint8_t valve_state;							/* Retract, Extend, Neutral, Init, Error etc */
	uint8_t fail_safe_mode;							/* The mode if we are going to use fail safe mode or not */
	uint8_t limit;									/* Enter a limit code */
	uint8_t from_ecu_address;						/* From which ECU came this message */
};

/* PGN: 0x00C400 (50176) */
struct General_purpose_valve_command {
	uint8_t standard_flow;							/* Command flow */
	uint8_t fail_safe_mode;							/* If the user want the valve to go to neutral */
	uint8_t	valve_state;							/* Retract, Extend, Neutral, Init, Error etc */
	uint16_t extended_flow;							/* Another command flow */
	uint8_t from_ecu_address;						/* From which ECU came this message */
};

/* PGN: 0x00C600 (50688) */
struct General_purpose_valve_estimated_flow {
	uint8_t extend_estimated_flow_standard;			/* A measurement */
	uint8_t retract_estimated_flow_standard;		/* A measurement */
	uint8_t valve_state;							/* Retract, Extend, Neutral, Init, Error etc */
	uint8_t fail_safe_mode;							/* The mode if we are going to use fail safe mode or not */
	uint8_t limit;									/* Enter a limit code */
	uint16_t extend_estimated_flow_extended;		/* A measurement */
	uint16_t retract_estimated_flow_extended;		/* A measurement */
	uint8_t from_ecu_address;						/* From which ECU came this message */
};

/* PGN: 0x00FF20 (65312) to 0x00FF2F (65327) */
struct Auxiliary_valve_measured_position {
	uint16_t measured_position_percent;				/* Percent position */
	uint8_t valve_state;							/* Retract, Extend, Neutral, Init, Error etc */
	uint16_t measured_position_micrometer;			/* Micrometer position */
	uint8_t from_ecu_address;						/* From which ECU came this message */
};

/* This struct is used for save information and load information from hard drive/SD-card/flash etc. due to the large size of J1939 */
typedef struct{
	struct Name this_name;
	uint8_t this_ECU_address;
	struct Identifications this_identifications;
} Information_this_ECU;

/* This struct is used for handling J1939 information */
typedef struct {
	/* Latest CAN message */
	uint32_t ID;									/* This is the CAN bus ID */
	uint8_t data[8];								/* This is the CAN bus data */
	bool ID_and_data_is_updated;					/* This is a flag that going to be set to true for every time ID and data updates */

	/* Store addresses of ECU */
	uint8_t number_of_other_ECU;				 	/* How many other ECU are connected */
	uint8_t number_of_cannot_claim_address;			/* How many ECU addresses could not claim their address */
	uint8_t other_ECU_address[255];					/* Store other ECU addresses here. Address 0xFF is the broad cast address, not an ECU address */

	/* Temporary store the information from the reading process - SAE J1939 */
	struct Name from_other_ecu_name;
	struct Acknowledgement from_other_ecu_acknowledgement;
	struct TP_CM from_other_ecu_tp_cm;
	struct TP_DT from_other_ecu_tp_dt;
	struct DM from_other_ecu_dm;
	struct Identifications from_other_ecu_identifications;

	/* Temporary hold this values for this ECU when we are going to send data */
	struct TP_CM this_ecu_tp_cm;
	struct TP_DT this_ecu_tp_dt;

	/* Temporary store the valve information from the reading process - ISO 11783-7 */
	struct Auxiliary_valve_estimated_flow from_other_ecu_auxiliary_valve_estimated_flow[16];
	struct Auxiliary_valve_measured_position from_other_ecu_auxiliary_valve_measured_position[16];
	struct General_purpose_valve_estimated_flow from_other_ecu_general_purpose_valve_estimated_flow;
	struct Auxiliary_valve_command from_other_ecu_auxiliary_valve_command[16];
	struct General_purpose_valve_command from_other_ecu_general_purpose_valve_command;

	/* For ID information about this ECU - SAE J1939 */
	Information_this_ECU information_this_ECU;
	struct DM this_dm;

	/* For valve information about this ECU - ISO 11783-7 */
	struct Auxiliary_valve_estimated_flow this_auxiliary_valve_estimated_flow[16];
	struct Auxiliary_valve_measured_position this_auxiliary_valve_measured_position[16];
	struct General_purpose_valve_estimated_flow this_general_purpose_valve_estimated_flow;

} J1939;


#endif /* OPEN_SAE_J1939_OPEN_SAE_J1939_STRUCTS_H_ */
