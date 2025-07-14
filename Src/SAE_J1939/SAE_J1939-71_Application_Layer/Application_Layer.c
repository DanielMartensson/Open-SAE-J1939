#ifdef __cplusplus
extern "C" {
#endif

#include "Application_Layer.h"

/* Callback functions */
void (*Callback_Function_Software_Identification)(struct Software_identification *) = 0;
void (*Callback_Function_ECU_Identification)(struct ECU_identification *) = 0;
void (*Callback_Function_Component_Identification)(struct Component_identification *) = 0;
void (*Callback_Function_Proprietary_A)(struct Proprietary_A *) = 0;
void (*Callback_Function_Proprietary_B)(struct Proprietary_B *) = 0;

void SAE_J1939_Set_Callback_Functions(
    void (*Callback_Function_Software_Identification_)(struct Software_identification *),
    void (*Callback_Function_ECU_Identification_)(struct ECU_identification *),
    void (*Callback_Function_Component_Identification_)(struct Component_identification *),
    void (*Callback_Function_Proprietary_A_)(struct Proprietary_A *),
    void (*Callback_Function_Proprietary_B_)(struct Proprietary_B *)
) {
    Callback_Function_Software_Identification = Callback_Function_Software_Identification_;
    Callback_Function_ECU_Identification = Callback_Function_ECU_Identification_;
    Callback_Function_Component_Identification = Callback_Function_Component_Identification_;
    Callback_Function_Proprietary_A = Callback_Function_Proprietary_A_;
    Callback_Function_Proprietary_B = Callback_Function_Proprietary_B_;
}

#ifdef __cplusplus
}
#endif

