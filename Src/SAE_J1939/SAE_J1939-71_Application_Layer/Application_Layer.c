#ifdef __cplusplus
extern "C" {
#endif

#include "Application_Layer.h"

/* Callback functions */
void (*Callback_Function_Application)(SAE_Application_Info) = 0;

void SAE_J1939_Set_Application_Callback_Function(
    void (*Callback_Function)(SAE_Application_Info)
) {
    Callback_Function_Application = Callback_Function;
}

#ifdef __cplusplus
}
#endif

