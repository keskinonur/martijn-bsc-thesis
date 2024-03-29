#ifndef _UI_H_
#define _UI_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <VP_Os/vp_os_types.h>
#include <ardrone_tool/UI/ardrone_input.h>

C_RESULT custom_reset_user_input(input_state_t* input_state, uint32_t user_input );
C_RESULT custom_update_user_input(input_state_t* input_state, uint32_t user_input );

#ifdef __cplusplus
}
#endif

#endif // _UI_H_
