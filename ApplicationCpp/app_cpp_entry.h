#ifndef ROBOT_APP_CPP_ENTRY_H
#define ROBOT_APP_CPP_ENTRY_H

#ifdef __cplusplus
extern "C" {
#endif

void AppCpp_Initialize(void);
void AppCpp_ProcessFrame(void);
void AppCpp_ControlTask(void const* argument);

#ifdef __cplusplus
}
#endif

#endif /* ROBOT_APP_CPP_ENTRY_H */
