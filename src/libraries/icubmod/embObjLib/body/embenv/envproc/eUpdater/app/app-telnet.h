

// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _APPTELNET_H_
#define _APPTELNET_H_

extern const char apptelnet_msg_login[];

extern const char apptelnet_msg_welcome[];

extern const char apptelnet_prompt[];


extern uint16_t apptelnet_onexecsafe(const char *cmd, char *rep, uint8_t *quitflag);

extern uint16_t apptelnet_onexecupdater(const char *cmd, char *rep, uint8_t *quitflag);

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------




