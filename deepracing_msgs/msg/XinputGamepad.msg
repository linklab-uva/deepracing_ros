#The struct we are mimicking:
#typedef struct _XINPUT_GAMEPAD
#{
#    WORD                                wButtons;
#    BYTE                                bLeftTrigger;
#    BYTE                                bRightTrigger;
#    SHORT                               sThumbLX;
#    SHORT                               sThumbLY;
#    SHORT                               sThumbRX;
#    SHORT                               sThumbRY;
#}
#All values are 16-bit or less, which makes them consistent across 64-bit platforms
uint16 buttons
uint8 left_trigger
uint8 right_trigger
int16 thumb_lx
int16 thumb_ly
int16 thumb_rx
int16 thumb_ry