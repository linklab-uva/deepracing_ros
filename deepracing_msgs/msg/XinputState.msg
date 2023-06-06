#The struct we are mimicking: 
#typedef struct _XINPUT_STATE
#{
#    DWORD                               dwPacketNumber;
#    XINPUT_GAMEPAD                      Gamepad;
#} XINPUT_STATE, *PXINPUT_STATE;
#
#DWORD (unsigned long) is 32-bit on windows, but 64-bit on unix. But w.e. this message should only ever be generated or consumed on Windows.

#Really just for timestamp, frame_id will be empty.
std_msgs/Header header

#Data corresponding to the XInput object that the Windows API spits out.
uint32 packet_number
XinputGamepad gamepad