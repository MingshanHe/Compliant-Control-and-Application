bool is_connected

bool is_firmware_ok
int32 firmware_version

bool in_protective_stop
bool in_emergency_stop
bool sto_feedback
bool is_restart_required

bool is_safety_muted
float64 max_lin_speed
float64 max_rot_speed

# Defines for filling out the mute_mask
uint8 MUTE_FRONT_RIGHT  = 1
uint8 MUTE_FRONT_CENTER = 2
uint8 MUTE_FRONT_LEFT   = 4
uint8 MUTE_LEFT_CENTER  = 8
uint8 MUTE_REAR_LEFT    = 16
uint8 MUTE_REAR_CENTER  = 32
uint8 MUTE_REAR_RIGHT   = 64
uint8 MUTE_RIGHT_CENTER = 128

uint8 MUTE_FRONT        = 7
uint8 MUTE_LEFT         = 28
uint8 MUTE_REAR         = 112
uint8 MUTE_RIGHT        = 193
uint8 MUTE_SIDES        = 221
uint8 MUTE_ALL          = 255

uint8 mute_mask
uint8 partial_mute_mask

bool is_limited_speed_active
bool is_lifter_down
bool in_sleep_mode

bool in_manual_mode
bool is_manual_mode_restart_required
