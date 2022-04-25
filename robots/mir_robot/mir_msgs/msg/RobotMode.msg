# The robot operates in different mode
uint8 ROBOT_MODE_NONE = 0		# start mode
uint8 ROBOT_MODE_MAPPING = 3		# in mapping a new map is made
uint8 ROBOT_MODE_MAPPING_FINALIZING = 4		# in mapping the recorded map is being finalised
uint8 ROBOT_MODE_MISSION = 7		# primary mode when executing a mission (action list)
uint8 ROBOT_MODE_CHANGING = 255		# a transition mode - to say that a transition is in progress

uint8 robotMode
string robotModeString
