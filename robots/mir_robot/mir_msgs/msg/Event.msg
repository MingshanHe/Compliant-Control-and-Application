uint32 EV_SPEED=1
uint32 EV_BLINK=2
uint32 EV_SOUND=3
uint32 EV_DOOR=4
uint32 EV_AMCLOFF=5
uint32 EV_FWDDIST=6
uint32 EV_IO=7
uint32 EV_FLEETLCK=8	# Fleet
uint32 EV_EMERGENCY=9	# Fleet
uint32 eventType	# The area event type
string area_guid	# The area unique identifier
string area_name	# The name of the area
geometry_msgs/Point[] polygon # An array of corner points that define the edges of the area
