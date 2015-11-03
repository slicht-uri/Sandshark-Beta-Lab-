string NAME="SandShark Dynamic Control Command"

### Vertical Mode ###
int16       VERTMODE_ELEVATOR = 3 # deg
int16       VERTMODE_PITCH    = 2 # deg
int16       VERTMODE_DEPTH    = 0 # m
int16       VERTMODE_ALTITUDE = 1 # m

int16 vertical_mode
float64     vertical_desired

### Horizontal Mode ###
int16       HORZMODE_RUDDER  = 1 # deg
int16       HORZMODE_HEADING = 0 # deg

int16 horizontal_mode
float64     horizontal_desired

### Speed Mode ###
int16       SPEEDMODE_PROP  = 0 # rad/sec (1 rad/sec ~= 9.55 RPM)
int16       SPEEDMODE_SPEED = 1 # m/s

int16       speed_mode
float64     speed_desired

