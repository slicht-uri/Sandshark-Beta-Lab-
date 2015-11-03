string NAME="Sandshark Navigation"

Header header

string      gmt_timestamp
float64     epoch     # ???
float64     latitude  # rad
float64     longitude # rad
float64     northings # m
float64     eastings  # m
int32       utm_zone
float64     depth     # m
float64     altitude  # m
float64     speed     # m/s
float64     gps_speed  # m/s
float64     bearing   # rad
float64     pitch     # rad
float64     roll      # rad

int32 quality
time computed_time

bool is_valid
string invalid_reason