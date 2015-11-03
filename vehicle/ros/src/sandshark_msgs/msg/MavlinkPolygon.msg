string NAME="Sandshark MavLink Polygon Message"

uint8       BOUNDING_POLYGON=1
uint8       AVOID_POLYGON=2
uint8       SURVEY_POLYGON=3

uint8       number_of_points
float64[]   poly_lon
float64[]   poly_lat
uint8       polygon_mode