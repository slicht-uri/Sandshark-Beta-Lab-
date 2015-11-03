// MESSAGE ADAPT_POLYGON PACKING

#define MAVLINK_MSG_ID_ADAPT_POLYGON 190

typedef struct __mavlink_adapt_polygon_t
{
 int32_t poly_lon[4]; ///< Longitude of the four polygon, expressed in 1E7
 int32_t poly_lat[4]; ///< Latitude of the four polygon, expressed in 1E7
 int16_t poly_type; ///< Polygon mode used to interpret points, see ADAPT_POLYGON_MODE enum
} mavlink_adapt_polygon_t;

#define MAVLINK_MSG_ID_ADAPT_POLYGON_LEN 34
#define MAVLINK_MSG_ID_190_LEN 34

#define MAVLINK_MSG_ID_ADAPT_POLYGON_CRC 254
#define MAVLINK_MSG_ID_190_CRC 254

#define MAVLINK_MSG_ADAPT_POLYGON_FIELD_POLY_LON_LEN 4
#define MAVLINK_MSG_ADAPT_POLYGON_FIELD_POLY_LAT_LEN 4

#define MAVLINK_MESSAGE_INFO_ADAPT_POLYGON { \
	"ADAPT_POLYGON", \
	3, \
	{  { "poly_lon", NULL, MAVLINK_TYPE_INT32_T, 4, 0, offsetof(mavlink_adapt_polygon_t, poly_lon) }, \
         { "poly_lat", NULL, MAVLINK_TYPE_INT32_T, 4, 16, offsetof(mavlink_adapt_polygon_t, poly_lat) }, \
         { "poly_type", NULL, MAVLINK_TYPE_INT16_T, 0, 32, offsetof(mavlink_adapt_polygon_t, poly_type) }, \
         } \
}


/**
 * @brief Pack a adapt_polygon message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param poly_type Polygon mode used to interpret points, see ADAPT_POLYGON_MODE enum
 * @param poly_lon Longitude of the four polygon, expressed in 1E7
 * @param poly_lat Latitude of the four polygon, expressed in 1E7
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_adapt_polygon_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       int16_t poly_type, const int32_t *poly_lon, const int32_t *poly_lat)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ADAPT_POLYGON_LEN];
	_mav_put_int16_t(buf, 32, poly_type);
	_mav_put_int32_t_array(buf, 0, poly_lon, 4);
	_mav_put_int32_t_array(buf, 16, poly_lat, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ADAPT_POLYGON_LEN);
#else
	mavlink_adapt_polygon_t packet;
	packet.poly_type = poly_type;
	mav_array_memcpy(packet.poly_lon, poly_lon, sizeof(int32_t)*4);
	mav_array_memcpy(packet.poly_lat, poly_lat, sizeof(int32_t)*4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ADAPT_POLYGON_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_ADAPT_POLYGON;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ADAPT_POLYGON_LEN, MAVLINK_MSG_ID_ADAPT_POLYGON_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ADAPT_POLYGON_LEN);
#endif
}

/**
 * @brief Pack a adapt_polygon message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param poly_type Polygon mode used to interpret points, see ADAPT_POLYGON_MODE enum
 * @param poly_lon Longitude of the four polygon, expressed in 1E7
 * @param poly_lat Latitude of the four polygon, expressed in 1E7
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_adapt_polygon_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           int16_t poly_type,const int32_t *poly_lon,const int32_t *poly_lat)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ADAPT_POLYGON_LEN];
	_mav_put_int16_t(buf, 32, poly_type);
	_mav_put_int32_t_array(buf, 0, poly_lon, 4);
	_mav_put_int32_t_array(buf, 16, poly_lat, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ADAPT_POLYGON_LEN);
#else
	mavlink_adapt_polygon_t packet;
	packet.poly_type = poly_type;
	mav_array_memcpy(packet.poly_lon, poly_lon, sizeof(int32_t)*4);
	mav_array_memcpy(packet.poly_lat, poly_lat, sizeof(int32_t)*4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ADAPT_POLYGON_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_ADAPT_POLYGON;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ADAPT_POLYGON_LEN, MAVLINK_MSG_ID_ADAPT_POLYGON_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ADAPT_POLYGON_LEN);
#endif
}

/**
 * @brief Encode a adapt_polygon struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param adapt_polygon C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_adapt_polygon_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_adapt_polygon_t* adapt_polygon)
{
	return mavlink_msg_adapt_polygon_pack(system_id, component_id, msg, adapt_polygon->poly_type, adapt_polygon->poly_lon, adapt_polygon->poly_lat);
}

/**
 * @brief Encode a adapt_polygon struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param adapt_polygon C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_adapt_polygon_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_adapt_polygon_t* adapt_polygon)
{
	return mavlink_msg_adapt_polygon_pack_chan(system_id, component_id, chan, msg, adapt_polygon->poly_type, adapt_polygon->poly_lon, adapt_polygon->poly_lat);
}

/**
 * @brief Send a adapt_polygon message
 * @param chan MAVLink channel to send the message
 *
 * @param poly_type Polygon mode used to interpret points, see ADAPT_POLYGON_MODE enum
 * @param poly_lon Longitude of the four polygon, expressed in 1E7
 * @param poly_lat Latitude of the four polygon, expressed in 1E7
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_adapt_polygon_send(mavlink_channel_t chan, int16_t poly_type, const int32_t *poly_lon, const int32_t *poly_lat)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ADAPT_POLYGON_LEN];
	_mav_put_int16_t(buf, 32, poly_type);
	_mav_put_int32_t_array(buf, 0, poly_lon, 4);
	_mav_put_int32_t_array(buf, 16, poly_lat, 4);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ADAPT_POLYGON, buf, MAVLINK_MSG_ID_ADAPT_POLYGON_LEN, MAVLINK_MSG_ID_ADAPT_POLYGON_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ADAPT_POLYGON, buf, MAVLINK_MSG_ID_ADAPT_POLYGON_LEN);
#endif
#else
	mavlink_adapt_polygon_t packet;
	packet.poly_type = poly_type;
	mav_array_memcpy(packet.poly_lon, poly_lon, sizeof(int32_t)*4);
	mav_array_memcpy(packet.poly_lat, poly_lat, sizeof(int32_t)*4);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ADAPT_POLYGON, (const char *)&packet, MAVLINK_MSG_ID_ADAPT_POLYGON_LEN, MAVLINK_MSG_ID_ADAPT_POLYGON_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ADAPT_POLYGON, (const char *)&packet, MAVLINK_MSG_ID_ADAPT_POLYGON_LEN);
#endif
#endif
}

#endif

// MESSAGE ADAPT_POLYGON UNPACKING


/**
 * @brief Get field poly_type from adapt_polygon message
 *
 * @return Polygon mode used to interpret points, see ADAPT_POLYGON_MODE enum
 */
static inline int16_t mavlink_msg_adapt_polygon_get_poly_type(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  32);
}

/**
 * @brief Get field poly_lon from adapt_polygon message
 *
 * @return Longitude of the four polygon, expressed in 1E7
 */
static inline uint16_t mavlink_msg_adapt_polygon_get_poly_lon(const mavlink_message_t* msg, int32_t *poly_lon)
{
	return _MAV_RETURN_int32_t_array(msg, poly_lon, 4,  0);
}

/**
 * @brief Get field poly_lat from adapt_polygon message
 *
 * @return Latitude of the four polygon, expressed in 1E7
 */
static inline uint16_t mavlink_msg_adapt_polygon_get_poly_lat(const mavlink_message_t* msg, int32_t *poly_lat)
{
	return _MAV_RETURN_int32_t_array(msg, poly_lat, 4,  16);
}

/**
 * @brief Decode a adapt_polygon message into a struct
 *
 * @param msg The message to decode
 * @param adapt_polygon C-struct to decode the message contents into
 */
static inline void mavlink_msg_adapt_polygon_decode(const mavlink_message_t* msg, mavlink_adapt_polygon_t* adapt_polygon)
{
#if MAVLINK_NEED_BYTE_SWAP
	mavlink_msg_adapt_polygon_get_poly_lon(msg, adapt_polygon->poly_lon);
	mavlink_msg_adapt_polygon_get_poly_lat(msg, adapt_polygon->poly_lat);
	adapt_polygon->poly_type = mavlink_msg_adapt_polygon_get_poly_type(msg);
#else
	memcpy(adapt_polygon, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_ADAPT_POLYGON_LEN);
#endif
}
