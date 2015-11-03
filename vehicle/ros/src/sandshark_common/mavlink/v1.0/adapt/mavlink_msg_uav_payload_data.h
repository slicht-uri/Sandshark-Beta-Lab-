// MESSAGE UAV_PAYLOAD_DATA PACKING

#include <mavlink.h>

#define MAVLINK_MSG_ID_UAV_PAYLOAD_DATA 185

typedef struct __mavlink_uav_payload_data_t
{
 int32_t lon; ///< Longitude expressed in 1E7
 int32_t lat; ///< Latitude expressed in 1E7
 int32_t alt; ///< Altitude expressed in millimeters
 int32_t depth; ///< Depth expressed in millimeters
 float fluor; ///< Fluorimetry
} mavlink_uav_payload_data_t;

#define MAVLINK_MSG_ID_UAV_PAYLOAD_DATA_LEN 20
#define MAVLINK_MSG_ID_185_LEN 20

#define MAVLINK_MSG_ID_UAV_PAYLOAD_DATA_CRC 129
#define MAVLINK_MSG_ID_185_CRC 129



#define MAVLINK_MESSAGE_INFO_UAV_PAYLOAD_DATA { \
	"UAV_PAYLOAD_DATA", \
	5, \
	{  { "lon", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_uav_payload_data_t, lon) }, \
         { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_uav_payload_data_t, lat) }, \
         { "alt", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_uav_payload_data_t, alt) }, \
         { "depth", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_uav_payload_data_t, depth) }, \
         { "fluor", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_uav_payload_data_t, fluor) }, \
         } \
}


/**
 * @brief Pack a uav_payload_data message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param lon Longitude expressed in 1E7
 * @param lat Latitude expressed in 1E7
 * @param alt Altitude expressed in millimeters
 * @param depth Depth expressed in millimeters
 * @param fluor Fluorimetry
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_uav_payload_data_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       int32_t lon, int32_t lat, int32_t alt, int32_t depth, float fluor)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_UAV_PAYLOAD_DATA_LEN];
	_mav_put_int32_t(buf, 0, lon);
	_mav_put_int32_t(buf, 4, lat);
	_mav_put_int32_t(buf, 8, alt);
	_mav_put_int32_t(buf, 12, depth);
	_mav_put_float(buf, 16, fluor);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_UAV_PAYLOAD_DATA_LEN);
#else
	mavlink_uav_payload_data_t packet;
	packet.lon = lon;
	packet.lat = lat;
	packet.alt = alt;
	packet.depth = depth;
	packet.fluor = fluor;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_UAV_PAYLOAD_DATA_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_UAV_PAYLOAD_DATA;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_UAV_PAYLOAD_DATA_LEN, MAVLINK_MSG_ID_UAV_PAYLOAD_DATA_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_UAV_PAYLOAD_DATA_LEN);
#endif
}

/**
 * @brief Pack a uav_payload_data message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param lon Longitude expressed in 1E7
 * @param lat Latitude expressed in 1E7
 * @param alt Altitude expressed in millimeters
 * @param depth Depth expressed in millimeters
 * @param fluor Fluorimetry
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_uav_payload_data_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           int32_t lon,int32_t lat,int32_t alt,int32_t depth,float fluor)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_UAV_PAYLOAD_DATA_LEN];
	_mav_put_int32_t(buf, 0, lon);
	_mav_put_int32_t(buf, 4, lat);
	_mav_put_int32_t(buf, 8, alt);
	_mav_put_int32_t(buf, 12, depth);
	_mav_put_float(buf, 16, fluor);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_UAV_PAYLOAD_DATA_LEN);
#else
	mavlink_uav_payload_data_t packet;
	packet.lon = lon;
	packet.lat = lat;
	packet.alt = alt;
	packet.depth = depth;
	packet.fluor = fluor;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_UAV_PAYLOAD_DATA_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_UAV_PAYLOAD_DATA;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_UAV_PAYLOAD_DATA_LEN, MAVLINK_MSG_ID_UAV_PAYLOAD_DATA_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_UAV_PAYLOAD_DATA_LEN);
#endif
}

/**
 * @brief Encode a uav_payload_data struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param uav_payload_data C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_uav_payload_data_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_uav_payload_data_t* uav_payload_data)
{
	return mavlink_msg_uav_payload_data_pack(system_id, component_id, msg, uav_payload_data->lon, uav_payload_data->lat, uav_payload_data->alt, uav_payload_data->depth, uav_payload_data->fluor);
}

/**
 * @brief Encode a uav_payload_data struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param uav_payload_data C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_uav_payload_data_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_uav_payload_data_t* uav_payload_data)
{
	return mavlink_msg_uav_payload_data_pack_chan(system_id, component_id, chan, msg, uav_payload_data->lon, uav_payload_data->lat, uav_payload_data->alt, uav_payload_data->depth, uav_payload_data->fluor);
}

/**
 * @brief Send a uav_payload_data message
 * @param chan MAVLink channel to send the message
 *
 * @param lon Longitude expressed in 1E7
 * @param lat Latitude expressed in 1E7
 * @param alt Altitude expressed in millimeters
 * @param depth Depth expressed in millimeters
 * @param fluor Fluorimetry
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_uav_payload_data_send(mavlink_channel_t chan, int32_t lon, int32_t lat, int32_t alt, int32_t depth, float fluor)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_UAV_PAYLOAD_DATA_LEN];
	_mav_put_int32_t(buf, 0, lon);
	_mav_put_int32_t(buf, 4, lat);
	_mav_put_int32_t(buf, 8, alt);
	_mav_put_int32_t(buf, 12, depth);
	_mav_put_float(buf, 16, fluor);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UAV_PAYLOAD_DATA, buf, MAVLINK_MSG_ID_UAV_PAYLOAD_DATA_LEN, MAVLINK_MSG_ID_UAV_PAYLOAD_DATA_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UAV_PAYLOAD_DATA, buf, MAVLINK_MSG_ID_UAV_PAYLOAD_DATA_LEN);
#endif
#else
	mavlink_uav_payload_data_t packet;
	packet.lon = lon;
	packet.lat = lat;
	packet.alt = alt;
	packet.depth = depth;
	packet.fluor = fluor;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UAV_PAYLOAD_DATA, (const char *)&packet, MAVLINK_MSG_ID_UAV_PAYLOAD_DATA_LEN, MAVLINK_MSG_ID_UAV_PAYLOAD_DATA_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UAV_PAYLOAD_DATA, (const char *)&packet, MAVLINK_MSG_ID_UAV_PAYLOAD_DATA_LEN);
#endif
#endif
}

#endif

// MESSAGE UAV_PAYLOAD_DATA UNPACKING


/**
 * @brief Get field lon from uav_payload_data message
 *
 * @return Longitude expressed in 1E7
 */
static inline int32_t mavlink_msg_uav_payload_data_get_lon(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  0);
}

/**
 * @brief Get field lat from uav_payload_data message
 *
 * @return Latitude expressed in 1E7
 */
static inline int32_t mavlink_msg_uav_payload_data_get_lat(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  4);
}

/**
 * @brief Get field alt from uav_payload_data message
 *
 * @return Altitude expressed in millimeters
 */
static inline int32_t mavlink_msg_uav_payload_data_get_alt(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Get field depth from uav_payload_data message
 *
 * @return Depth expressed in millimeters
 */
static inline int32_t mavlink_msg_uav_payload_data_get_depth(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  12);
}

/**
 * @brief Get field fluor from uav_payload_data message
 *
 * @return Fluorimetry
 */
static inline float mavlink_msg_uav_payload_data_get_fluor(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Decode a uav_payload_data message into a struct
 *
 * @param msg The message to decode
 * @param uav_payload_data C-struct to decode the message contents into
 */
static inline void mavlink_msg_uav_payload_data_decode(const mavlink_message_t* msg, mavlink_uav_payload_data_t* uav_payload_data)
{
#if MAVLINK_NEED_BYTE_SWAP
	uav_payload_data->lon = mavlink_msg_uav_payload_data_get_lon(msg);
	uav_payload_data->lat = mavlink_msg_uav_payload_data_get_lat(msg);
	uav_payload_data->alt = mavlink_msg_uav_payload_data_get_alt(msg);
	uav_payload_data->depth = mavlink_msg_uav_payload_data_get_depth(msg);
	uav_payload_data->fluor = mavlink_msg_uav_payload_data_get_fluor(msg);
#else
	memcpy(uav_payload_data, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_UAV_PAYLOAD_DATA_LEN);
#endif
}
