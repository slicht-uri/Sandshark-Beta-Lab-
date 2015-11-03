/** @file
 *	@brief MAVLink comm protocol testsuite generated from adapt.xml
 *	@see http://qgroundcontrol.org/mavlink/
 */
#ifndef ADAPT_TESTSUITE_H
#define ADAPT_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL
static void mavlink_test_common(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_adapt(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_test_common(system_id, component_id, last_msg);
	mavlink_test_adapt(system_id, component_id, last_msg);
}
#endif

#include "../common/testsuite.h"


static void mavlink_test_uav_payload_data(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_uav_payload_data_t packet_in = {
		963497464,
	}963497672,
	}963497880,
	}963498088,
	}129.0,
	};
	mavlink_uav_payload_data_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.lon = packet_in.lon;
        	packet1.lat = packet_in.lat;
        	packet1.alt = packet_in.alt;
        	packet1.depth = packet_in.depth;
        	packet1.fluor = packet_in.fluor;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_uav_payload_data_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_uav_payload_data_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_uav_payload_data_pack(system_id, component_id, &msg , packet1.lon , packet1.lat , packet1.alt , packet1.depth , packet1.fluor );
	mavlink_msg_uav_payload_data_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_uav_payload_data_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.lon , packet1.lat , packet1.alt , packet1.depth , packet1.fluor );
	mavlink_msg_uav_payload_data_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_uav_payload_data_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_uav_payload_data_send(MAVLINK_COMM_1 , packet1.lon , packet1.lat , packet1.alt , packet1.depth , packet1.fluor );
	mavlink_msg_uav_payload_data_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_adapt_polygon(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_adapt_polygon_t packet_in = {
		{ 963497464, 963497465, 963497466, 963497467 },
	}{ 963498296, 963498297, 963498298, 963498299 },
	}18899,
	};
	mavlink_adapt_polygon_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.poly_type = packet_in.poly_type;
        
        	mav_array_memcpy(packet1.poly_lon, packet_in.poly_lon, sizeof(int32_t)*4);
        	mav_array_memcpy(packet1.poly_lat, packet_in.poly_lat, sizeof(int32_t)*4);
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_adapt_polygon_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_adapt_polygon_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_adapt_polygon_pack(system_id, component_id, &msg , packet1.poly_type , packet1.poly_lon , packet1.poly_lat );
	mavlink_msg_adapt_polygon_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_adapt_polygon_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.poly_type , packet1.poly_lon , packet1.poly_lat );
	mavlink_msg_adapt_polygon_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_adapt_polygon_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_adapt_polygon_send(MAVLINK_COMM_1 , packet1.poly_type , packet1.poly_lon , packet1.poly_lat );
	mavlink_msg_adapt_polygon_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_adapt(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_test_uav_payload_data(system_id, component_id, last_msg);
	mavlink_test_adapt_polygon(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // ADAPT_TESTSUITE_H
