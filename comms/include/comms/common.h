/*
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
    This file contains global IO structs, common definitions, and common functions that much of ROS Serial depends on.
    Notes:
        *RPI_IN_MSG --> RPI_IN_LENG?
        *PICO_IN_MSG --> PICO_IN_LENG?
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
*/

/**
 * <comms/common.h>
 *
 * @brief global IO structs, common definitions, common functions ROS Serial depends on
 * @addtogroup Common
 * @ingroup Comms
 * @{
 */

#include <stdio.h>
#include <stdint.h>
// #include <memory.h>
#include <string.h>
#include <pico/binary_info.h>

#ifndef COMMONS_H
#define COMMONS_H

// definitions
#define SYNC_FLAG       0xff //beginning of packet sync flag
#define VERSION_FLAG    0xfe //version flag compatible with ROS2

#define ROS_HEADER_LENGTH 7
#define ROS_FOOTER_LENGTH 1
#define ROS_PKG_LENGTH  (ROS_HEADER_LENGTH + ROS_FOOTER_LENGTH) //length (in bytes) of ros packaging (header and footer)

// specific checksum method as defined by http://wiki.ros.org/rosserial/Overview/Protocol
uint8_t checksum(uint8_t* addends, int len);

// converts an array of four uint8_t members to an int32_t
int32_t bytes_to_int32(uint8_t bytes[4]);

// converts an int32_t to an array of four uint8_t members
uint8_t* int32_to_bytes(int32_t i32t);

// decodes a complete ROS packet into bytes array 'MSG' and uint16 topic as defined by http://wiki.ros.org/rosserial/Overview/Protocol
int decode_rospkt(uint8_t* ROSPKT, int rospkt_len, uint16_t* TOPIC, uint8_t* MSG, int msg_len);

// encodes a message and topic into a bytes array 'ROSPKT' as defined by http://wiki.ros.org/rosserial/Overview/Protocol
int encode_msg(uint8_t* MSG, int msg_len, uint16_t TOPIC, uint8_t* ROSPKT, int rospkt_len);

#endif