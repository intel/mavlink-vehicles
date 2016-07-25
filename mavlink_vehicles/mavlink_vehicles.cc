/*
// Copyright (c) 2016 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
*/

#include <arpa/inet.h>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <iostream>
#include <mavlink.h>
#include <sys/socket.h>
#include <sys/types.h>

#include "mavlink_vehicles.hh"

#define DEBUG_MAVLINK (false)

#if DEBUG_MAVLINK
#define print_debug_mav(...) printf(__VA_ARGS__)
#else
#define print_debug_mav(...) ;
#endif

namespace defaults
{
const uint8_t target_system_id = 1;
const uint8_t target_component_id = 1;
const uint8_t system_id = 22;
const uint8_t component_id = 0;
const float takeoff_init_alt_m = 1.5;
const float lookat_rot_speed_degps = 90.0;
const size_t send_buffer_len = 2041;
const uint16_t remote_max_respond_time_ms = 10000;
}

namespace request_intervals_ms
{
const uint16_t home_position = 3000;
const uint16_t arm_disarm = 1000;
const uint16_t heartbeat = 1000;
const uint16_t set_mode = 1000;
const uint16_t takeoff = 1000;
}

namespace mavlink_vehicles
{

class msghandler
{
  public:
    static void handle(mav_vehicle &mav, const mavlink_message_t *msg);
};

void msghandler::handle(mav_vehicle &mav, const mavlink_message_t *msg)
{
    switch (msg->msgid) {
    case MAVLINK_MSG_ID_LOCAL_POSITION_NED: {
        mavlink_local_position_ned_t local_pos_ned;
        mavlink_msg_local_position_ned_decode(msg, &local_pos_ned);
        mav.local.timestamp = std::chrono::system_clock::now();
        mav.local.x = local_pos_ned.x;
        mav.local.y = local_pos_ned.y;
        mav.local.z = local_pos_ned.z;
        mav.local.is_new = true;
        return;
    }
    case MAVLINK_MSG_ID_ATTITUDE: {
        mavlink_attitude_t attitude;
        mavlink_msg_attitude_decode(msg, &attitude);
        mav.att.timestamp = std::chrono::system_clock::now();
        mav.att.roll = attitude.roll;
        mav.att.pitch = attitude.pitch;
        mav.att.yaw = attitude.yaw;
        mav.att.is_new = true;
        return;
    }
    case MAVLINK_MSG_ID_HOME_POSITION: {
        std::cout << "[mav_vehicle] "
                  << "Home position received." << std::endl;
        mavlink_home_position_t home_position;
        mavlink_msg_home_position_decode(msg, &home_position);
        mav.home.timestamp = std::chrono::system_clock::now();
        mav.home.lat = home_position.latitude;
        mav.home.lon = home_position.longitude;
        mav.home.alt = home_position.altitude;
        mav.home.is_new = true;
        break;
    }
    case MAVLINK_MSG_ID_GPS_RAW_INT: {
        mavlink_gps_raw_int_t gps_raw_int;
        mavlink_msg_gps_raw_int_decode(msg, &gps_raw_int);
        mav.gps = gps_raw_int.fix_type > 2 ? gps_status::FIX_2D_PLUS
                                           : gps_status::NO_FIX;
        break;
    }
    case MAVLINK_MSG_ID_HEARTBEAT: {
        mavlink_heartbeat_t hb;
        mavlink_msg_heartbeat_decode(msg, &hb);
        mav.base_mode = hb.base_mode & MAV_MODE_FLAG_GUIDED_ENABLED
                            ? mode::GUIDED
                            : mode::OTHER;
        mav.arm_stat = hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED
                           ? arm_status::ARMED
                           : arm_status::NOT_ARMED;
        mav.stat = hb.system_status == MAV_STATE_ACTIVE ? status::ACTIVE
                                                        : status::STANDBY;
        break;
    }
    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
        mavlink_global_position_int_t global_pos_int;
        mavlink_msg_global_position_int_decode(msg, &global_pos_int);
        mav.global.timestamp = std::chrono::system_clock::now();
        mav.global.lat = global_pos_int.lat;
        mav.global.lon = global_pos_int.lon;
        mav.global.alt = global_pos_int.alt;
        mav.global.is_new = true;
        break;
    }
    }
}

mav_vehicle::mav_vehicle(int socket_fd)
{
    // Store socket
    this->sock = socket_fd;
}

mav_vehicle::~mav_vehicle()
{
}

bool mav_vehicle::started()
{
    // Wait for a remote to respond
    if (!is_remote_responding()) {
        return false;
    }

    // Check if vehicle is ready
    gps_status gps = get_gps_status();
    status stat = get_status();

    if (gps != gps_status::FIX_2D_PLUS ||
        (stat != status::STANDBY && stat != status::ACTIVE)) {
        return false;
    }

    // Check if home position has been received
    if (!get_home_position_int().is_initialized()) {
        // If not, request home position
        send_cmd_long(MAV_CMD_GET_HOME_POSITION, 0, 0, 0, 0, 0, 0, 0,
                      request_intervals_ms::home_position);
        return false;
    }

    return true;
}

status mav_vehicle::get_status() const
{
    return stat;
}

arm_status mav_vehicle::get_arm_status() const
{
    return arm_stat;
}

mode mav_vehicle::get_mode() const
{
    return base_mode;
}

attitude mav_vehicle::get_attitude()
{
    attitude att_copy = this->att;
    this->att.is_new = false;
    return att_copy;
}

global_pos_int mav_vehicle::get_home_position_int()
{
    global_pos_int home_copy = this->home;
    this->home.is_new = false;
    return home_copy;
}

local_pos mav_vehicle::get_local_position_ned()
{
    local_pos local_copy = this->local;
    this->local.is_new = false;
    return local_copy;
}

global_pos_int mav_vehicle::get_global_position_int()
{
    global_pos_int global_copy = this->global;
    this->global.is_new = false;
    return global_copy;
}

gps_status mav_vehicle::get_gps_status() const
{
    return gps;
}

void mav_vehicle::send_heartbeat()
{
    using namespace std::chrono;

    cmd_custom cmd = cmd_custom::HEARTBEAT;

    // Check if cmd_custom::HEARTBEAT has been sent recently
    if (cmd_custom_timestamps.count(cmd) &&
        duration_cast<milliseconds>(system_clock::now() -
                                    cmd_custom_timestamps[cmd])
                .count() <= request_intervals_ms::heartbeat) {
        return;
    }

    // Generate heartbeat mavlink message
    mavlink_message_t mav_msg;
    mavlink_heartbeat_t mav_heartbeat;
    mav_heartbeat.type = MAV_TYPE_GCS;
    mav_heartbeat.autopilot = MAV_AUTOPILOT_INVALID;
    mav_heartbeat.base_mode = 0;
    mav_heartbeat.custom_mode = 0;
    mav_heartbeat.system_status = MAV_STATE_ACTIVE;

    // Encode and send
    uint8_t mav_data_buffer[defaults::send_buffer_len];
    mavlink_msg_heartbeat_encode(defaults::system_id, defaults::component_id,
                                 &mav_msg, &mav_heartbeat);
    int n = mavlink_msg_to_send_buffer(mav_data_buffer, &mav_msg);
    if (send_data(mav_data_buffer, n) == -1) {
        std::perror("[mav_vehicle] Error sending heartbeat");
    }

    // Update timestamp
    cmd_custom_timestamps[cmd] = system_clock::now();
}

void mav_vehicle::set_mode(mode m)
{
    using namespace std::chrono;
    time_point<system_clock> curr_time = system_clock::now();

    cmd_custom cmd = cmd_custom::SET_MODE;

    // Check if cmd_custom::SET_MODE has been sent recently
    if (cmd_custom_timestamps.count(cmd) &&
        duration_cast<milliseconds>(curr_time - cmd_custom_timestamps[cmd])
                .count() <= request_intervals_ms::set_mode) {
        return;
    }

    switch (m) {
    case mode::GUIDED: {

        // Generate set mode mavlink message
        // Arducopter does not use the standard MAV_MODE_FLAG. It uses
        // a custom mode instead. mode::GUIDED mode is defined as 4.
        mavlink_message_t mav_msg;
        mavlink_set_mode_t mav_cmd_set_mode;
        mav_cmd_set_mode.base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
        mav_cmd_set_mode.custom_mode = 4; // mode::GUIDED == 4

        // Encode and send
        uint8_t mav_data_buffer[defaults::send_buffer_len];
        mavlink_msg_set_mode_encode(defaults::system_id, defaults::component_id,
                                    &mav_msg, &mav_cmd_set_mode);
        int n = mavlink_msg_to_send_buffer(mav_data_buffer, &mav_msg);
        send_data(mav_data_buffer, n);

        // Update timestamp
        cmd_custom_timestamps[cmd] = system_clock::now();

        print_debug_mav("Changing to mode::GUIDED mode...\n");
    }
    case mode::OTHER:
        break;
    }
}

void mav_vehicle::arm_throttle()
{
    send_cmd_long(MAV_CMD_COMPONENT_ARM_DISARM, 1, 0, 0, 0, 0, 0, 0,
                  request_intervals_ms::arm_disarm);
}

void mav_vehicle::takeoff()
{
    send_cmd_long(MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0,
                  defaults::takeoff_init_alt_m, request_intervals_ms::takeoff);
}

void mav_vehicle::rotate(double angle_deg)
{
    send_cmd_long(MAV_CMD_CONDITION_YAW, fabs(angle_deg),
                  defaults::lookat_rot_speed_degps, -copysign(1, angle_deg), 1,
                  0, 0, 0, 2000);
}

void mav_vehicle::goto_waypoint(double lat, double lon, double alt)
{
    // Convert global position to mav_waypoint
    mavlink_mission_item_t mav_waypoint;

    mav_waypoint.param1 = 0;    // Hold time in decimal seconds
    mav_waypoint.param2 = 0.01; // Acceptance radius in meters
    mav_waypoint.param3 = 0;    // Radius in meters to pass through wp
    mav_waypoint.param4 = 0;    // Desired yaw angle
    mav_waypoint.x = lat;
    mav_waypoint.y = lon;
    mav_waypoint.z = alt;
    mav_waypoint.seq = 0;
    mav_waypoint.command = MAV_CMD_NAV_WAYPOINT;
    mav_waypoint.target_system = defaults::target_system_id;
    mav_waypoint.target_component = defaults::target_component_id;

    // Arducopter supports only MAV_FRAME_GLOBAL_RELATIVE_ALT.
    mav_waypoint.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
    mav_waypoint.current = 2; // Must be set as 2 for mode::GUIDED waypoint
    mav_waypoint.autocontinue = 0;

    // Encode and Send
    mavlink_message_t mav_msg;
    uint8_t mav_data_buffer[defaults::send_buffer_len];
    mavlink_msg_mission_item_encode(defaults::system_id, defaults::component_id,
                                    &mav_msg, &mav_waypoint);
    int n = mavlink_msg_to_send_buffer(mav_data_buffer, &mav_msg);
    send_data(mav_data_buffer, n);
}

ssize_t mav_vehicle::send_data(uint8_t *data, size_t len)
{
    if (!is_remote_responding()) {
        // Do not send any data if remote is not responding
        return 0;
    }

    return sendto(sock, (void *)data, len, 0,
                  (struct sockaddr *)&this->remote_addr, this->remote_addr_len);
}

void mav_vehicle::send_cmd_long(int cmd, float p1, float p2, float p3, float p4,
                                float p5, float p6, float p7, int timeout)
{
    using namespace std::chrono;
    time_point<system_clock> curr_time = system_clock::now();

    // Check if cmd_custom::HEARTBEAT has been sent recently
    if (cmd_long_timestamps.count(cmd) &&
        duration_cast<milliseconds>(curr_time - cmd_long_timestamps[cmd])
                .count() <= timeout) {
        return;
    }

    // Generate command_long mavlink message
    mavlink_message_t mav_msg;
    mavlink_command_long_t mav_cmd;
    mav_cmd.target_system = defaults::target_system_id;
    mav_cmd.target_component = defaults::target_component_id;
    mav_cmd.command = cmd;
    mav_cmd.confirmation = 0;
    mav_cmd.param1 = p1;
    mav_cmd.param2 = p2;
    mav_cmd.param3 = p3;
    mav_cmd.param4 = p4;
    mav_cmd.param5 = p5;
    mav_cmd.param6 = p6;
    mav_cmd.param7 = p7;

    // Encode and send
    static uint8_t mav_data_buffer[defaults::send_buffer_len];
    mavlink_msg_command_long_encode(defaults::system_id, defaults::component_id,
                                    &mav_msg, &mav_cmd);
    int n = mavlink_msg_to_send_buffer(mav_data_buffer, &mav_msg);
    if (send_data(mav_data_buffer, n) == -1) {
        std::perror("[mav_vehicle] Error sending cmd_long");
    }

    // Update timestamp
    cmd_long_timestamps[cmd] = system_clock::now();

    switch (cmd) {
    case MAV_CMD_COMPONENT_ARM_DISARM:
        print_debug_mav("Arming Throttle...\n");
        break;
    case MAV_CMD_NAV_TAKEOFF:
        print_debug_mav("Requesting Takeoff...\n");
        break;
    case MAV_CMD_GET_HOME_POSITION:
        std::cout << "[mav_vehicle] "
                  << "Requesting Home Position." << std::endl;
        break;
    default:
        break;
    }
}

void mav_vehicle::update()
{
    mavlink_message_t msg;
    mavlink_status_t status;

    send_heartbeat();

    uint8_t data_recv[defaults::send_buffer_len] = {0};

    struct sockaddr_storage curr_addr = {0};
    socklen_t curr_addr_len = sizeof(curr_addr);

    ssize_t bytes_recvd =
        recvfrom(sock, (void *)data_recv, defaults::send_buffer_len, 0,
                 (struct sockaddr *)&curr_addr, &curr_addr_len);

    if (bytes_recvd <= 0) {
        return;
    }

    // Check if remote address has changed
    if (((struct sockaddr_in *)&this->remote_addr)->sin_addr.s_addr !=
            ((struct sockaddr_in *)&curr_addr)->sin_addr.s_addr ||
        ((struct sockaddr_in *)&this->remote_addr)->sin_port !=
            ((struct sockaddr_in *)&curr_addr)->sin_port) {
        // Remote address is different. Update only if permanent address is not
        // responding.
        if (!is_remote_responding()) {

            // Update permanent remote address
            this->remote_addr = curr_addr;
            this->remote_addr_len = curr_addr_len;

            // Print new connection information
            struct sockaddr_in *rem = (struct sockaddr_in *)&this->remote_addr;
            std::cout << "[mav_vehicle] Connected to vehicle "
                      << inet_ntoa(rem->sin_addr) << ":" << ntohs(rem->sin_port)
                      << std::endl;
        } else {
            // Permanent remote is still responding. Ignore the message from
            // this unknown remote.
            return;
        }
    }

    // Update remote address response timestamp
    this->remote_last_respond_time = std::chrono::system_clock::now();

    print_debug_mav("Bytes Received: %d\nDatagram: ", (int)bytes_recvd);

    for (unsigned int i = 0; i < bytes_recvd; ++i) {
        print_debug_mav("%02x ", (unsigned char)data_recv[i]);

        if (mavlink_parse_char(MAVLINK_COMM_0, data_recv[i], &msg, &status)) {

            // Do not handle unexpected mavlink messages
            if (msg.sysid != defaults::target_system_id ||
                msg.compid != defaults::target_component_id) {
                continue;
            }

            print_debug_mav("\nReceived packet: CHK: %d, MGC: %d, SYS: %d, "
                            "COMP: %d, LEN: %d, MSG ID: %d, SEQ: %d\n",
                            msg.checksum, msg.magic, msg.sysid, msg.compid,
                            msg.len, msg.msgid, msg.seq);

            msghandler::handle(*this, &msg);
        }
    }
    print_debug_mav("\n");
}
bool mav_vehicle::is_remote_responding() const
{
    using namespace std::chrono;
    return duration_cast<milliseconds>(system_clock::now() -
                                       remote_last_respond_time)
               .count() <= defaults::remote_max_respond_time_ms;
}
}
