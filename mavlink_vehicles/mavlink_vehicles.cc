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

#include <algorithm>
#include <arpa/inet.h>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <iostream>
#include <mavlink.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <vector>

#include "mavlink_vehicles.hh"

#if PRINT_MAVLINK
#define print_mavlink(...) printf(__VA_ARGS__)
#else
#define print_mavlink(...) ;
#endif

#if VERBOSE
#define print_verbose(...) printf("[mav_vehicle] " __VA_ARGS__)
#else
#define print_verbose(...) ;
#endif

namespace defaults
{
const uint8_t target_system_id = 1;
const uint8_t target_component_id = 1;
const uint8_t system_id = 20;
const uint8_t component_id = 0;
const float takeoff_init_alt_m = 1.5;
const float lookat_rot_speed_degps = 90.0;
const size_t send_buffer_len = 2041;
const uint16_t remote_max_response_time_ms = 10000;
const double waypoint_acceptance_radius_m = 0.01;
const double detour_arrival_max_dist_m = 0.5;
const double rotation_arrival_max_dif_deg = 10;
}

namespace request_intervals_ms
{
const uint16_t request_mission_item = 3000;
const uint16_t home_position = 3000;
const uint16_t arm_disarm = 1000;
const uint16_t heartbeat = 1000;
const uint16_t set_mode = 1000;
const uint16_t takeoff = 1000;
const uint16_t rotate = 1000;
}

bool is_timedout(std::chrono::time_point<std::chrono::system_clock> timestamp,
                 uint16_t timeout_ms)
{
    using namespace std::chrono;
    return duration_cast<milliseconds>(system_clock::now() - timestamp)
               .count() > timeout_ms;
}

bool is_same_socket(sockaddr_storage &r1, sockaddr_storage &r2)
{
    return (((struct sockaddr_in *)&r1)->sin_addr.s_addr ==
                ((struct sockaddr_in *)&r2)->sin_addr.s_addr &&
            ((struct sockaddr_in *)&r1)->sin_port ==
                ((struct sockaddr_in *)&r2)->sin_port);
}

namespace mavlink_vehicles
{

namespace math
{

inline double rad2deg(double x)
{
    return (180.0 / M_PI) * x;
}

inline double deg2rad(double x)
{
    return (M_PI / 180.0) * x;
}

local_pos global_to_local_ned(global_pos_int point, global_pos_int reference)
{
    // Scaling factor to convert from degrees (fixed-point-1e7) to meters at
    // equator: (M_PI/(180*1e7))*EARTH_RADIUS_AT_EQUATOR
    const double deg_to_meters_scaling_factor = 0.011131884502145034f;

    // Compensate shrinking in earth radius as you move north or
    // shouth from the equator.
    double radius_scaling_factor = cosf(math::deg2rad(reference.lat * 1.0e-7f));

    // Subtract and scale
    local_pos local;
    local.x = (point.lat - reference.lat) * deg_to_meters_scaling_factor;
    local.y = (point.lon - reference.lon) * deg_to_meters_scaling_factor *
              radius_scaling_factor;

    // Subtract and convert from millimeters (int) to meters (double)
    local.z = (point.alt - reference.alt) * 1.0e-3f;

    // Convert from ENU to NED
    std::swap(local.x, local.y);
    local.z = -local.z;

    return local;
}

global_pos_int local_ned_to_global(local_pos point, global_pos_int reference)
{
    // Scaling factor to convert from meters to degrees (fixed-point-1e7) at
    // equator: ((180*1e7)/M_PI)/EARTH_RADIUS_AT_EQUATOR
    const double meters_to_deg_scaling_factor = 89.83204953368922f;

    // Compensate shrinking in earth radius as you move north or
    // south from the equator.
    double radius_scaling_factor = cosf(math::deg2rad(reference.lat * 1.0e-7f));

    // Convert from NED to ENU
    std::swap(point.x, point.y);
    point.z = -point.z;

    // Scale and add
    global_pos_int global;
    global.lat = point.x * meters_to_deg_scaling_factor + reference.lat;
    global.lon =
        point.y * meters_to_deg_scaling_factor / radius_scaling_factor +
        reference.lon;

    // Convert from meters (double) to millimeters (int) and sum with reference
    global.alt = point.z * 1000.0 + reference.alt;

    return global;
}

double ground_dist(global_pos_int p1, global_pos_int p2)
{
    local_pos res = global_to_local_ned(p1, p2);
    return sqrt(res.x * res.x + res.y * res.y);
}

double ground_dist(local_pos p1, local_pos p2)
{
    local_pos res(p2.x - p1.x, p2.y - p1.y, 0);
    return sqrt(res.x * res.x + res.y * res.y);
}

double dist(global_pos_int p1, global_pos_int p2)
{
    local_pos res = global_to_local_ned(p1, p2);
    return sqrt(res.x * res.x + res.y * res.y + res.z * res.z);
}
}

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
        print_verbose("Home position received\n");
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
        mav.gps = (gps_raw_int.fix_type > 2) ? gps_status::FIX_2D_PLUS
                                             : gps_status::NO_FIX;
        break;
    }
    case MAVLINK_MSG_ID_HEARTBEAT: {
        mavlink_heartbeat_t hb;
        mavlink_msg_heartbeat_decode(msg, &hb);

        // Decode mode flag ArduCopter only supports custom_modes. The
        // custom_mode value for each mode is described on ArduCopter's
        // documentation.
        mav.base_mode = mode::OTHER;

        if (!(hb.base_mode & MAV_MODE_FLAG_CUSTOM_MODE_ENABLED)) {
            break;
        }

        switch (hb.custom_mode) {
        case 3:
            mav.base_mode = mode::AUTO;
            break;
        case 4:
            mav.base_mode = mode::GUIDED;
            break;
        }

        // Decode arm status flag
        mav.arm_stat = (hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED)
                           ? arm_status::ARMED
                           : arm_status::NOT_ARMED;

        // Decode status flag
        mav.stat = (hb.system_status == MAV_STATE_ACTIVE) ? status::ACTIVE
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
    case MAVLINK_MSG_ID_MISSION_REQUEST:
    case MAVLINK_MSG_ID_MISSION_REQUEST_INT: {
        mavlink_mission_request_t mission_request;
        mavlink_msg_mission_request_decode(msg, &mission_request);

        print_verbose("MI Request received: %d\n",
                      mission_request.target_system);

        // ArduCopter broadcasts MAVLINK_MSG_ID_MISSION_REQUESTS with
        // 'target_sytem_id' set to 255, therefore it's not possible to
        // determine whether the request is aimed to us or not. This might cause
        // problems in case two instances of a vehicle are sending mission items
        // to the Copter.

        // If we are not sending a mission or if the mission item requested is
        // not within our mission size, simply ignore it.
        if (!mav.is_sending_mission ||
            mission_request.seq >= mav.mission_items_list.size()) {
            break;
        }

        // Send requested mission waypoint
        global_pos_int item = mav.mission_items_list[mission_request.seq];
        mav.send_mission_waypoint(item, mission_request.seq);
        return;
    }
    case MAVLINK_MSG_ID_MISSION_ACK: {
        mavlink_mission_ack_t ack;
        mavlink_msg_mission_ack_decode(msg, &ack);

        // A MISSION_ACK has been broadcast what means that the mission might
        // have changed, and so, our currently stored mission item might be
        // outdated.
        mav.curr_mission_item_outdated = true;

        // If the target of the ACK is our system and if we have been sending a
        // mission list, then it means that the list has been fully received by
        // the vehicle and that we can request a mission start.
        if (ack.target_system == mav.system_id && mav.is_sending_mission) {
            mav.set_mode(mode::AUTO, 0);
            mav.is_sending_mission = false;
        }
        break;
    }
    case MAVLINK_MSG_ID_MISSION_CURRENT: {
        mavlink_mission_current_t mission_current;
        mavlink_msg_mission_current_decode(msg, &mission_current);

        // If the current mission item id has changes our currently stored
        // mission item might be outdated.
        if (mav.curr_mission_item_id != mission_current.seq) {
            mav.curr_mission_item_id = mission_current.seq;
            mav.curr_mission_item_outdated = true;
        }
        break;
    }
    case MAVLINK_MSG_ID_MISSION_ITEM_INT: {
        mavlink_mission_item_int_t mission_item;
        mavlink_msg_mission_item_int_decode(msg, &mission_item);
        global_pos_int global_mission_item;

        // Ignore if we are not the target of this mission item
        if (mission_item.target_system != mav.system_id) {
            break;
        }

        // Evaluate mission frame and store mission item position with global
        // coordinates
        switch (mission_item.frame) {
        case MAV_FRAME_GLOBAL: {
            global_mission_item.lat = mission_item.x;
            global_mission_item.lon = mission_item.y;
            global_mission_item.alt = mission_item.z * 1e3;
            break;
        }
        case MAV_FRAME_GLOBAL_RELATIVE_ALT: {
            global_mission_item.lat = mission_item.x;
            global_mission_item.lon = mission_item.y;
            global_mission_item.alt = mission_item.z * 1e3 + mav.home.alt;
            break;
        }
        default: {
            // Other types of frames are not supported
            print_verbose("Received mission item with "
                          "unsupported frame num: %d",
                          (int)mission_item.frame);
            break;
        }
        }

        // Update state variable properties
        global_mission_item.timestamp = std::chrono::system_clock::now();
        global_mission_item.is_new = true;

        // Retrieve current mission item data converting from
        // global-relative-alt to global
        if (mav.curr_mission_item_id == mission_item.seq) {
            mav.curr_mission_item_pos = global_mission_item;
            mav.curr_mission_item_outdated = false;
            print_verbose("Mission waypoint updated\n");
        }
        break;
    }
    case MAVLINK_MSG_ID_COMMAND_ACK: {
        mavlink_command_ack_t command_ack;
        mavlink_msg_command_ack_decode(msg, &command_ack);

        // Check if a rotation has been initialized
        if (command_ack.command == MAV_CMD_CONDITION_YAW &&
            command_ack.result == MAV_RESULT_ACCEPTED) {
            mav.rotation_goal =
                std::fmod(mav.get_attitude().yaw - mav.rotation_goal, M_PI);
            mav.rotation_active = true;
            mav.detour_active = false;
            break;
        }
    }
    }
}

mav_vehicle::mav_vehicle(int socket_fd)
{
    // Store socket
    this->sock = socket_fd;

    // The system_id must be unique for each instance of the mav_vehicle. We
    // will use the port associated to the socket as the system_id of this
    // instance.
    struct sockaddr_storage our_addr = {0};
    socklen_t our_addr_len = sizeof(our_addr);
    if (getsockname(this->sock, (struct sockaddr *)&our_addr, &our_addr_len) ==
        -1) {
        print_verbose("The socket provided to the constructor is invalid\n");
        this->system_id = defaults::system_id;
    }
    uint16_t our_port = ntohs(((struct sockaddr_in *)&our_addr)->sin_port);

    // TODO: Since mavlink system_ids are uint8_t values while ports are
    // uint16_t, we need to calculate the remainder in order to have a valid
    // value. This might not be the best approach, since sockets associated to
    // different ports could possibly still result the same system_id to this
    // instance of mav_vehicle. We need to figure out a better way of
    // generating this system_id.
    this->system_id = our_port % UINT8_MAX;
    print_verbose("Our system id: %d %d\n", this->system_id, our_port);
    print_verbose("Waiting for vehicle...\n");
}

mav_vehicle::~mav_vehicle()
{
}

bool mav_vehicle::is_ready()
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

global_pos_int mav_vehicle::get_mission_waypoint()
{
    global_pos_int curr_mission_item_pos_copy = this->curr_mission_item_pos;
    this->curr_mission_item_pos.is_new = false;
    return curr_mission_item_pos_copy;
}

global_pos_int mav_vehicle::get_detour_waypoint()
{
    global_pos_int detour_waypoint_copy = this->detour_waypoint;
    this->detour_waypoint.is_new = false;
    return detour_waypoint_copy;
}

bool mav_vehicle::is_detour_active() const
{
    return this->detour_active;
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
        !is_timedout(cmd_custom_timestamps[cmd],
                     request_intervals_ms::heartbeat)) {
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
    mavlink_msg_heartbeat_encode(this->system_id, defaults::component_id,
                                 &mav_msg, &mav_heartbeat);
    int n = mavlink_msg_to_send_buffer(mav_data_buffer, &mav_msg);

    // Update timestamp
    cmd_custom_timestamps[cmd] = system_clock::now();

    if (send_data(mav_data_buffer, n) == -1) {
        print_verbose("Error sending heartbeat\n");
    }
}

void mav_vehicle::set_mode(mode m)
{
    set_mode(m, request_intervals_ms::set_mode);
}

void mav_vehicle::set_mode(mode m, int timeout)
{
    using namespace std::chrono;
    cmd_custom cmd = cmd_custom::SET_MODE_GUIDED;

    switch (m) {
    case mode::GUIDED: {
        cmd = cmd_custom::SET_MODE_GUIDED;
        break;
    }
    case mode::AUTO: {
        cmd = cmd_custom::SET_MODE_AUTO;
        break;
    }
    case mode::OTHER:
    default:
        print_verbose("Trying to set unsupported mode");
        return;
    }

    // Check if cmd_custom::SET_MODE_GUIDED has been sent recently
    if (cmd_custom_timestamps.count(cmd) &&
        !is_timedout(cmd_custom_timestamps[cmd],
                     request_intervals_ms::set_mode)) {
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
        mavlink_msg_set_mode_encode(this->system_id, defaults::component_id,
                                    &mav_msg, &mav_cmd_set_mode);
        int n = mavlink_msg_to_send_buffer(mav_data_buffer, &mav_msg);

        // Update timestamp
        cmd_custom_timestamps[cmd] = system_clock::now();

        if (send_data(mav_data_buffer, n) == -1) {
            print_verbose("Error changing mode to GUIDED");
            break;
        }

        print_verbose("Mode change to GUIDED\n");
        break;
    }
    case mode::AUTO: {

        // Generate set mode mavlink message
        // Arducopter does not use the standard MAV_MODE_FLAG. It uses
        // a custom mode instead. mode::GUIDED mode is defined as 4.
        mavlink_message_t mav_msg;
        mavlink_set_mode_t mav_cmd_set_mode;
        mav_cmd_set_mode.base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
        mav_cmd_set_mode.custom_mode = 3; // mode::GUIDED == 4

        // Encode and send
        uint8_t mav_data_buffer[defaults::send_buffer_len];
        mavlink_msg_set_mode_encode(this->system_id, defaults::component_id,
                                    &mav_msg, &mav_cmd_set_mode);
        int n = mavlink_msg_to_send_buffer(mav_data_buffer, &mav_msg);
        send_data(mav_data_buffer, n);

        // Update timestamp
        cmd_custom_timestamps[cmd] = system_clock::now();

        if (send_data(mav_data_buffer, n) == -1) {
            print_verbose("Error changing mode to AUTO\n");
            break;
        }

        print_verbose("Mode change to AUTO\n");
        break;
    }
    case mode::OTHER:
        break;
    }
}

void mav_vehicle::request_mission_item(uint16_t item_id)
{
    cmd_custom cmd = cmd_custom::REQUEST_MISSION_ITEM;

    // Check if cmd_custom::SET_MODE has been sent recently
    if (cmd_custom_timestamps.count(cmd) &&
        !is_timedout(cmd_custom_timestamps[cmd],
                     request_intervals_ms::request_mission_item)) {
        return;
    }

    // Generate mission request mavlink message
    mavlink_message_t mav_msg;
    mavlink_mission_request_int_t mav_mission_request;
    mav_mission_request.target_system = defaults::target_system_id;
    mav_mission_request.target_component = defaults::target_component_id;
    mav_mission_request.seq = item_id;

    // Encode and send
    uint8_t mav_data_buffer[defaults::send_buffer_len];
    mavlink_msg_mission_request_int_encode(this->system_id,
                                           defaults::component_id, &mav_msg,
                                           &mav_mission_request);
    int n = mavlink_msg_to_send_buffer(mav_data_buffer, &mav_msg);
    if (send_data(mav_data_buffer, n) == -1) {
        std::perror("Error requesting mission item");
    }

    // Update timestamp
    cmd_custom_timestamps[cmd] = std::chrono::system_clock::now();

    print_verbose("Requesting Mission waypoint\n");
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
    cmd_custom cmd = cmd_custom::ROTATE;

    // Check if cmd_custom::ROTATE has been sent recently
    if (cmd_custom_timestamps.count(cmd) &&
        !is_timedout(cmd_custom_timestamps[cmd],
                     request_intervals_ms::rotate)) {
        return;
    }

    // Set rotation goal
    this->rotation_goal = math::deg2rad(angle_deg);

    // We need to make sure mode is set as GUIDED in order to send the rotation
    // command
    set_mode(mode::GUIDED, 0);

    // Set a detour waypoint on the current position
    // send_detour_waypoint(this->global);

    // Send the rotation command immediately
    send_cmd_long(MAV_CMD_CONDITION_YAW, fabs(angle_deg),
                  defaults::lookat_rot_speed_degps, -copysign(1, angle_deg), 1,
                  0, 0, 0, 0);

    // Update timestamp
    cmd_custom_timestamps[cmd] = std::chrono::system_clock::now();

    print_verbose("Sending rotation command\n");
}

bool mav_vehicle::is_rotation_active() const
{
    return this->rotation_active;
}

void mav_vehicle::send_mission_waypoint(double lat, double lon, double alt)
{
    global_pos_int wp;
    wp.lat = lat * 1e7;
    wp.lon = lon * 1e7;
    wp.alt = alt * 1e3 + double(this->home.alt);

    send_mission_waypoint(wp);
}

void mav_vehicle::send_mission_waypoint(global_pos_int wp)
{
    // We need to toggle from AUTO to GUIDED in order to update the mission
    // waypoints
    set_mode(mode::GUIDED, 0);

    // Ardupilot reserves the first element of the mission commands list
    // (seq=0) to the home position. Sending a mission item with that sequence
    // number does not overwrite the home position. Ardupilot simply ignores
    // it. For that reason, the first waypoint (seq=0) must be a mock waypoint
    // that will never be used and our waypoints must start with seq=1
    this->mission_items_list.resize(0);
    this->mission_items_list.push_back(wp); // Mock waypoint. Will be ignored.
    this->mission_items_list.push_back(wp); // Real target waypoint.

    // Send the mission_count command to Ardupilot so that it prepares to
    // request the mission items one by one.
    send_mission_count(2);

    // TODO: These flags preferably should be set on the CMD_ACK of this
    // command and not here.
    this->detour_active = false;
    this->rotation_active = false;

    print_verbose("Mission started\n");

    this->is_sending_mission = true;
}

void mav_vehicle::send_mission_waypoint(global_pos_int wp, uint16_t seq)
{
    mavlink_mission_item_int_t mav_waypoint;

    // Set mavlink message attributes
    mav_waypoint.command = MAV_CMD_NAV_WAYPOINT;
    mav_waypoint.target_system = defaults::target_system_id;
    mav_waypoint.target_component = defaults::target_component_id;

    // Arducopter supports only MAV_FRAME_GLOBAL_RELATIVE_ALT.
    mav_waypoint.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;

    // Set coordinates in Degrees * 1e7 according to frame and message type
    mav_waypoint.x = wp.lat;
    mav_waypoint.y = wp.lon;

    // Set relative altitude in meters according to frame
    mav_waypoint.z = (wp.alt - double(this->home.alt)) / 1e3f;

    // Set sequence number
    mav_waypoint.seq = seq;

    // Set params
    mav_waypoint.param1 = 0;       // Hold time in decimal seconds
    mav_waypoint.param2 = 0.01;    // Acceptance radius in meters
    mav_waypoint.param3 = 0;       // Radius in meters to pass through wp
    mav_waypoint.param4 = 0;       // Desired yaw angle at arrival
    mav_waypoint.current = 0;      // Unused
    mav_waypoint.autocontinue = 0; // Unused

    // Encode and Send
    mavlink_message_t mav_msg;
    uint8_t mav_data_buffer[defaults::send_buffer_len];
    mavlink_msg_mission_item_int_encode(this->system_id, defaults::component_id,
                                        &mav_msg, &mav_waypoint);
    int n = mavlink_msg_to_send_buffer(mav_data_buffer, &mav_msg);
    send_data(mav_data_buffer, n);
}

void mav_vehicle::send_detour_waypoint(double lat, double lon, double alt)
{
    global_pos_int wp;
    wp.lat = lat * 1e7;
    wp.lon = lon * 1e7;
    wp.alt = alt * 1e3 + double(this->home.alt);

    send_detour_waypoint(wp);
}

void mav_vehicle::send_detour_waypoint(global_pos_int wp)
{
    mavlink_mission_item_t mav_waypoint;

    // Request change to guided mode
    set_mode(mode::GUIDED, 0);

    // Set mavlink message attributes
    mav_waypoint.command = MAV_CMD_NAV_WAYPOINT;
    mav_waypoint.target_system = defaults::target_system_id;
    mav_waypoint.target_component = defaults::target_component_id;

    // Arducopter supports only MAV_FRAME_GLOBAL_RELATIVE_ALT.
    mav_waypoint.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;

    // Set coordinates in Degrees according to frame and message type
    mav_waypoint.x = double(wp.lat) / 1e7f;
    mav_waypoint.y = double(wp.lon) / 1e7f;

    // Set relative altitude in meters according to frame
    mav_waypoint.z = double(wp.alt - this->home.alt) / 1e3f;

    // Set params
    mav_waypoint.seq = 0;    // Unused
    mav_waypoint.param1 = 0; // Hold time in decimal seconds
    mav_waypoint.param2 = defaults::waypoint_acceptance_radius_m;
    mav_waypoint.param3 = 0;       // Radius in meters to pass through wp
    mav_waypoint.param4 = 0;       // Desired yaw angle
    mav_waypoint.current = 2;      // Must be set as 2 for GUIDED waypoint
    mav_waypoint.autocontinue = 0; // Unused

    // Encode and Send
    mavlink_message_t mav_msg;
    uint8_t mav_data_buffer[defaults::send_buffer_len];
    mavlink_msg_mission_item_encode(this->system_id, defaults::component_id,
                                    &mav_msg, &mav_waypoint);
    int n = mavlink_msg_to_send_buffer(mav_data_buffer, &mav_msg);
    send_data(mav_data_buffer, n);

    // Store detour waypoint
    this->detour_waypoint = wp;

    // Disable flags
    this->rotation_active = false;

    // Enable detour flag
    this->detour_active = true;

    print_verbose("Detour started\n");
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

    // Check if the command long stored in cmd has been sent recently
    if (cmd_long_timestamps.count(cmd) &&
        !is_timedout(cmd_long_timestamps[cmd], timeout)) {
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
    mavlink_msg_command_long_encode(this->system_id, defaults::component_id,
                                    &mav_msg, &mav_cmd);
    int n = mavlink_msg_to_send_buffer(mav_data_buffer, &mav_msg);

    // Update timestamp
    cmd_long_timestamps[cmd] = system_clock::now();

    if (send_data(mav_data_buffer, n) == -1) {
        print_verbose("Error sending command long\n");
        return;
    }

    switch (cmd) {
    case MAV_CMD_COMPONENT_ARM_DISARM:
        print_verbose("Arm throttle command sent\n");
        break;
    case MAV_CMD_NAV_TAKEOFF:
        print_verbose("Takeoff command sent\n");
        break;
    case MAV_CMD_GET_HOME_POSITION:
        print_verbose("Requested home position\n");
        break;
    default:
        break;
    }
}

void mav_vehicle::update()
{
    send_heartbeat();

    // Check if remote is still responding
    if (is_remote_responding() &&
        is_timedout(this->remote_last_response_time,
                    defaults::remote_max_response_time_ms)) {
        print_verbose("Connection to vehicle lost\n");
        print_verbose("Waiting for vehicle...\n");
        this->remote_responding = false;
    }

    // Try to read from remote
    uint8_t data_recv[defaults::send_buffer_len] = {0};
    struct sockaddr_storage curr_addr = {0};
    socklen_t curr_addr_len = sizeof(curr_addr);
    ssize_t bytes_recvd =
        recvfrom(sock, (void *)data_recv, defaults::send_buffer_len, 0,
                 (struct sockaddr *)&curr_addr, &curr_addr_len);

    if (bytes_recvd <= 0) {
        return;
    }

    // If data has been received, check if remote address has changed
    if (!is_same_socket(this->remote_addr, curr_addr)) {
        // Remote address is different. Replace remote address only if
        // permanent address is not responding, otherwise, ignore this message.
        if (is_remote_responding()) {
            return;
        }
        // Update permanent remote address
        this->remote_addr = curr_addr;
        this->remote_addr_len = curr_addr_len;

        // Print new connection information
        print_verbose(
            "Connected to vehicle %s:%d\n",
            inet_ntoa(((struct sockaddr_in *)&this->remote_addr)->sin_addr),
            ntohs(((struct sockaddr_in *)&this->remote_addr)->sin_port));
    }

    // Update remote address response timestamp
    this->remote_last_response_time = std::chrono::system_clock::now();
    this->remote_responding = true;

    // Parse mavlink message
    mavlink_message_t msg;
    mavlink_status_t status;
    print_mavlink("Bytes Received: %d\nDatagram: ", (int)bytes_recvd);
    for (unsigned int i = 0; i < bytes_recvd; ++i) {
        print_mavlink("%02x ", (unsigned char)data_recv[i]);

        if (!mavlink_parse_char(MAVLINK_COMM_0, data_recv[i], &msg, &status)) {
            continue;
        }

        // Do not handle unexpected mavlink messages
        if (msg.sysid != defaults::target_system_id ||
            msg.compid != defaults::target_component_id) {
            continue;
        }

        print_mavlink("\nReceived packet: CHK: %d, MGC: %d, SYS: %d, "
                      "COMP: %d, LEN: %d, MSG ID: %d, SEQ: %d\n",
                      msg.checksum, msg.magic, msg.sysid, msg.compid, msg.len,
                      msg.msgid, msg.seq);

        msghandler::handle(*this, &msg);
    }

    print_mavlink("\n");

    // Request home position if it has not been received
    if (!get_home_position_int().is_initialized()) {
        send_cmd_long(MAV_CMD_GET_HOME_POSITION, 0, 0, 0, 0, 0, 0, 0,
                      request_intervals_ms::home_position);
    }

    // Check if mission has changed
    if (this->curr_mission_item_outdated) {
        request_mission_item(this->curr_mission_item_id);
    }

    // Check if a detour has been finished in order to continue the mission
    if (is_detour_active() &&
        fabs(math::dist(detour_waypoint, get_global_position_int())) <=
            defaults::detour_arrival_max_dist_m) {

        // Finish detour
        detour_active = false;

        // Get back to AUTO mode to continue the mission
        set_mode(mode::AUTO, 0);

        print_verbose("Detour finished\n");
    }

    // Check if a rotation has been finished in order to continue the mission
    if (is_rotation_active() &&
        (fabs(get_attitude().yaw - this->rotation_goal) <=
         math::deg2rad(defaults::rotation_arrival_max_dif_deg))) {

        // Finish rotation
        rotation_active = false;

        // Get back to AUTO mode to continue the mission
        set_mode(mode::AUTO, 0);

        print_verbose("Rotation finished\n");
    }
}

bool mav_vehicle::is_remote_responding() const
{
    return remote_responding;
}

void mav_vehicle::send_mission_ack(uint8_t type)
{
    mavlink_mission_ack_t ack;

    ack.type = type;
    ack.target_system = defaults::target_system_id;
    ack.target_component = defaults::target_component_id;

    // Encode and Send
    mavlink_message_t mav_msg;
    uint8_t mav_data_buffer[defaults::send_buffer_len];
    mavlink_msg_mission_ack_encode(this->system_id, defaults::component_id,
                                   &mav_msg, &ack);
    int n = mavlink_msg_to_send_buffer(mav_data_buffer, &mav_msg);
    send_data(mav_data_buffer, n);

    print_verbose("Mission ack sent: %d\n", type);
}

void mav_vehicle::send_mission_count(int c)
{
    mavlink_mission_count_t count;

    count.count = c;
    count.target_system = defaults::target_system_id;
    count.target_component = defaults::target_component_id;

    // Encode and Send
    mavlink_message_t mav_msg;
    uint8_t mav_data_buffer[defaults::send_buffer_len];
    mavlink_msg_mission_count_encode(this->system_id, defaults::component_id,
                                     &mav_msg, &count);
    int n = mavlink_msg_to_send_buffer(mav_data_buffer, &mav_msg);
    send_data(mav_data_buffer, n);
}
}

