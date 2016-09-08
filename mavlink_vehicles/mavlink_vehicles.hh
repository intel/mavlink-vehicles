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

#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <sys/socket.h>
#include <sys/types.h>
#include <unordered_map>
#include <vector>

namespace mavlink_vehicles
{

struct state_variable {
    std::chrono::time_point<std::chrono::system_clock> timestamp =
        std::chrono::system_clock::from_time_t(0);
    bool is_new = false;
    bool is_initialized()
    {
        return timestamp != std::chrono::system_clock::from_time_t(0);
    }
};

struct attitude : state_variable {
    float roll;
    float pitch;
    float yaw;
};

struct global_pos_int : state_variable {
    int32_t lat; // Degrees * 1e7
    int32_t lon; // Degrees * 1e7
    int32_t alt; // Millimeters (AMSL)
    global_pos_int() {}
    global_pos_int(int32_t _lat, int32_t _lon, int32_t _alt)
        : lat(_lat), lon(_lon), alt(_alt) {}
};

struct local_pos : state_variable {
    float x; // Meters
    float y; // Meters
    float z; // Meters
    local_pos() {}
    local_pos(float _x, float _y, float _z) : x(_x), y(_y), z(_z) {}
};

enum class status { STANDBY, ACTIVE };

enum class mode { GUIDED, AUTO, BRAKE, OTHER };

enum class arm_status { ARMED, NOT_ARMED };

enum class gps_status { NO_FIX, FIX_2D_PLUS };

enum class cmd_custom {
    HEARTBEAT,
    SET_MODE_GUIDED,
    SET_MODE_AUTO,
    SET_MODE_BRAKE,
    REQUEST_MISSION_ITEM,
    ROTATE
};

enum class mission_status {
    BRAKING,
    DETOURING,
    ROTATING,
    NORMAL
};

class msghandler;

namespace math
{
inline double rad2deg(double x);
inline double deg2rad(double x);
double dist(global_pos_int p1, global_pos_int p2);
double ground_dist(global_pos_int p1, global_pos_int p2);
double ground_dist(local_pos p1, local_pos p2);
local_pos global_to_local_ned(global_pos_int point, global_pos_int reference);
global_pos_int local_ned_to_global(local_pos point, global_pos_int reference);
double get_waypoint_rel_angle(global_pos_int wp_pos, global_pos_int ref_pos,
                              attitude ref_att);
}

class mav_vehicle
{
  public:
    mav_vehicle(int socket_fd);
    ~mav_vehicle();

    void update();
    bool is_ready();

    mode get_mode() const;
    status get_status() const;
    gps_status get_gps_status() const;
    arm_status get_arm_status() const;

    attitude get_attitude();
    local_pos get_local_position_ned();
    global_pos_int get_home_position_int();
    global_pos_int get_global_position_int();

    global_pos_int get_mission_waypoint();
    global_pos_int get_detour_waypoint();

    void takeoff();
    void arm_throttle();
    void send_heartbeat();
    void set_mode(mode m);
    void request_mission_item(uint16_t item_id);

    // Command the vehicle to immediately stop and rotate before moving to the
    // next detour or mission waypoint.
    void rotate(double angle_deg, bool autocontinue = true);
    bool is_rotation_active() const;

    // Command the vehicle to immediately take a detour through the given
    // waypoint before moving to the next mission waypoint.
    void send_detour_waypoint(double lat, double lon, double alt,
                              bool autocontinue = true,
                              bool autorotate = false);
    void send_detour_waypoint(global_pos_int global, bool autocontinue = true,
                              bool autorotate = false);
    bool is_detour_active() const;

    // Command the vehicle to go immediately to the given waypoint.
    void send_mission_waypoint(double lat, double lon, double alt,
                               bool autorotate = false);
    void send_mission_waypoint(global_pos_int global, bool autorotate = false);
    bool is_sending_mission() const;

    // Command the vehicle to brake immediately. The vehicle automatically
    // continues the mission after achieving v=0 if autocontinue is true.
    void brake(bool autocontinue);
    bool is_brake_active() const;

  private:
    status stat;
    arm_status arm_stat;
    gps_status gps;
    mode base_mode;

    attitude att;
    local_pos local;
    local_pos speed;
    global_pos_int home;
    global_pos_int global;

    global_pos_int mission_waypoint;
    uint16_t mission_waypoint_id = 0;
    bool mission_waypoint_outdated = true;

    std::vector<global_pos_int> mission_items_list;
    bool sending_mission = false;

    mission_status mstatus = mission_status::NORMAL;

    global_pos_int detour_waypoint;
    bool detour_waypoint_autocontinue = true;
    bool mission_waypoint_autorotate = false;
    bool detour_waypoint_autorotate = false;

    float rotation_goal = 0;
    float rotation_change = 0;

    bool is_our_control = false;
    bool autocontinue_after_brake = true;
    bool autocontinue_after_rotation = false;
    mission_status autocontinue_action = mission_status::NORMAL;

    std::chrono::time_point<std::chrono::system_clock> stop_time =
        std::chrono::system_clock::from_time_t(0);
    bool is_stopped();

    uint8_t system_id = 0;
    int sock = 0;
    struct sockaddr_storage remote_addr = {0};
    socklen_t remote_addr_len = sizeof(remote_addr);
    std::chrono::time_point<std::chrono::system_clock>
        remote_last_response_time = std::chrono::system_clock::from_time_t(0);
    bool remote_responding = false;
    bool is_remote_responding() const;

    std::unordered_map<int, std::chrono::time_point<std::chrono::system_clock>>
        cmd_long_timestamps;
    std::unordered_map<cmd_custom,
                       std::chrono::time_point<std::chrono::system_clock>>
        cmd_custom_timestamps;

    ssize_t send_data(uint8_t *data, size_t len);
    void send_cmd_long(int cmd, float p1, float p2, float p3, float p4,
                       float p5, float p6, float p7, int timeout);
    void send_mission_waypoint(global_pos_int wp, uint16_t seq);
    void send_mission_ack(uint8_t type);
    void set_mode(mode m, int timeout);
    void send_mission_count(int n);

    friend class msghandler;
};
}
